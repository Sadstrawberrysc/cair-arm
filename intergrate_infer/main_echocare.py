import os
import json
import time
import threading
import cv2
import numpy as np
import torch
import torch.nn.functional as F
import segmentation_models_pytorch as smp
import redis
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as T
import torch.backends.cudnn as cudnn

# 确保你在 models/__init__.py 或对应的文件中导出了 SwinPoseHead
from utils import rot6d_to_matrix
from models import (
    models_mae,
    SwinTransformer,
    SwinTransformerV2,
    PoseHead,
    PoseClsHead,
    PoseClsHead2,
    PhaseHead,  
    RESNetHead,
    SwinPoseHead  # [NEW] 导入我们新写的 Swin 预测头
)
from utils import (
    build_inv_normalizer_from_dataset,
)

# Global variables for thread communication
redis_client = None
redis_pubsub = None
camera = None
inference_running = True
trigger_inference = False  # Flag to trigger inference manually

# Phase Label Map
PHASE_LABELS = {0: 'Scan', 1: 'Trigger', 2: 'Action'}

def colorize_mask(mask, color_map):
    color_mask = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    for class_id, color in enumerate(color_map):
        color_mask[mask == class_id] = color
    return color_mask

def publish_command(redis_client, y_value, rz_value, terminate_value, phase_idx=None, robot_action_state="idle"):
    """Publish command to Redis channel"""
    try:
        command_data = {
            "parameters": {
                "y": float(y_value),
                "rz": float(rz_value)
            },
            "terminate": 1 if terminate_value else 0,
            "phase_idx": int(phase_idx) if phase_idx is not None else -1,
            "action_state": 1 if robot_action_state == "moving" else 0
        }
        redis_client.publish("robot:command:channel", json.dumps(command_data))
        print(f"[Python] Published command - y: {y_value:.4f}, rz: {rz_value:.4f}, terminate: {1 if terminate_value else 0}, phase_idx: {command_data['phase_idx']}, action_state: {command_data['action_state']}")
    except Exception as e:
        print(f"[Python] Redis publish error: {e}")

def _normalize_key(k):
    import re
    replacements = [
        (r'^module\.', ''),
        (r'^encoder\.', ''),
        (r'\.mlp\.fc1\.', '.mlp.linear1.'),
        (r'\.mlp\.fc2\.', '.mlp.linear2.'),
        (r'layers\.(\d+)\.', r'layers\1.'),
    ]
    for pat, rep in replacements:
        k = re.sub(pat, rep, k)
    return k

def build_swinmodel(swincheckpoint, device):
    model = SwinTransformer(
        in_chans=3, embed_dim=128, patch_size=(2, 2), window_size=(8, 8),
        depths=(2, 2, 18, 2), num_heads=(4, 8, 16, 32),
        mlp_ratio=4., qkv_bias=True, spatial_dims=2, use_v2=True
    ).to(device).eval()

    if os.path.isfile(swincheckpoint):
        raw = torch.load(swincheckpoint, map_location='cpu')
        raw = raw.get('model', raw)
        ckpt = {_normalize_key(k): v for k, v in raw.items()}
        loadable = {k: v for k, v in ckpt.items()
                    if k in model.state_dict() and v.shape == model.state_dict()[k].shape}
        msg = model.load_state_dict(loadable, strict=False)
        print(f"swin pretrain loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
    else:
        print(f"swin pretrain not found: {swincheckpoint}, random init.")
    return model

def build_swinmodel_origin(swinocheckpoint, device):
    swin = SwinTransformerV2(
        img_size=256, patch_size=2, in_chans=3, num_classes=0,
        embed_dim=48, depths=(2, 2, 2, 2), num_heads=(3, 6, 12, 24),
        window_size=8, drop_path_rate=0.3
    ).to(device).eval()
    if os.path.isfile(swinocheckpoint):
        raw_state = torch.load(swinocheckpoint, map_location="cpu")
        raw_state = raw_state.get("model", raw_state)
        enc_state = {k: v for k, v in raw_state.items() if not k.startswith("head.")}
        swin.load_state_dict(enc_state, strict=False)
        print("swino pretrain loaded.")
    return swin

def build_maemodel(maecheckpoint, maemodel, device):
    model = getattr(models_mae, maemodel)(norm_pix_loss=False).to(device).eval()
    ckpt = torch.load(maecheckpoint, map_location='cpu', weights_only=False)
    state_dict = ckpt["model"] if "model" in ckpt else ckpt
    msg = model.load_state_dict(state_dict, strict=False)
    print(f"mae pretrain loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
    return model

def preprocess_cv2(img_bgr, backbone, size):
    if backbone in ('swin', 'swino', 'transformer'):
        interp = cv2.INTER_CUBIC  
        img_bgr = cv2.resize(img_bgr, (size, size), interpolation=interp)
    else:
        raise ValueError(backbone)

    # 统一使用 ImageNet 归一化（Swin 和 MAE 通常都需要）
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    img_rgb = (img_rgb - mean) / std

    chw = np.transpose(img_rgb, (0, 1, 2)).transpose(2, 0, 1) 
    tensor = torch.from_numpy(chw).unsqueeze(0)
    return tensor

def extract_embedding(backbone, model, img_tensor, pick_indices):
    if backbone == 'swin':
        # Swin 需要返回整个特征金字塔
        feats = model.forward_features(img_tensor, feat_type='pyramid')
        return None, feats, feats[-1]
    elif backbone == 'transformer':
        x, feats, emb = model.forward_encoder2(img_tensor, mask_ratio=0.0, pick_indices=pick_indices)
        return x, feats, emb
    else:
        raise ValueError(backbone)

def init_camera(camera_id, width, height, fps):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera {camera_id}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    print(f"Camera initialized: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} @ {cap.get(cv2.CAP_PROP_FPS)}fps")
    return cap


def main():
    global redis_client, redis_pubsub, camera, inference_running, trigger_inference
    
    # ====================== Configuration Settings ======================
    camera_id = 0
    camera_width = 1920
    camera_height = 1080
    camera_fps = 30
    
    redis_host = '127.0.0.1'
    redis_port = 7777
    
    # [IMPORTANT] 设置为 swin 模式
    backbone = 'swin' 
    swincheckpoint = '/home/jiuan/AAAworkspace/Myproject/slicelocalization/pretrain_weights/simmim/swin_ckp_final_2.pt'
    
    # [IMPORTANT] 请修改为你的最新 posehead 权重路径！
    posehead_ckpt = "weights/best_swin_posehead.pth" 
    
    maecheckpoint = 'weights/backbone/carotid_base_checkpoint-500_0721.pth'
    maemodel = 'mae_vit_base_patch16'
    transformer_layers = [9] 

    # 其他旧任务权重路径（在 swin 模式下暂时不会加载它们）
    anglehead_ckpt = 'weights/angle_head.pth'
    classhead_ckpt = 'weights/clc_head.pth'
    phasehead_ckpt = 'weights/res_phase_head.pth'

    seg_model_path = 'weights/seg_model.pth' 
    seg_encoder = 'resnet34'
    seg_encoder_weights = 'imagenet'
    seg_classes = 3
    seg_threshold = 0.6 
    seg_y_threshold = 100 
    seg_color_map = [[0, 0, 0], [0, 0, 0], [255, 50, 0]]

    class_names = ['diagonally', 'long']
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    stats_path = "weights/norm_stats_all.pt"
    
    cudnn.benchmark = True
    
    # ====================== Initialization ======================
    try:
        redis_client = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)
        redis_pubsub = redis_client.pubsub()
        redis_client.ping()
        print(f"[Python] Connected to Redis")
    except Exception as e:
        print(f"[Python] Redis warning: {e}")
    
    try:
        camera = init_camera(camera_id, camera_width, camera_height, camera_fps)
    except Exception as e:
        print(f"[Python] Camera error: {e}")
        return

    inv_dT1e = build_inv_normalizer_from_dataset(
        key="dT_1e", stats_path=stats_path, mode="minmax_pad", pad_ratio=0.05, feat_axis=-1, index=[1]
    )

    img_size = 256 if backbone in ('swin', 'swino') else 224

    # 构建 Backbone
    if backbone == 'swin':
        backbone_model = build_swinmodel(swincheckpoint, device)
    else:
        backbone_model = build_maemodel(maecheckpoint, maemodel, device)
    
    # 构建 Heads
    if backbone == 'swin':
        swin_dims = [128, 256, 512, 1024]
        # [NEW] 使用你修改后的 SwinPoseHead
        posehead = SwinPoseHead(in_dims=swin_dims, hidden_dim=256, out_dim=1).to(device).eval()
        anglehead = None
        classhead = None
        phasehead = None
    else:
        posehead = PoseHead(in_dim=768, out_dim=1).to(device).eval()
        anglehead = PoseHead(in_dim=768, out_dim=1).to(device).eval()
        classhead = PoseClsHead2(in_dim=768, num_classes=len(class_names), binary_mode="ce").to(device).eval()
        phasehead = RESNetHead(in_dim=768, out_dim=3, dropout=0.2).to(device).eval()

    # 构建 Segmentation
    seg_model = None
    try:
        seg_model = smp.Unet(encoder_name=seg_encoder, encoder_weights=seg_encoder_weights, classes=seg_classes, activation=None).to(device)
        if os.path.exists(seg_model_path):
            seg_model.load_state_dict(torch.load(seg_model_path, map_location='cpu'))
        seg_model.eval()
    except Exception as e:
        pass

    # ====================== Load Checkpoints ======================
    print("\n[Loading Checkpoints...]")
    if backbone == 'swin':
        if os.path.isfile(posehead_ckpt):
            ckpt = torch.load(posehead_ckpt, map_location='cpu')
            state_dict = ckpt['posehead'] if 'posehead' in ckpt else ckpt
            msg = posehead.load_state_dict(state_dict, strict=False)
            print(f"Swin PoseHead loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
        else:
            print(f"Warning: Swin PoseHead ckpt not found at {posehead_ckpt}")
    else:
        # MAE 原版的加载逻辑 (省略以保持代码整洁，你可以自行加回)
        pass

    print("\n[Python] Starting inference loop... Press 'q' to quit, 'b' to trigger inference\n")
    if redis_pubsub:
        redis_pubsub.subscribe("robot:status:channel")
        redis_pubsub.subscribe("robot_control")
    
    # ====================== Main Inference Loop ======================
    terminate_flag_key = False
    robot_action_state = "idle"  
    segmentation_flag = False  

    while inference_running:
        if redis_pubsub:
            try:
                message = redis_pubsub.get_message(timeout=0.001)
                if message and message['type'] == 'message' and message['channel'] == 'robot_control':
                    data = json.loads(message['data'])
                    if data.get('command') == 'robot' and data.get('action') == 'move':
                        robot_action_state = "moving"
            except Exception: pass

        ret, frame = camera.read()
        if not ret: continue
        
        frame = cv2.flip(frame, 0)
        frame = frame[160:680, 595:1415, :]
        
        # --- Segmentation ---
        seg_overlay_bgr = frame.copy()
        if seg_model is not None and segmentation_flag:
            with torch.no_grad():
                h, w = frame.shape[:2]
                img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img_tensor_s = img_rgb.astype('float32') / 255.0
                img_tensor_s = torch.from_numpy(img_tensor_s.transpose(2, 0, 1)).unsqueeze(0).to(device)
                
                pad_h = (32 - h % 32) % 32
                pad_w = (32 - w % 32) % 32
                if pad_h > 0 or pad_w > 0:
                    img_tensor_s = F.pad(img_tensor_s, (0, pad_w, 0, pad_h), mode='constant', value=0)
                
                seg_output = seg_model(img_tensor_s)
                seg_probs = torch.softmax(seg_output, dim=1)
                max_vals, pred_mask = torch.max(seg_probs, dim=1)
                
                pred_mask = pred_mask.squeeze(0).cpu().numpy()
                max_vals = max_vals.squeeze(0).cpu().numpy()
                pred_mask[max_vals < seg_threshold] = 0
                if pad_h > 0 or pad_w > 0: pred_mask = pred_mask[:h, :w]
                if seg_y_threshold > 0: pred_mask[:seg_y_threshold, :] = 0

                colored_mask = colorize_mask(pred_mask, seg_color_map)
                colored_mask_bgr = cv2.cvtColor(colored_mask, cv2.COLOR_RGB2BGR)
                seg_overlay_bgr = cv2.addWeighted(frame, 1, colored_mask_bgr, 0.8, 0)
        
        cavana = cv2.resize(seg_overlay_bgr.copy(), (1300, 1300))

        key = cv2.waitKey(1)
        if key == ord('q'): inference_running = False; break
        elif key == ord('b'): trigger_inference = not trigger_inference
        elif key == ord('t'): terminate_flag_key = not terminate_flag_key
        elif key == ord('m'): robot_action_state = "moving" if robot_action_state == "idle" else "idle"
        elif key == ord('s'): segmentation_flag = not segmentation_flag

        if trigger_inference:
            img_tensor = preprocess_cv2(frame, backbone, img_size).to(device)
            
            with torch.no_grad():
                with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):
                    x, feats, emb = extract_embedding(backbone, backbone_model, img_tensor, pick_indices=transformer_layers)
                    
                    if backbone == 'swin':
                        # ==========================================
                        # [NEW] SWIN 推理模式 (Mock 隔离未训练头)
                        # ==========================================
                        # 1. 真实运行 y 位移预测
                        pose = posehead(feats)
                        pred_xy_real = inv_dT1e(pose)
                        y_value = float(pred_xy_real[0, 0].cpu().numpy())
                        
                        # 2. 伪造其他未训练的输出以保证系统运行
                        rz_value = 0.0          # 伪造角度
                        class_value = 0         # 伪造分类
                        phase_idx = 0           # 伪造状态 (Scan)
                        phaseprobs = torch.tensor([[1.0, 0.0, 0.0]], device=device) # [Scan, Trigger, Action]
                        
                    else:
                        # 原版 MAE 推理模式
                        pose = posehead(feats[-1], x)
                        pred_xy_real = inv_dT1e(pose)
                        angle = anglehead(feats[-1], x) * 90
                        classlogits = classhead(x)
                        x_pooled = x.mean(dim=1)  
                        phaselogits = phasehead(x_pooled) 
                        
                        phaseprobs = torch.softmax(phaselogits, dim=1)
                        classprobs = torch.softmax(classlogits, dim=1)
                        
                        y_value = float(pred_xy_real[0, 0].cpu().numpy())  
                        rz_value = float(angle[0, 0].cpu().numpy())
                        class_value = 1 if classprobs[0,1] > 0.9 else 0
                        # ... (此处省略原版 MAE 复杂的触发逻辑以节省空间) ...
                        phase_idx = 0

                    # --- 公共后处理 ---
                    terminate_value = (class_value == 1) or terminate_flag_key
                    if abs(y_value) < 0.002:
                        y_value = y_value / 2

                    if redis_client:
                        publish_command(redis_client, y_value, rz_value, terminate_value, phase_idx, robot_action_state)
                
                # --- 屏幕信息渲染 ---
                overlay_text = f"y: {y_value:.4f}"
                phase_str = PHASE_LABELS.get(phase_idx, "Unknown")
                phase_text = f"Phase: {phase_str}({phaseprobs[0, phase_idx]:.2f})"
                
                p_color = (0, 255, 0) if phase_idx == 0 else (0, 0, 255) if phase_idx == 1 else (255, 0, 0)
                cv2.putText(cavana, phase_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, p_color, 2, cv2.LINE_AA)
                cv2.putText(cavana, f"Terminate: {terminate_value}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                
                if y_value > 0.0003:
                    cv2.arrowedLine(cavana, (50, 50), (100, 50), (0, 0, 255), 2)
                    cv2.putText(cavana, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
                elif y_value < -0.0003:
                    cv2.arrowedLine(cavana, (100, 50), (50, 50), (0, 255, 0), 2)
                    cv2.putText(cavana, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
                else:
                    cv2.putText(cavana, "On Track", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2, cv2.LINE_AA)
                
                cv2.putText(cavana, f"rz: {rz_value:.2f}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

        if segmentation_flag:
            cv2.putText(cavana, "Segmentation: ON", (1150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(cavana, "Segmentation: OFF", (1150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
        
        cv2.imshow('US image', cavana)

    if redis_client: publish_command(redis_client, 0.0, 0.0, True, -1)
    if redis_pubsub: redis_pubsub.unsubscribe(); redis_pubsub.close()
    if camera is not None: camera.release()
    cv2.destroyAllWindows()
    print("[Python] Stopped")

if __name__ == "__main__":
    main()