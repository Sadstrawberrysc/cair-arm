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
from torch.backends.cuda import sdp_kernel

from utils import rot6d_to_matrix
from models import (
    models_mae,
    SwinTransformer,
    SwinTransformerV2,
    PoseHead,
    PoseClsHead,
    PoseClsHead2,
    PhaseHead  # 确保 models/__init__.py 里导出了这个类
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

def publish_command(redis_client, y_value, rz_value, terminate_value, phase_idx=None,robot_action_state="idle"):
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
        # Publish to command channel
        redis_client.publish("robot:command:channel", json.dumps(command_data))
        print(f"[Python] Published command - y: {y_value:.4f}, rz: {rz_value:.4f}, terminate: {1 if terminate_value else 0}, phase_idx: {command_data['phase_idx']}, action_state: {command_data['action_state']}")
        
    except Exception as e:
        print(f"[Python] Redis publish error: {e}")

def subscribe_status(redis_pubsub):
    """Subscribe and listen for robot status messages"""
    try:
        redis_pubsub.subscribe("robot:status:channel")
        
        # Get message with timeout
        message = redis_pubsub.get_message(timeout=1.0)
        while message:
            if message['type'] == 'message':
                status_data = json.loads(message['data'])
                print(f"[Python] Received status: {status_data}")
                return status_data
            message = redis_pubsub.get_message(timeout=0.01)
    except Exception as e:
        print(f"[Python] Redis subscribe error: {e}")
    
    return None

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

    ckpt_path = swincheckpoint
    if os.path.isfile(ckpt_path):
        raw = torch.load(ckpt_path, map_location='cpu')
        raw = raw.get('model', raw)
        ckpt = {_normalize_key(k): v for k, v in raw.items()}
        loadable = {k: v for k, v in ckpt.items()
                    if k in model.state_dict() and v.shape == model.state_dict()[k].shape}
        msg = model.load_state_dict(loadable, strict=False)
        print(f"swin pretrain loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
    else:
        print(f"swin pretrain not found: {ckpt_path}, random init.")
    return model

def build_swinmodel_origin(swinocheckpoint, device):
    swin = SwinTransformerV2(
        img_size=256, patch_size=2, in_chans=3,
        num_classes=0,
        embed_dim=48, depths=(2, 2, 2, 2),
        num_heads=(3, 6, 12, 24),
        window_size=8, drop_path_rate=0.3
    ).to(device).eval()

    ckpt_p = swinocheckpoint
    if os.path.isfile(ckpt_p):
        raw_state = torch.load(ckpt_p, map_location="cpu")
        raw_state = raw_state.get("model", raw_state)
        enc_state = {k: v for k, v in raw_state.items() if not k.startswith("head.")}
        swin.load_state_dict(enc_state, strict=False)
        print("[swino pretrain loaded.")
    else:
        print(f"swino pretrain not found: {ckpt_p}, random init.")
    return swin

def build_maemodel(maecheckpoint, maemodel, device):
    assert hasattr(models_mae, maemodel), f"models_mae.py cannot find '{maemodel}'"
    model = getattr(models_mae, maemodel)(norm_pix_loss=False).to(device).eval()
    ckpt = torch.load(maecheckpoint, map_location='cpu', weights_only=False)
    state_dict = ckpt["model"] if "model" in ckpt else ckpt
    msg = model.load_state_dict(state_dict, strict=False)
    print(f"mae pretrain loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
    return model

def center_crop_numpy(img, size):
    h, w = img.shape[:2]
    th, tw = size, size
    y1 = max(0, (h - th) // 2)
    x1 = max(0, (w - tw) // 2)
    return img[y1:y1+th, x1:x1+tw, :]

def preprocess_cv2(img_bgr, backbone, size):
    if backbone in ('swin', 'swino', 'transformer'):
        interp = cv2.INTER_CUBIC  
        img_bgr = cv2.resize(img_bgr, (size, size), interpolation=interp)
    else:
        raise ValueError(backbone)

    if backbone == 'transformer':
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img_rgb = (img_rgb - mean) / std
    else:
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

    chw = np.transpose(img_rgb, (0, 1, 2)).transpose(2, 0, 1) 
    tensor = torch.from_numpy(chw).unsqueeze(0)
    return tensor

def extract_embedding(backbone, model, img_tensor, pick_indices):
    if backbone == 'swin':
        feat = model.forward_features(img_tensor, feat_type='pyramid')[-1]
        return None, None, [feat]
    elif backbone == 'swino':
        feat = model.forward_features2(img_tensor)
        return None, None, feat
    elif backbone == 'transformer':
        x, feats, emb = model.forward_encoder2(img_tensor, mask_ratio=0.0, pick_indices=pick_indices)
        return x, feats, emb
    else:
        raise ValueError(backbone)

def init_camera(camera_id, width, height, fps):
    """Initialize USB camera"""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera {camera_id}")
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    # Verify settings
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps")
    return cap

def main():
    global redis_client, redis_pubsub, camera, inference_running, trigger_inference
    
    # ====================== Configuration Settings ======================
    # Camera settings
    camera_id = 0
    camera_width = 1920
    camera_height = 1080
    camera_fps = 30
    
    # Redis settings
    redis_host = '127.0.0.1'
    redis_port = 7777
    
    # Model settings
    backbone = 'transformer'
    swincheckpoint = '/home/jiuan/AAAworkspace/Myproject/slicelocalization/pretrain_weights/simmim/swin_ckp_final_2.pt'
    swinocheckpoint = '/path/to/swino_pretrain.pth'
    maecheckpoint = 'weights/backbone/carotid_base_checkpoint-500_0721.pth'
    maemodel = 'mae_vit_base_patch16'
    transformer_layers = [9] 
    
    # Head checkpoints
    posehead_ckpt = "weights/pose_head.pth"
    anglehead_ckpt = 'weights/angle_head.pth'
    classhead_ckpt = 'weights/clc_head.pth'
    # [NEW] Phase head checkpoint path
    phasehead_ckpt = 'weights/phase_head.pth'

    # Segmentation settings
    seg_model_path = 'weights/seg_model.pth' 
    seg_encoder = 'resnet34'
    seg_encoder_weights = 'imagenet'
    seg_classes = 3
    seg_threshold = 0.6 # 增加阈值配置    
    seg_y_threshold = 100 # y轴小于此值的区域不显示分割结果 (像素单位)    
    # 0:Black, 1:Black, 2:Orange
    seg_color_map = [
        [0, 0, 0],       
        [0, 0, 0],   
        [255, 50, 0]    
    ]

    # Other settings
    class_names = ['diagonally', 'long']
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # Normalization stats path
    stats_path = "weights/norm_stats_all.pt"
    
    # Video saving settings
    output_dir = './output'
    
    # ====================== Initialization ======================
    
    cudnn.benchmark = True
    
    # Initialize Redis connection
    try:
        redis_client = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)
        redis_pubsub = redis_client.pubsub()
        redis_client.ping()
        print(f"[Python] Connected to Redis at {redis_host}:{redis_port}")
    except Exception as e:
        print(f"[Python] Failed to connect to Redis: {e}")
        return
    
    # Initialize camera
    try:
        camera = init_camera(camera_id, camera_width, camera_height, camera_fps)
    except Exception as e:
        print(f"[Python] Failed to initialize camera: {e}")
        return
        
    # Initialize video writer
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    video_filename = os.path.join(output_dir, f"{timestamp}.avi")
    frame_width_out = 600
    frame_height_out = 640
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # video_writer = cv2.VideoWriter(video_filename, fourcc, float(camera_fps), (frame_width_out, frame_height_out))
    print(f"Recording video to {video_filename}")

    # Load normalization stats
    inv_dT1e = build_inv_normalizer_from_dataset(
        key="dT_1e", 
        stats_path=stats_path, 
        mode="minmax_pad", 
        pad_ratio=0.05, 
        feat_axis=-1, 
        index=[1]
    )

    img_size = 256 if backbone in ('swin', 'swino') else 224

    # Build backbone
    if backbone == 'swin':
        backbone_model = build_swinmodel(swincheckpoint, device)
    elif backbone == 'swino':
        backbone_model = build_swinmodel_origin(swinocheckpoint, device)
    else:
        backbone_model = build_maemodel(maecheckpoint, maemodel, device)
    
    # --- Build Heads ---
    posehead = PoseHead(in_dim=768, out_dim=1).to(device).eval()
    anglehead = PoseHead(in_dim=768, out_dim=1).to(device).eval()
    classhead = PoseClsHead2(in_dim=768, num_classes=len(class_names), binary_mode="ce").to(device).eval()
    
    # [NEW] Build Phase Head (assuming input 768, output 3 classes: Scan/Trigger/Action)
    phasehead = PhaseHead(in_dim=768, out_dim=3, dropout=0.5).to(device).eval()

    # --- Build Segmentation Model ---
    print("[Python] Loading Segmentation Model...")
    try:
        seg_model = smp.Unet(
            encoder_name=seg_encoder, 
            encoder_weights=seg_encoder_weights, 
            classes=seg_classes, 
            activation=None
        ).to(device)
        
        if os.path.exists(seg_model_path):
            seg_model.load_state_dict(torch.load(seg_model_path, map_location='cpu'))
            print("[Python] Segmentation model loaded.")
        else:
            print(f"[Python] Segmentation model not found at {seg_model_path}, using random init.")
        seg_model.eval()
    except Exception as e:
        print(f"[Python] Failed to load segmentation model: {e}")
        seg_model = None

    # --- Load Checkpoints ---
    print("\n[Loading Checkpoints...]")
    
     # Load checkpoints
    posehead_ckpt_data = torch.load(posehead_ckpt, map_location='cpu', weights_only=False)
    anglehead_ckpt_data = torch.load(anglehead_ckpt, map_location='cpu', weights_only=False)
    classhead_ckpt_data = torch.load(classhead_ckpt, map_location='cpu', weights_only=False)
    phasehead_ckpt_data = torch.load(phasehead_ckpt, map_location='cpu', weights_only=False)
    print(phasehead_ckpt_data.keys())
    print(anglehead_ckpt_data.keys())
    if 'posehead' in posehead_ckpt_data:
        missing, unexpected = posehead.load_state_dict(posehead_ckpt_data['posehead'], strict=False)
        print(f"posehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    if 'posehead' in anglehead_ckpt_data:
        missing, unexpected = anglehead.load_state_dict(anglehead_ckpt_data['posehead'], strict=False)
        print(f"anglehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    if 'clshead' in classhead_ckpt_data:
        missing, unexpected = classhead.load_state_dict(classhead_ckpt_data['clshead'], strict=False)
        print(f"classhead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    if 'head' in phasehead_ckpt_data:
        missing, unexpected = phasehead.load_state_dict(phasehead_ckpt_data['head'], strict=False)
        print(f"phasehead loaded: missing={len(missing)} unexpected={len(unexpected)}")

    print("\n[Python] Starting inference loop...")
    print("Press 'q' to quit, 'b' to trigger inference\n")

    # Subscribe to status channel
    redis_pubsub.subscribe("robot:status:channel")
    redis_pubsub.subscribe("robot_control")
    
    # ====================== Main Inference Loop ======================
    # try:
    terminate_flag_key = False

    robot_action_state = "idle"  # Track robot action state
    segmentation_flag = False  # Enable segmentation overlay display

    while inference_running:
        # Check for control messages
        try:
            message = redis_pubsub.get_message(timeout=0.001)
            if message and message['type'] == 'message':
                if message['channel'] == 'robot_control':
                    try:
                        data = json.loads(message['data'])
                        if data.get('command') == 'robot' and data.get('action') == 'move':
                            bodypart = data.get('parameter')
                            print(f"[Python] Recevied Robot Start for: {bodypart}")
                            robot_action_state = "moving"
                    except json.JSONDecodeError:
                        pass
        except Exception:
            pass

        begin_time = time.time()
        ret, frame = camera.read()
        if not ret:
            print("[Python] Failed to capture frame")
            continue
        
        frame = cv2.flip(frame, 0)
        frame = frame[160:680,595:1415,:]
        # video_writer.write(frame)
        
        # --- Segmentation Inference ---
        seg_overlay_bgr = frame.copy()
        if seg_model is not None and segmentation_flag:
            try:
                with torch.no_grad():
                    h, w = frame.shape[:2]
                    # Preprocess
                    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img_tensor_s = img_rgb.astype('float32') / 255.0
                    img_tensor_s = torch.from_numpy(img_tensor_s.transpose(2, 0, 1)).unsqueeze(0).to(device)
                    
                    # Padding
                    pad_h = (32 - h % 32) % 32
                    pad_w = (32 - w % 32) % 32
                    if pad_h > 0 or pad_w > 0:
                        img_tensor_s = F.pad(img_tensor_s, (0, pad_w, 0, pad_h), mode='constant', value=0)
                    
                    # Inference
                    seg_output = seg_model(img_tensor_s)
                    
                    # 获取概率
                    seg_probs = torch.softmax(seg_output, dim=1)
                    
                    # 获取最大概率值和对应的类别索引
                    max_vals, pred_mask = torch.max(seg_probs, dim=1)
                    
                    pred_mask = pred_mask.squeeze(0).cpu().numpy()
                    max_vals = max_vals.squeeze(0).cpu().numpy()
                    
                    # 应用阈值过滤：只有当预测概率大于阈值时才保留，否则设为0 (背景)
                    pred_mask[max_vals < seg_threshold] = 0
                    
                    # Unpad
                    if pad_h > 0 or pad_w > 0:
                        pred_mask = pred_mask[:h, :w]
                    
                    # Y-axis Filter
                    if seg_y_threshold > 0:
                        # y轴小于seg_y_threshold的区域设为背景(0)
                        pred_mask[:seg_y_threshold, :] = 0

                    # Visualize
                    colored_mask = colorize_mask(pred_mask, seg_color_map)
                    colored_mask_bgr = cv2.cvtColor(colored_mask, cv2.COLOR_RGB2BGR)
                    seg_overlay_bgr = cv2.addWeighted(frame, 1, colored_mask_bgr, 0.8, 0)
            except Exception as e:
                # print(f"Seg inference error: {e}") 
                pass
        # seg_overlay_bgr = cv2.resize(seg_overlay_bgr, (800, 600))
        cavana = seg_overlay_bgr.copy()
        cavana = cv2.resize(cavana, (1300, 1300))
        # Display camera feed (basic)

        key = cv2.waitKey(1)
        
        if key == ord('q'):
            print("[Python] User requested quit")
            inference_running = False
            break
        elif key == ord('b'):
            print("[Python] User triggered inference")
            trigger_inference = not trigger_inference
        elif key == ord('t'):
            terminate_flag_key = not terminate_flag_key
            print(f"[Python] Terminate flag set to {terminate_flag_key}")
        elif key == ord('m'):
            robot_action_state = "moving" if robot_action_state == "idle" else "idle"
            print(f"[Python] Robot action state set to {robot_action_state}")
        elif key == ord('s'):
            segmentation_flag = not segmentation_flag

        if trigger_inference:
                img_tensor = preprocess_cv2(frame, backbone, img_size).to(device)
                
                with torch.no_grad():
                    with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):
                        # 1. Extract Backbone Features
                        # x shape: [1, 197, 768] (Sequence of patches)
                        x, feats, emb = extract_embedding(backbone, backbone_model, img_tensor, pick_indices=transformer_layers)
                        
                        # 2. Existing Heads Inference
                        pose = posehead(feats[-1], x)
                        pred_xy_real = inv_dT1e(pose)
                        angle = anglehead(feats[-1], x) * 90
                        
                        classlogits = classhead(x)
                        classpreds = torch.argmax(classlogits, dim=1)
                        classprobs = torch.softmax(classlogits, dim=1)
                        print(f"[ClassHead] probs: {classprobs[0].cpu().numpy().tolist()}")
                        class_value = 1 if classprobs[0,1] > 0.9 else 0
                        
                        # ==========================================
                        # 3. [NEW] Phase Head Inference (Fix Error)
                        # ==========================================
                        # x 是 [1, 197, 768]，Head 需要 [1, 768]
                        # 进行平均池化 (Mean Pooling)
                        x_pooled = x.mean(dim=1)  
                        
                        phaselogits = phasehead(x_pooled) 
                        # ==========================================
                        
                        # phasepred = torch.argmax(phaselogits, dim=1)
                        phaseprobs = torch.softmax(phaselogits, dim=1)
                        prob_trigger = phaseprobs[:, 1]
                        prob_scan = phaseprobs[:, 0]
                        prob_action = phaseprobs[:, 2]

                                        # Scan vs Action
                        pred_others = torch.where(prob_scan > prob_action, 
                                          torch.tensor(0, device=device), 
                                          torch.tensor(2, device=device))
                
                        # Trigger Logic
                        batch_preds_tensor = torch.where(
                            prob_trigger > 0.7,
                            torch.tensor(1, device=device),
                            pred_others
                        )

                        phase_idx = int(batch_preds_tensor[0].cpu().numpy())
                        
                        phase_str = PHASE_LABELS.get(phase_idx, "Unknown")
                        
                        # Extract Values
                        y_value = float(pred_xy_real[0, 0].cpu().numpy())  
                        rz_value = float(angle[0, 0].cpu().numpy())
                        # class_value = int(classpreds[0].cpu().numpy())
                        # if(abs(rz_value)<10):
                        #     class_value =1
                        # else:
                        #     class_value =0
                        terminate_value = (class_value == 1)
                        if terminate_flag_key:
                            terminate_value = True
                        if(abs(y_value) < 0.002):
                            y_value = y_value / 2

                        # [Updated Print] Include Phase
                        print(f"[Infer] y: {y_value:.4f}, rz: {rz_value:.4f}, cls: {class_value}, phase: {phase_idx}")
                        
                        # Publish command
                        publish_command(redis_client, y_value, rz_value, terminate_value, phase_idx,robot_action_state)
                    
                    # --- Visualization ---
                    overlay_text = f"y: {y_value:.4f}"
                    phase_text = f"Phase: {phase_str}"
                    
                    # Phase Color (Scan:Green, Trigger:Red, Action:Blue)
                    p_color = (0, 255, 0) if phase_idx == 0 else (0, 0, 255) if phase_idx == 1 else (255, 0, 0)
                    
                    

                    cv2.putText(cavana, phase_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, p_color, 2, cv2.LINE_AA)
                    terminate_text = f"Terminate: {terminate_value}"
                    cv2.putText(cavana, terminate_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
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
        # cv2.imshow('US image', cavana)
                    # cv2.waitKey(1)
        cv2.imshow('US image', cavana)
        # cv2.imshow('Segmentation Overlay', seg_overlay_bgr)
            # end_time = time.time()
            # print(f"Time: {end_time - begin_time:.4f}s")
            
    # Send final terminate command
    if redis_client:
        publish_command(redis_client, 0.0, 0.0, True, -1)
        print("[Python] Sent final terminate command")
    
    # Unsubscribe
    if redis_pubsub:
        redis_pubsub.unsubscribe()
        redis_pubsub.close()

    if camera is not None: camera.release()
    # if video_writer is not None: video_writer.release()
    cv2.destroyAllWindows()
    print("[Python] Stopped")

if __name__ == "__main__":
    main()
