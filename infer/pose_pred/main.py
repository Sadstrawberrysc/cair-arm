import os
import json
import time
import threading
import cv2
import numpy as np
import torch
import torch.nn.functional as F
import redis
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as T
import torch.backends.cudnn as cudnn
from torch.backends.cuda import sdp_kernel

from utils import rot6d_to_matrix
from Mymodels import (
    models_mae,
    SwinTransformer,
    SwinTransformerV2,
    PoseHead4,
    PoseClsHead2
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

def publish_command(redis_client, y_value, rz_value, terminate_value):
    """Publish command to Redis channel"""
    try:
        command_data = {
            "parameters": {
                "y": float(y_value),
                "rz": float(rz_value)
            },
            "terminate": bool(terminate_value)
        }
        
        # Publish to command channel
        redis_client.publish("robot:command:channel", json.dumps(command_data))
        print(f"[Python] Published command - y: {y_value:.4f}, rz: {rz_value:.4f}, terminate: {terminate_value}")
        
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
    maecheckpoint = '/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/carotid_base_checkpoint-500_0721.pth'
    maemodel = 'mae_vit_base_patch16'
    transformer_layers = [9]  # Updated from [5] to [9] based on new inference.py
    
    # Head checkpoints - using new paths from inference.py
    posehead_ckpt = '/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/pose/best.pth'
    anglehead_ckpt = '/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/angle/best.pth'
    classhead_ckpt = '/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/classification/best.pth'
    
    # Other settings
    class_names = ['diagonally', 'long']
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    inference_interval = 0.1
    
    # Normalization stats path - updated from new inference.py
    stats_path = "/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/norm_stats_all.pt"
    
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
    # Frame size is based on the cropped dimensions: (1110-510) x (800-160) = 600 x 640
    frame_width_out = 600
    frame_height_out = 640
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = cv2.VideoWriter(video_filename, fourcc, float(camera_fps), (frame_width_out, frame_height_out))
    print(f"Recording video to {video_filename}")

    # Load normalization stats - updated to use index=[1] from new inference.py
    inv_dT1e = build_inv_normalizer_from_dataset(
        key="dT_1e", 
        stats_path=stats_path, 
        mode="minmax_pad", 
        pad_ratio=0.05, 
        feat_axis=-1, 
        index=[1]  # Changed from [0, 1] to [1] based on new inference.py
    )

    img_size = 256 if backbone in ('swin', 'swino') else 224

    # Build backbone
    if backbone == 'swin':
        backbone_model = build_swinmodel(swincheckpoint, device)
    elif backbone == 'swino':
        backbone_model = build_swinmodel_origin(swinocheckpoint, device)
    else:
        backbone_model = build_maemodel(maecheckpoint, maemodel, device)
    
    # Build heads - using new PoseHead4 and PoseClsHead2 architecture
    posehead = PoseHead4(
        in_dim=768,
        out_dim=1,
    ).to(device).eval()
    
    anglehead = PoseHead4(
        in_dim=768,
        out_dim=1,
    ).to(device).eval()
    
    num_classes = len(class_names)
    classhead = PoseClsHead2(
        in_dim=768,
        num_classes=num_classes,
        binary_mode="ce",
    ).to(device).eval()

    # Load checkpoints
    posehead_ckpt_data = torch.load(posehead_ckpt, map_location='cpu', weights_only=False)
    anglehead_ckpt_data = torch.load(anglehead_ckpt, map_location='cpu', weights_only=False)
    classhead_ckpt_data = torch.load(classhead_ckpt, map_location='cpu', weights_only=False)

    if 'posehead' in posehead_ckpt_data:
        missing, unexpected = posehead.load_state_dict(posehead_ckpt_data['posehead'], strict=False)
        print(f"posehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    if 'posehead' in anglehead_ckpt_data:
        missing, unexpected = anglehead.load_state_dict(anglehead_ckpt_data['posehead'], strict=False)
        print(f"anglehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    if 'clshead' in classhead_ckpt_data:
        missing, unexpected = classhead.load_state_dict(classhead_ckpt_data['clshead'], strict=False)
        print(f"classhead loaded: missing={len(missing)} unexpected={len(unexpected)}")

    print("\n[Python] Starting inference loop...")
    print("Press 'q' to quit, 'b' to trigger inference\n")
    
    # Subscribe to status channel
    redis_pubsub.subscribe("robot:status:channel")
    
    # ====================== Main Inference Loop ======================
    
    try:
        while inference_running:
            begin_time = time.time()
            # Capture frame from camera
            ret, frame = camera.read()
            if not ret:
                print("[Python] Failed to capture frame")
                continue
            frame = frame[160:800, 510:1110]
            
            # Write frame to video file
            video_writer.write(frame)
            
            # Display camera feed
            cv2.imshow('US image', frame)
            key = cv2.waitKey(1)
            
            if key == ord('q'):
                print("[Python] User requested quit")
                inference_running = False
                break
            elif key == ord('b'):
                print("[Python] User triggered inference")
                trigger_inference = True

            # Check for status messages from robot
            message = redis_pubsub.get_message(timeout=0.001)
            if message and message['type'] == 'message':
                status_data = json.loads(message['data'])
                print(f"[Python] Received status: {status_data}")
                trigger_inference = True  # Trigger inference when status received
            
            # Perform inference if triggered
            if trigger_inference:
                trigger_inference = False
                
                # Preprocess image
                img_tensor = preprocess_cv2(frame, backbone, img_size).to(device)
                
                # Run inference
                with torch.no_grad():
                    with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):
                        # Extract embeddings - updated to handle new return format
                        x, feats, emb = extract_embedding(backbone, backbone_model, img_tensor, pick_indices=transformer_layers)
                        
                        # Pose prediction - using new head architecture
                        pose = posehead(feats[-1], x)
                        pred_xy_norm = pose
                        pred_xy_real = inv_dT1e(pred_xy_norm)
                        
                        # Angle prediction - using new head architecture
                        angle = anglehead(feats[-1], x) * 90
                        
                        # Class prediction - using new head architecture
                        classlogits = classhead(x)
                        classpreds = torch.argmax(classlogits, dim=1)
                        
                        # Extract values for Redis
                        y_value = float(pred_xy_real[0, 0].cpu().numpy())  # Changed index since out_dim=1
                        rz_value = float(angle[0, 0].cpu().numpy())
                        class_value = int(classpreds[0].cpu().numpy())
                        
                        # terminate is true only when class_value is 1
                        terminate_value = (class_value == 1)
                        
                        print(f"[Python] Inference - y: {y_value:.4f}, rz: {rz_value:.4f}, class: {class_value}, terminate: {terminate_value}")
                        
                        # Publish command
                        publish_command(redis_client, y_value, rz_value, terminate_value)
                
                end_time = time.time()
                print(f"[Python] Inference time: {end_time - begin_time:.4f} seconds")
                
                # Control inference rate
                time.sleep(inference_interval)
            
    except KeyboardInterrupt:
        print("\n[Python] Interrupted by user")
    except Exception as e:
        print(f"[Python] Error in main loop: {e}")
    finally:
        # Cleanup
        if camera is not None:
            camera.release()
        if video_writer is not None:
            video_writer.release()
            print("[Python] Video recording stopped and saved.")
        cv2.destroyAllWindows()
        
        # Send final terminate command
        if redis_client:
            publish_command(redis_client, 0.0, 0.0, True)
            print("[Python] Sent final terminate command")
        
        # Unsubscribe
        if redis_pubsub:
            redis_pubsub.unsubscribe()
            redis_pubsub.close()
        
        print("[Python] Inference stopped")

if __name__ == "__main__":
    main()
