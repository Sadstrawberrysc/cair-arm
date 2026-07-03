import os
import time
import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as T
from PIL import Image
from collections import deque

# 尝试导入 models_mae
try:
    from models import models_mae
except ImportError:
    print("Warning: models_mae not found. Please ensure it is in path.")
    models_mae = None

# ==============================================================================
# 1. 独立的 Head 定义
# ==============================================================================
class SimpleClassHead(nn.Module):
    def __init__(self, in_dim=768, out_dim=3, dropout=0.5):
        super(SimpleClassHead, self).__init__()
        self.bn = nn.BatchNorm1d(in_dim)
        self.dropout = nn.Dropout(dropout)
        self.fc = nn.Linear(in_dim, out_dim)

    def forward(self, x):
        x = self.bn(x)
        x = self.dropout(x)
        x = self.fc(x)
        return x

# ==============================================================================
# 2. 模型构建辅助函数
# ==============================================================================
def build_maemodel(maecheckpoint, maemodel_name, device):
    """构建并加载 Backbone"""
    print(f"Building Backbone: {maemodel_name} ...")
    model = getattr(models_mae, maemodel_name)(norm_pix_loss=False).to(device).eval()
    
    if os.path.isfile(maecheckpoint):
        ckpt = torch.load(maecheckpoint, map_location='cpu')
        state_dict = ckpt["model"] if "model" in ckpt else ckpt
        # strict=False 因为 backbone 权重里没有 head
        msg = model.load_state_dict(state_dict, strict=False)
        print(f"MAE Backbone loaded: missing={len(msg.missing_keys)}, unexpected={len(msg.unexpected_keys)}")
    else:
        print(f"Warning: MAE checkpoint not found at {maecheckpoint}. Using random weights!")
    
    return model

def build_head(checkpoint_path, device, num_classes=3):
    """构建并加载 Head"""
    print("Building Head ...")
    head = SimpleClassHead(in_dim=768, out_dim=num_classes, dropout=0.5).to(device).eval()
    
    if os.path.isfile(checkpoint_path):
        print(f"Loading Head weights from: {checkpoint_path}")
        ckpt = torch.load(checkpoint_path, map_location='cpu')
        
        # 兼容处理：检查是否有 'head' 键 (Head-Only 保存模式) 或 'model' 键 (完整保存模式)
        if 'head' in ckpt:
            state_dict = ckpt['head']
        elif 'model' in ckpt:
            state_dict = ckpt['model']
        else:
            state_dict = ckpt

        # 关键步骤：清洗 Key
        # 如果训练时用了 CarotidClassifier，key 可能是 "head.fc.weight"
        # 现在的 head 只需要 "fc.weight"
        clean_state_dict = {}
        for k, v in state_dict.items():
            if k.startswith('head.'):
                clean_state_dict[k.replace('head.', '')] = v
            elif not k.startswith('backbone.'): # 如果原本就是纯 head key
                clean_state_dict[k] = v
                
        msg = head.load_state_dict(clean_state_dict, strict=True)
        print(f"Head weights loaded successfully.")
    else:
        print(f"Error: Head checkpoint not found at {checkpoint_path}")
        exit(1)
        
    return head

# ==============================================================================
# 3. 工具函数
# ==============================================================================
def preprocess_frame(frame_bgr, transform):
    """OpenCV BGR -> PIL -> Tensor"""
    img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)
    return transform(pil_img).unsqueeze(0)

# 颜色定义 (BGR)
COLORS = {
    0: (0, 255, 0),    # Scan: Green
    1: (0, 0, 255),    # Trigger: Red
    2: (255, 0, 0)     # Action: Blue
}
LABELS = {0: 'Scan', 1: 'Trigger', 2: 'Action'}

# ==============================================================================
# 4. 主逻辑
# ==============================================================================
def main():
    # === 配置 ===
    camera_id = 0
    # 你的 Backbone 权重
    mae_ckpt_path = '/home/mingcong/project/us_carotid/pose_pilot/pretrain_weights/mae/carotid_base_checkpoint-500_0721.pth' 
    # 你的 Head 权重 (可以是只包含 head 的文件，也可以是之前的完整文件)
    head_ckpt_path = '/home/mingcong/project/us_carotid/scan_pilot/phase_clc/models_save/phase_head.pth'
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # 1. 分别构建模型
    backbone = build_maemodel(mae_ckpt_path, 'mae_vit_base_patch16', device)
    head = build_head(head_ckpt_path, device, num_classes=3)

    # 2. 预处理
    transform = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    # 3. 摄像头初始化
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return

    print("\nStarting Inference Loop (Press 'q' to exit)...")

    # 历史记录 (用于平滑或显示)
    history = deque(maxlen=50)
    prev_time = 0

    while True:
        ret, frame = cap.read()
        if not ret: break

        # === 核心推理流程 ===
        # 1. 预处理
        input_tensor = preprocess_frame(frame, transform).to(device)

        with torch.no_grad():
            # 2. Backbone 提取特征
            # forward_encoder 返回 (latent, mask, ids_restore)，我们需要 latent
            latent, _, _ = backbone.forward_encoder(input_tensor, mask_ratio=0)
            
            # Global Average Pooling (与训练时保持一致: feature = latent.mean(dim=1))
            feature = latent.mean(dim=1)
            
            # 3. Head 分类
            logits = head(feature)
            
            probs = torch.softmax(logits, dim=1)
            conf, idx = torch.max(probs, 1)
            pred_cls = idx.item()
            pred_conf = conf.item()

        # === 可视化 ===
        history.append(pred_cls)
        
        # 简单 UI
        display_h = 600
        h, w = frame.shape[:2]
        scale = display_h / h
        new_w = int(w * scale)
        vis_frame = cv2.resize(frame, (new_w, display_h))

        # 文字
        color = COLORS[pred_cls]
        text = f"{LABELS[pred_cls]} ({pred_conf:.1%})"
        
        # 加个黑底背景让字更清楚
        cv2.rectangle(vis_frame, (0, 0), (new_w, 80), (0,0,0), -1)
        cv2.putText(vis_frame, text, (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)

        # FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if prev_time > 0 else 0
        prev_time = curr_time
        cv2.putText(vis_frame, f"FPS: {fps:.1f}", (new_w - 150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        cv2.imshow('Carotid Classification', vis_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()