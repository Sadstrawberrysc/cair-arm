import os
import argparse
import glob
import re
import numpy as np
import cv2
import torch
import torch.nn as nn
from torchvision import transforms as T
from PIL import Image
from tqdm import tqdm

# 尝试导入 models_mae
try:
    from models import models_mae
except ImportError:
    print("Warning: models_mae not found. Please ensure it is in path.")
    models_mae = None

# ==============================================================================
# 1. 模型定义
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

class CarotidClassifier(nn.Module):
    def __init__(self, backbone_name='mae_vit_base_patch16', num_classes=3):
        super().__init__()
        # 初始化 Backbone (随机权重)
        self.backbone = getattr(models_mae, backbone_name)(norm_pix_loss=False)
        self.head = SimpleClassHead(in_dim=768, out_dim=num_classes, dropout=0.5)

    def forward(self, x):
        latent, _, _ = self.backbone.forward_encoder(x, mask_ratio=0)
        feature = latent.mean(dim=1)
        out = self.head(feature)
        return out

# ==============================================================================
# 2. 工具函数 (核心修改在这里)
# ==============================================================================

def natural_sort_key(s):
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('([0-9]+)', s)]

def load_model(args, device):
    print(f"Initializing Network Structure: {args.maemodel} ...")
    model = CarotidClassifier(backbone_name=args.maemodel, num_classes=3).to(device)
    
    # -----------------------------------------------------------
    # 步骤 1: 加载通用的 MAE Backbone (如果指定)
    # -----------------------------------------------------------
    # 如果你是用 Head-Only 模式，必须加载这个，否则 Backbone 是随机初始化的，结果会全是错的
    if args.mae_checkpoint and os.path.isfile(args.mae_checkpoint):
        print(f"Loading MAE Backbone from: {args.mae_checkpoint}")
        mae_ckpt = torch.load(args.mae_checkpoint, map_location='cpu')
        # 处理可能的 key 不匹配 ('model' 或 直接是 state_dict)
        mae_state = mae_ckpt['model'] if 'model' in mae_ckpt else mae_ckpt
        # strict=False 因为 MAE 权重里没有 head
        msg = model.backbone.load_state_dict(mae_state, strict=False)
        print(f"MAE Backbone Loaded: {msg}")
    else:
        print("Warning: No MAE backbone checkpoint provided. If using Head-Only model, results will be random!")

    # -----------------------------------------------------------
    # 步骤 2: 加载任务特定的 Checkpoint (Head 或 Full)
    # -----------------------------------------------------------
    print(f"Loading Task Checkpoint: {args.checkpoint} ...")
    checkpoint = torch.load(args.checkpoint, map_location='cpu')
    
    # 【新逻辑】判断是否为 Head-Only 模型
    if 'head' in checkpoint and isinstance(checkpoint['head'], dict):
        print(">>> Detected Head-Only checkpoint. Loading head weights...")
        model.head.load_state_dict(checkpoint['head'])
        
    # 【旧逻辑】兼容完整模型
    else:
        print(">>> Detected Full Model checkpoint. Loading all weights...")
        state_dict = checkpoint['model'] if 'model' in checkpoint else checkpoint
        
        # 移除 DDP 的 module. 前缀
        new_state_dict = {}
        for k, v in state_dict.items():
            name = k.replace("module.", "")
            new_state_dict[name] = v
            
        # strict=False 允许一些不匹配 (比如 backbone 版本微小差异)
        model.load_state_dict(new_state_dict, strict=False)

    model.eval()
    return model

# 颜色定义
COLORS = {
    0: (0, 255, 0),    # Scan: Green
    1: (0, 0, 255),    # Trigger: Red
    2: (255, 0, 0)     # Action: Blue
}
LABELS = {0: 'Scan', 1: 'Trigger', 2: 'Action'}

# ==============================================================================
# 3. 主逻辑
# ==============================================================================
def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--img_size', type=int, default=512, help='Display size (height)')
    
    # 数据路径
    parser.add_argument('--input_dir', type=str, default='/home/mingcong/project/us_carotid/scan_pilot/phase_clc/kaili_data/lable_carotid/carotid_dataset_v1/test/004_right_data24/images', help='Path to image directory')
    
    # 模型路径
    # 1. 你的新 Head 权重
    parser.add_argument('--checkpoint', type=str, default='/home/mingcong/project/us_carotid/scan_pilot/phase_clc/models_save/phase_head.pth', help='Path to .pth (Head-Only or Full)')
    # 2. 你的通用 MAE 底座权重 (对于 Head-Only 模式必须提供)
    parser.add_argument('--mae_checkpoint', type=str, default='/home/mingcong/project/us_carotid/pose_pilot/pretrain_weights/mae/carotid_base_checkpoint-500_0721.pth', help='Path to pretrained MAE backbone .pth')
    
    parser.add_argument('--maemodel', default='mae_vit_base_patch16', help='Backbone name')

    args = parser.parse_args()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # 1. 查找图片
    exts = ['*.jpg', '*.png', '*.bmp', '*.jpeg']
    img_files = []
    for ext in exts:
        img_files.extend(glob.glob(os.path.join(args.input_dir, ext)))
    
    if not img_files:
        print("No images found in directory!")
        return
        
    img_files.sort(key=natural_sort_key)
    print(f"Found {len(img_files)} frames.")

    # 2. 加载模型 (核心修改已生效)
    model = load_model(args, device)

    # 3. 预处理
    transform = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    # 4. [Pass 1] 批量推理
    print("Running inference on all frames (Pass 1)...")
    predictions = [] 
    
    batch_size = 32
    for i in tqdm(range(0, len(img_files), batch_size)):
        batch_files = img_files[i : i+batch_size]
        batch_tensors = []
        
        for f in batch_files:
            try:
                img = Image.open(f).convert('RGB')
                batch_tensors.append(transform(img))
            except Exception as e:
                print(f"Error reading {f}: {e}")
                
        if not batch_tensors:
            continue
            
        input_tensor = torch.stack(batch_tensors).to(device)
        
        with torch.no_grad():
            logits = model(input_tensor)
            probs = torch.softmax(logits, dim=1)
            confs, idxs = torch.max(probs, 1)
            
            for c, idx in zip(confs, idxs):
                predictions.append((idx.item(), c.item()))

    if not predictions:
        print("No predictions made.")
        return

    # 5. [Pass 2] 可视化循环
    print("\nStarting Viewer...")
    print("Controls: [A] Left | [D] Right | [Q] Quit")

    idx = 0
    num_frames = len(predictions) # 使用实际预测的数量
    
    DISP_H = args.img_size
    BAR_H = 50 

    while True:
        if idx >= len(img_files): break

        img_path = img_files[idx]
        pred_cls, pred_conf = predictions[idx]
        
        frame = cv2.imread(img_path)
        if frame is None:
            break
            
        h, w = frame.shape[:2]
        scale = DISP_H / h
        new_w = int(w * scale)
        frame_resized = cv2.resize(frame, (new_w, DISP_H))

        # 绘制底部时间轴
        bar_img = np.zeros((BAR_H, new_w, 3), dtype=np.uint8)
        step = new_w / num_frames
        
        for i in range(num_frames):
            p_cls, _ = predictions[i]
            c = COLORS[p_cls]
            x1 = int(i * step)
            x2 = int((i + 1) * step) + 1 # +1 防止缝隙
            cv2.rectangle(bar_img, (x1, 0), (x2, BAR_H), c, -1)
            
        cur_x = int(idx * step) + int(step/2)
        cv2.line(bar_img, (cur_x, 0), (cur_x, BAR_H), (255, 255, 255), 2)
        
        final_vis = np.vstack([frame_resized, bar_img])

        # 文字覆盖
        label_text = f"{LABELS[pred_cls]} ({pred_conf:.1%})"
        frame_info = f"Frame: {idx+1}/{num_frames}"
        
        overlay = final_vis.copy()
        cv2.rectangle(overlay, (0, 0), (new_w, 80), (0, 0, 0), -1)
        final_vis = cv2.addWeighted(overlay, 0.6, final_vis, 0.4, 0)
        
        cv2.putText(final_vis, label_text, (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, COLORS[pred_cls], 3)
        cv2.putText(final_vis, frame_info, (20, 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow('Carotid Sequence Viewer', final_vis)

        key = cv2.waitKey(0) & 0xFF
        if key == 27 or key == ord('q'): 
            break
        elif key == ord('a') or key == 81: 
            idx = max(0, idx - 1)
        elif key == ord('d') or key == 83: 
            idx = min(num_frames - 1, idx + 1)
            
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()