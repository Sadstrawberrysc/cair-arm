import os
import argparse
import torch
import torch.nn as nn
import re
import json
import numpy as np
from torchvision import transforms as T
from PIL import Image, ImageDraw, ImageFont
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
# 2. 辅助工具 (报表绘制等)
# ==============================================================================
COLOR_MAP = {
    0: (144, 238, 144),  # Scan: 浅绿色
    1: (255, 69, 0),     # Trigger: 红橙色
    2: (30, 144, 255),   # Action: 蓝色
    -1: (200, 200, 200)  # Unknown: 灰色
}

def natural_key(s):
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", s)]

def load_ground_truth(json_path, image_names_sorted):
    if not json_path or not os.path.exists(json_path):
        return None
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        name_to_label = {}
        if "frames" in data:
            for item in data["frames"]:
                raw_lb = int(item['label'])
                model_lb = raw_lb - 1 if raw_lb > 0 else raw_lb 
                name_to_label[item['name']] = model_lb
        gt_list = []
        for name in image_names_sorted:
            gt_list.append(name_to_label.get(name, 0))
        return gt_list
    except Exception as e:
        print(f"Error loading JSON {json_path}: {e}")
        return None

def generate_comparison_report(seq_name, preds, gts, save_path):
    total_frames = len(preds)
    if total_frames == 0: return

    bar_height = 80
    # 动态调整宽度，避免图片过大或过小
    scale_w = max(2, min(5, 1200 // total_frames)) 
    img_w = total_frames * scale_w + 40 
    img_h = (bar_height * 2) + 120 
    
    canvas = Image.new('RGB', (img_w, img_h), (255, 255, 255))
    draw = ImageDraw.Draw(canvas)
    
    try:
        font_large = ImageFont.truetype("arial.ttf", 24)
        font_small = ImageFont.truetype("arial.ttf", 16)
    except:
        font_large = ImageFont.load_default()
        font_small = ImageFont.load_default()

    draw.text((20, 10), f"Sequence: {seq_name}", fill='black', font=font_large)
    
    def draw_bar(labels, y_start, title):
        draw.text((20, y_start - 25), title, fill='black', font=font_small)
        if labels is None:
            draw.text((100, y_start + 30), "GT File Missing", fill='gray', font=font_large)
            return
        for i, lb in enumerate(labels):
            color = COLOR_MAP.get(lb, (128,128,128))
            x0 = 20 + i * scale_w
            x1 = x0 + scale_w
            y0 = y_start
            y1 = y_start + bar_height
            draw.rectangle([x0, y0, x1, y1], fill=color, outline=None)
            # 如果类别变化，画一条小白线分隔
            if i > 0 and labels[i] != labels[i-1]:
                draw.line([(x0, y0), (x0, y1)], fill='white', width=1)

    draw_bar(gts, 60, "Ground Truth")
    draw_bar(preds, 60 + bar_height + 50, "Model Prediction")

    legend_y = img_h - 30
    draw.text((20, legend_y), "Legend:", fill='black', font=font_small)
    legends = [("Scan", 0), ("Trigger", 1), ("Action", 2)]
    curr_x = 100
    for name, code in legends:
        c = COLOR_MAP[code]
        draw.rectangle([curr_x, legend_y, curr_x+20, legend_y+20], fill=c, outline='black')
        draw.text((curr_x+25, legend_y), name, fill='black', font=font_small)
        curr_x += 120
    canvas.save(save_path)

# ==============================================================================
# 3. 核心逻辑 (加载模型与推理)
# ==============================================================================

def load_model(args, device):
    print(f"Initializing Network Structure: {args.maemodel} ...")
    model = CarotidClassifier(backbone_name=args.maemodel, num_classes=3).to(device)
    
    # --- 1. 加载 MAE Backbone (底座) ---
    if args.mae_checkpoint and os.path.isfile(args.mae_checkpoint):
        print(f"Loading MAE Backbone from: {args.mae_checkpoint}")
        mae_ckpt = torch.load(args.mae_checkpoint, map_location='cpu')
        # 处理可能的 key 不匹配
        mae_state = mae_ckpt['model'] if 'model' in mae_ckpt else mae_ckpt
        # strict=False 因为 MAE 权重里没有 head
        msg = model.backbone.load_state_dict(mae_state, strict=False)
        print(f"MAE Backbone Loaded: {msg}")
    else:
        print("Warning: No MAE backbone checkpoint provided. If using Head-Only model, results will be random!")

    # --- 2. 加载 Task Checkpoint (Head 或 Full) ---
    print(f"Loading Task Checkpoint: {args.checkpoint} ...")
    if not os.path.exists(args.checkpoint):
        print(f"Error: Checkpoint {args.checkpoint} not found.")
        exit(1)
        
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
            
        model.load_state_dict(new_state_dict, strict=False)

    model.eval()
    return model

def parse_test_root(root_path):
    dataset_items = []
    if not os.path.exists(root_path): return []
    # 假设结构: root/seq1/images/*.jpg
    subfiles = [os.path.join(root_path, d) for d in os.listdir(root_path) if os.path.isdir(os.path.join(root_path, d))]
    
    for subfile_path in sorted(subfiles):
        subfile_name = os.path.basename(subfile_path)
        json_file = os.path.join(subfile_path, "label_carotid.json")
        if not os.path.exists(json_file): json_file = None
        
        # 有些数据集结构里 images 文件夹可能在下一级
        # 这里尝试找 'images' 文件夹，或者直接用当前文件夹
        if os.path.exists(os.path.join(subfile_path, 'images')):
             img_dir = os.path.join(subfile_path, 'images')
        else:
            # 兼容你的第一段代码里的结构
            inner_dirs = [os.path.join(subfile_path, d) for d in os.listdir(subfile_path) if os.path.isdir(os.path.join(subfile_path, d))]
            img_dir = inner_dirs[0] if len(inner_dirs) > 0 else subfile_path
        
        dataset_items.append({"name": subfile_name, "img_dir": img_dir, "json_path": json_file})
    return dataset_items

def process_single_sequence(model, item_info, transform, device, trigger_threshold=None):
    """
    trigger_threshold: float. 如果 Trigger (Class 1) 的概率 > threshold，则强制预测为 1。
    """
    img_dir = item_info['img_dir']
    json_path = item_info['json_path']
    
    valid_ext = {".jpg", ".jpeg", ".png", ".bmp"}
    img_files = [f for f in os.listdir(img_dir) if os.path.splitext(f)[1].lower() in valid_ext]
    img_files.sort(key=natural_key) 
    
    if not img_files: return [], []

    gt_labels = load_ground_truth(json_path, img_files)
    preds = []
    BATCH_SIZE = 32 
    
    for i in range(0, len(img_files), BATCH_SIZE):
        batch_files = img_files[i : i + BATCH_SIZE]
        batch_tensors = []
        for img_name in batch_files:
            img_path = os.path.join(img_dir, img_name)
            try:
                raw_img = Image.open(img_path).convert('RGB')
                batch_tensors.append(transform(raw_img))
            except:
                # 坏图处理
                batch_tensors.append(torch.zeros(3, 224, 224))
        
        if not batch_tensors: continue
        
        input_batch = torch.stack(batch_tensors).to(device)
        
        with torch.no_grad():
            outputs = model(input_batch)
            
            if trigger_threshold is not None:
                # --- 自定义阈值逻辑 ---
                probs = torch.softmax(outputs, dim=1)
                
                prob_trigger = probs[:, 1]
                prob_scan = probs[:, 0]
                prob_action = probs[:, 2]
                
                # Scan vs Action
                pred_others = torch.where(prob_scan > prob_action, 
                                          torch.tensor(0, device=device), 
                                          torch.tensor(2, device=device))
                
                # Trigger Logic
                batch_preds_tensor = torch.where(
                    prob_trigger > trigger_threshold,
                    torch.tensor(1, device=device),
                    pred_others
                )
                batch_preds = batch_preds_tensor.cpu().numpy().tolist()
            else:
                batch_preds = outputs.argmax(dim=1).cpu().numpy().tolist()
                
            preds.extend(batch_preds)

    return preds, gt_labels

def main():
    parser = argparse.ArgumentParser()
    # 数据集路径
    parser.add_argument('--test_root', type=str, default="/home/mingcong/project/us_carotid/scan_pilot/phase_clc/kaili_data/lable_carotid/carotid_dataset_v1/test")
    
    # 路径：Head 模型
    parser.add_argument('--checkpoint', type=str, default="/home/mingcong/project/us_carotid/scan_pilot/phase_clc/models_save/phase_head.pth", help="Head-only or Full model path")
    
    # 路径：MAE Backbone
    parser.add_argument('--mae_checkpoint', type=str, default='/home/mingcong/project/us_carotid/pose_pilot/pretrain_weights/mae/carotid_base_checkpoint-500_0721.pth', help="MAE Backbone path")

    parser.add_argument('--maemodel', default='mae_vit_base_patch16')
    parser.add_argument('--output_dir', type=str, default='./inference_reports')
    
    # 阈值参数
    parser.add_argument('--trigger_threshold', type=float, default=0.7, 
                        help='Confidence threshold for Trigger (label 1).')
    
    args = parser.parse_args()
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # 加载组合模型
    model = load_model(args, device)
    
    transform = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    
    print(f"Scanning structure in {args.test_root}...")
    dataset_items = parse_test_root(args.test_root)
    print(f"Found {len(dataset_items)} valid sequences.")

    if args.trigger_threshold is not None:
        print(f"!!! Using customized Trigger Threshold: {args.trigger_threshold} !!!")
    
    os.makedirs(args.output_dir, exist_ok=True)
    
    for item in tqdm(dataset_items, desc="Evaluating"):
        preds, gts = process_single_sequence(model, item, transform, device, args.trigger_threshold)
        
        if preds:
            safe_name = item['name'].replace(os.sep, "_")
            save_path = os.path.join(args.output_dir, f"compare_{safe_name}.png")
            generate_comparison_report(safe_name, preds, gts, save_path)
            
    print(f"\nDone! Check reports in: {args.output_dir}")

if __name__ == "__main__":
    main()