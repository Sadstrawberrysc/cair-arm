import os
import time
import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as T
from PIL import Image
from collections import deque
import torch.nn.functional as F

# 尝试导入 models_mae
try:
    from models import models_mae
except ImportError:
    print("Warning: models_mae not found. Please ensure it is in path.")
    models_mae = None

# ==============================================================================
# 1. 独立的 Head 定义
# ==============================================================================
# class SimpleClassHead(nn.Module):
#     def __init__(self, in_dim=768, out_dim=3, dropout=0.5):
#         super(SimpleClassHead, self).__init__()
#         self.bn = nn.BatchNorm1d(in_dim)
#         self.dropout = nn.Dropout(dropout)
#         self.fc = nn.Linear(in_dim, out_dim)

#     def forward(self, x):
#         x = self.bn(x)
#         x = self.dropout(x)
#         x = self.fc(x)
#         return x

class RESNetHead(nn.Module):
    def __init__(self, in_dim=768, out_dim=3, dropout=0.2, hidden_dim=512):
        super(RESNetHead, self).__init__()
        # 输入层归一化
        self.bn1 = nn.BatchNorm1d(in_dim)
        
        # 第一个残差块：in_dim -> hidden_dim
        self.fc1 = nn.Linear(in_dim, hidden_dim)
        self.bn2 = nn.BatchNorm1d(hidden_dim)
        self.dropout1 = nn.Dropout(dropout)
        
        # 第二个残差块：hidden_dim -> hidden_dim
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.bn3 = nn.BatchNorm1d(hidden_dim)
        self.dropout2 = nn.Dropout(dropout)
        
        # 用于跳连的投影层（当输入维度不等于隐藏维度时）
        self.shortcut_proj = nn.Linear(in_dim, hidden_dim) if in_dim != hidden_dim else None
        
        # 输出层
        self.fc3 = nn.Linear(hidden_dim, out_dim)
        
        # 权重初始化
        nn.init.normal_(self.fc1.weight, std=0.01)
        nn.init.constant_(self.fc1.bias, 0)
        nn.init.normal_(self.fc2.weight, std=0.01)
        nn.init.constant_(self.fc2.bias, 0)
        nn.init.normal_(self.fc3.weight, std=0.01)
        nn.init.constant_(self.fc3.bias, 0)
        if self.shortcut_proj:
            nn.init.normal_(self.shortcut_proj.weight, std=0.01)
            nn.init.constant_(self.shortcut_proj.bias, 0)

    def forward(self, x):
        # x shape: [Batch, 768]
        identity_input = x
        
        # 输入层：bn
        x = self.bn1(x)
        
        # 第一个残差块
        # 投影输入用于跳连
        if self.shortcut_proj:
            shortcut = self.shortcut_proj(identity_input)
        else:
            shortcut = identity_input
        
        x = self.fc1(x)
        x = self.bn2(x)
        x = F.relu(x)
        
        # 第二个残差块 + 跳连
        identity = x
        x = self.fc2(x)
        x = self.bn3(x)
        x = x + identity  # 残差连接
        x = F.relu(x)
        x = self.dropout2(x)
        
        # 输出层
        x = self.fc3(x)
        return x

# ==============================================================================
# 2. 模型构建辅助函数
# ==============================================================================
def build_maemodel(maecheckpoint, maemodel_name, device):
    """构建并加载 Backbone"""
    print(f"Building Backbone: {maemodel_name} ...")
    model = getattr(models_mae, maemodel_name)(norm_pix_loss=False).to(device).eval()
    
    if os.path.isfile(maecheckpoint):
        # Handle PyTorch 2.6 weights_only restriction with argparse.Namespace
        try:
            ckpt = torch.load(maecheckpoint, map_location='cpu')
        except Exception as e:
            if 'argparse.Namespace' in str(e):
                import argparse as _argparse
                try:
                    if hasattr(torch.serialization, 'safe_globals'):
                        with torch.serialization.safe_globals([_argparse.Namespace]):
                            ckpt = torch.load(maecheckpoint, map_location='cpu')
                    else:
                        torch.serialization.add_safe_globals([_argparse.Namespace])
                        ckpt = torch.load(maecheckpoint, map_location='cpu')
                except Exception as e2:
                    ckpt = torch.load(maecheckpoint, map_location='cpu', weights_only=False)
            else:
                raise
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
    head = RESNetHead(in_dim=768, out_dim=num_classes, dropout=0.2).to(device).eval()
    
    if os.path.isfile(checkpoint_path):
        print(f"Loading Head weights from: {checkpoint_path}")
        # Handle PyTorch 2.6 weights_only restriction with argparse.Namespace
        try:
            ckpt = torch.load(checkpoint_path, map_location='cpu')
        except Exception as e:
            if 'argparse.Namespace' in str(e):
                import argparse as _argparse
                try:
                    if hasattr(torch.serialization, 'safe_globals'):
                        with torch.serialization.safe_globals([_argparse.Namespace]):
                            ckpt = torch.load(checkpoint_path, map_location='cpu')
                    else:
                        torch.serialization.add_safe_globals([_argparse.Namespace])
                        ckpt = torch.load(checkpoint_path, map_location='cpu')
                except Exception as e2:
                    ckpt = torch.load(checkpoint_path, map_location='cpu', weights_only=False)
            else:
                raise
        
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
    mae_ckpt_path = '/home/cair/Projects/demo/uspilot_ctrl/scan_pilot/intergrate_infer/weights/backbone/carotid_base_checkpoint-500_0721.pth' 
    # 你的 Head 权重 (可以是只包含 head 的文件，也可以是之前的完整文件)
    head_ckpt_path = './best_trigger_model.pth'
    
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
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

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
        frame = cv2.flip(frame, 0)
        frame = frame[160:680,595:1415,:]
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



       # # 3. 加载测试数据集图片
    # if not os.path.isdir(test_data_path):
    #     print(f"Error: Test dataset path not found: {test_data_path}")
    #     return
    
    # # 递归收集所有图片
    # image_files = []
    # for root, dirs, files in os.walk(test_data_path):
    #     for file in files:
    #         if file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
    #             image_files.append(os.path.join(root, file))
    
    # if not image_files:
    #     print(f"Error: No images found in {test_data_path}")
    #     return
    
    # image_files.sort()
    # print(f"\nFound {len(image_files)} images. Starting inference...")
    
    # # 推理结果统计
    # results = []
    
    # for idx, img_path in enumerate(image_files):
    #     try:
    #         # 读取图片
    #         frame = cv2.imread(img_path)
    #         if frame is None:
    #             print(f"[{idx+1}/{len(image_files)}] Failed to read: {img_path}")
    #             continue
            
    #         # === 核心推理流程 ===
    #         input_tensor = preprocess_frame(frame, transform).to(device)

    #         with torch.no_grad():
    #             # 提取特征
    #             latent, _, _ = backbone.forward_encoder(input_tensor, mask_ratio=0)
    #             feature = latent.mean(dim=1)
                
    #             # 分类
    #             logits = head(feature)
    #             probs = torch.softmax(logits, dim=1)
    #             conf, idx_cls = torch.max(probs, 1)
    #             pred_cls = idx_cls.item()
    #             pred_conf = conf.item()

    #         # 保存结果
    #         results.append({
    #             'image': os.path.basename(img_path),
    #             'pred_class': LABELS[pred_cls],
    #             'confidence': pred_conf,
    #             'class_idx': pred_cls
    #         })
            
    #         # === 显示 ===
    #         display_h = 600
    #         h, w = frame.shape[:2]
    #         scale = display_h / h
    #         new_w = int(w * scale)
    #         vis_frame = cv2.resize(frame, (new_w, display_h))

    #         color = COLORS[pred_cls]
    #         text = f"{LABELS[pred_cls]} ({pred_conf:.1%})"
            
    #         cv2.rectangle(vis_frame, (0, 0), (new_w, 80), (0,0,0), -1)
    #         cv2.putText(vis_frame, text, (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)
    #         cv2.putText(vis_frame, f"[{idx+1}/{len(image_files)}]", (new_w - 200, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

    #         # 跳过显示窗口（headless 模式）
    #         # cv2.imshow('Carotid Classification - Test Dataset', vis_frame)
    #         # key = cv2.waitKey(0) & 0xFF
    #         # if key == ord('q'):
    #         #     break
            
    #         print(f"[{idx+1}/{len(image_files)}] Processed: {os.path.basename(img_path)}")
                
    #     except Exception as e:
    #         print(f"Error processing {img_path}: {e}")
    #         continue

    # cv2.destroyAllWindows()
    
    # # 打印统计
    # print("\n" + "="*60)
    # print("Inference Results Summary:")
    # print("="*60)
    # for i, result in enumerate(results):
    #     print(f"[{i+1}] {result['image']:40s} -> {result['pred_class']:8s} ({result['confidence']:.1%})")
    
    # # 类别统计
    # class_counts = {}
    # for result in results:
    #     class_name = result['pred_class']
    #     class_counts[class_name] = class_counts.get(class_name, 0) + 1
    
    # print("\nClass Distribution:")
    # for cls_name, count in class_counts.items():
    #     print(f"  {cls_name}: {count} ({count/len(results)*100:.1f}%)")
    
    # # 保存结果到文件
    # output_dir = './inference_results'
    # os.makedirs(output_dir, exist_ok=True)
    
    # # 保存为 CSV
    # import csv
    # csv_path = os.path.join(output_dir, 'predictions.csv')
    # with open(csv_path, 'w', newline='', encoding='utf-8') as f:
    #     writer = csv.DictWriter(f, fieldnames=['image', 'pred_class', 'confidence', 'class_idx'])
    #     writer.writeheader()
    #     writer.writerows(results)
    # print(f"\n✓ Results saved to CSV: {csv_path}")
    
    # # 保存为 JSON
    # import json
    # json_path = os.path.join(output_dir, 'predictions.json')
    # with open(json_path, 'w', encoding='utf-8') as f:
    #     json.dump({
    #         'total_images': len(results),
    #         'predictions': results,
    #         'class_distribution': class_counts
    #     }, f, indent=2, ensure_ascii=False)
    # print(f"✓ Results saved to JSON: {json_path}")
    
    # print(f"\nAll results saved to: {os.path.abspath(output_dir)}/")