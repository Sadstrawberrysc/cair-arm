import os
import argparse
import csv
import math
import datetime
import json
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter
import torchvision.transforms as T
import timm.optim.optim_factory as optim_factory
from PIL import Image
from tqdm import tqdm
from pathlib import Path

# ================= ADDED: WandB =================
import wandb
# ================================================

# 引入你的模型定义
try:
    from models import models_mae
except ImportError:
    # 简单的 fallback，防止导入报错，实际运行时请确保路径正确
    print("Warning: models_mae not found, please ensure it is in the python path.")
    models_mae = None

# ==============================================================================
# 1. Loss 定义
# ==============================================================================
class FocalLoss(nn.Module):
    def __init__(self, alpha=None, gamma=2.0, reduction='mean'):
        super(FocalLoss, self).__init__()
        self.alpha = alpha 
        self.gamma = gamma
        self.reduction = reduction

    def forward(self, inputs, targets):
        ce_loss = F.cross_entropy(inputs, targets, reduction='none', weight=self.alpha)
        pt = torch.exp(-ce_loss)
        focal_loss = ((1 - pt) ** self.gamma) * ce_loss

        if self.reduction == 'mean':
            return focal_loss.mean()
        elif self.reduction == 'sum':
            return focal_loss.sum()
        else:
            return focal_loss

# ==============================================================================
# 2. 定义数据加载器 (CarotidDataset)
# ==============================================================================
class CarotidDataset(Dataset):
    def __init__(self, root_dir, split='train', transform=None):
        self.root_dir = os.path.join(root_dir, split)
        self.transform = transform
        self.samples = [] 
        self.raw_counts = {} 
        self._load_data()

    def _load_data(self):
        if not os.path.exists(self.root_dir):
            print(f"Warning: {self.root_dir} does not exist.")
            return
        
        seq_dirs = [d for d in os.listdir(self.root_dir) if os.path.isdir(os.path.join(self.root_dir, d))]
        valid_raw_labels = {1, 2, 3}
        
        for seq_name in seq_dirs:
            seq_path = os.path.join(self.root_dir, seq_name)
            json_path = os.path.join(seq_path, 'label_carotid.json')
            
            try:
                with open(json_path, 'r', encoding='utf-8') as f:
                    label_data = json.load(f)
                    frames = label_data.get('frames', [])
                    
                    for item in frames:
                        raw_label = int(item['label'])
                        if raw_label not in valid_raw_labels:
                            continue 
                        # 映射标签：1->0, 2->1, 3->2
                        mapped_label = raw_label - 1
                        
                        img_path = os.path.join(seq_path, 'images', item['name'])
                        if os.path.exists(img_path):
                            self.samples.append((img_path, mapped_label))
                            self.raw_counts[raw_label] = self.raw_counts.get(raw_label, 0) + 1
                            
            except Exception as e:
                pass
        
        print(f"Dataset Loaded '{os.path.basename(self.root_dir)}': Found raw labels {self.raw_counts}")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img_path, label = self.samples[idx]
        image = Image.open(img_path).convert('RGB')
        if self.transform:
            image = self.transform(image)
        return image, label

    def get_class_counts(self):
        counts = {0: 0, 1: 0, 2: 0}
        for _, label in self.samples:
            if label in counts:
                counts[label] += 1
        return counts

# ==============================================================================
# 3. 定义简单的 Head (2D Input)
# ==============================================================================
class SimpleClassHead(nn.Module):
    def __init__(self, in_dim=768, out_dim=3, dropout=0.5):
        super(SimpleClassHead, self).__init__()
        # 这里的输入是 [Batch, 768]，直接进全连接
        self.bn = nn.BatchNorm1d(in_dim)
        self.dropout = nn.Dropout(dropout)
        self.fc = nn.Linear(in_dim, out_dim)
        
        nn.init.normal_(self.fc.weight, std=0.01)
        nn.init.constant_(self.fc.bias, 0)

    def forward(self, x):
        # x shape: [Batch, 768]
        x = self.bn(x)
        x = self.dropout(x)
        x = self.fc(x)
        return x

# ==============================================================================
# 4. 组合模型 (VirMAE Backbone + SimpleHead)
# ==============================================================================
class CarotidClassifier(nn.Module):
    def __init__(self, backbone, num_classes=3):
        super().__init__()
        # 接收外部构建好的 backbone
        self.backbone = backbone
        
        # 强制使用 SimpleClassHead 以匹配 GAP 后的 2D 特征
        # 这样就彻底解决了 "expected 3, got 2" 的报错
        self.head = SimpleClassHead(in_dim=768, out_dim=num_classes, dropout=0.5)

    def forward(self, x):
        # 1. Backbone 提取特征
        # MAE forward_encoder 返回 (latent, mask, ids_restore)
        # latent shape: [B, 197, 768]
        latent, _, _ = self.backbone.forward_encoder(x, mask_ratio=0)
        
        # 2. 【关键修复】Global Average Pooling
        # 对 Patch 维度 (dim=1) 求平均，变成 [B, 768]
        feature = latent.mean(dim=1) 
        
        # 3. 传入 Head
        out = self.head(feature)
        return out

def build_maemodel(args, device):
    print(f"Loading MAE Backbone: {args.maemodel} ...")
    model = getattr(models_mae, args.maemodel)(norm_pix_loss=False).to(device)
    
    if args.maecheckpoint and os.path.isfile(args.maecheckpoint):
        checkpoint = torch.load(args.maecheckpoint, map_location='cpu')
        # 处理可能的 key 不匹配
        state_dict = checkpoint['model'] if 'model' in checkpoint else checkpoint
        msg = model.load_state_dict(state_dict, strict=False)
        print(f"MAE Pretrained Loaded: {msg}")
    
    return model

# ==============================================================================
# 5. 辅助函数
# ==============================================================================
class AMPScaler:
    def __init__(self):
        self.scaler = torch.cuda.amp.GradScaler(enabled=torch.cuda.is_available())

    def step(self, loss, optimizer, clip_grad=None, parameters=None, create_graph=False):
        self.scaler.scale(loss).backward(create_graph=create_graph)
        if clip_grad is not None:
            self.scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(parameters, clip_grad)
        self.scaler.step(optimizer)
        self.scaler.update()
        optimizer.zero_grad(set_to_none=True)

def set_randomness(seed):
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    np.random.seed(seed)

def get_args():
    parser = argparse.ArgumentParser('Carotid Classification Training', add_help=False)
    
    # Training Params
    parser.add_argument('--epochs', default=50, type=int)
    parser.add_argument('--batch_size', default=128, type=int)
    parser.add_argument('--accum_iter', default=1, type=int)
    parser.add_argument('--lr', default=5e-5, type=float)
    parser.add_argument('--weight_decay', default=0.1, type=float)
    parser.add_argument('--warmup_epochs', default=5, type=int)

    # Model Params
    parser.add_argument('--maemodel', default='mae_vit_base_patch16')
    parser.add_argument('--maecheckpoint', default='/home/mingcong/project/us_carotid/pose_pilot/pretrain_weights/mae/carotid_base_checkpoint-500_0721.pth')
    # 建议这里先试 0，如果你执意要冻结，可以改回 1，但注意前面讨论的风险
    parser.add_argument('--freeze_encoder', type=int, default=1, help="0=finetune all, 1=freeze backbone")

    # Data Params
    parser.add_argument('--data_rootdir', default='/home/mingcong/project/us_carotid/scan_pilot/kaili_data/lable_carotid/carotid_dataset_v1/')
    parser.add_argument('--out_dir', default='./runs/cls_v1')
    parser.add_argument('--num_workers', default=8, type=int)
    parser.add_argument('--seed', default=42, type=int)
    
    # WandB
    parser.add_argument('--wandb', type=int, default=1)
    parser.add_argument('--project', type=str, default='Carotid-Classification')
    parser.add_argument('--run_name', type=str, default=None)

    return parser.parse_args()

# ==============================================================================
# 6. 主函数
# ==============================================================================
def main():
    args = get_args()
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    cudnn.benchmark = True
    set_randomness(args.seed)

    # Setup Output Dir
    if args.run_name is None:
        args.run_name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.out_dir, args.run_name)
    os.makedirs(out_dir, exist_ok=True)
    
    if args.wandb:
        wandb.init(project=args.project, name=args.run_name, config=vars(args))

    print(f"Training on {device}, Output: {out_dir}")

    # --- Dataset ---
    transform_train = T.Compose([
        T.Resize((224, 224)),
        T.RandomHorizontalFlip(),
        T.ColorJitter(brightness=0.2, contrast=0.2),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    transform_val = T.Compose([
        T.Resize((224, 224)),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    train_set = CarotidDataset(args.data_rootdir, split='train', transform=transform_train)
    val_set = CarotidDataset(args.data_rootdir, split='val', transform=transform_val)
    
    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True, 
                              num_workers=args.num_workers, pin_memory=True, drop_last=True)
    val_loader = DataLoader(val_set, batch_size=args.batch_size, shuffle=False, 
                            num_workers=args.num_workers, pin_memory=True)

    print(f"Data: Train={len(train_set)}, Val={len(val_set)}")

    # --- Class Weights ---
    counts = train_set.get_class_counts()
    print(f"Class Distribution: {counts}")
    total_samples = sum(counts.values())
    weights = [total_samples / (3 * max(1, counts[i])) for i in range(3)]
    weights[1] *= 1.2 
    class_weights = torch.FloatTensor(weights).to(device)

    # --- Model Build ---
    # 1. 构建 backbone
    backbone = build_maemodel(args, device)
    
    # 2. 构建分类器 (注意：这里直接传入 backbone，不需要再传 head，类内部会自己建)
    model = CarotidClassifier(backbone, num_classes=3).to(device)

    # Freeze Backbone Logic
    if args.freeze_encoder:
        for p in model.backbone.parameters():
            p.requires_grad = False
        print("Backbone Freezed.")
    else:
        print("Backbone Trainable.")

    # --- Optimizer ---
    param_groups = optim_factory.param_groups_weight_decay(model, args.weight_decay)
    optimizer = torch.optim.AdamW(param_groups, lr=args.lr, betas=(0.9, 0.95))
    loss_scaler = AMPScaler()
    criterion = FocalLoss(alpha=class_weights, gamma=2.0)

    # --- Scheduler ---
    def lr_lambda(epoch):
        if epoch < args.warmup_epochs:
            return (epoch + 1) / args.warmup_epochs
        progress = (epoch - args.warmup_epochs) / (args.epochs - args.warmup_epochs)
        return 0.5 * (1 + math.cos(math.pi * progress))
    
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)

    # --- Loop ---
    best_recall = 0.0

    for epoch in range(args.epochs):
        model.train()
        train_loss = 0.0
        train_acc = 0.0
        total_train = 0

        pbar = tqdm(train_loader, desc=f"Ep {epoch+1}/{args.epochs}", ncols=100)
        
        for imgs, labels in pbar:
            imgs, labels = imgs.to(device), labels.to(device)

            with torch.cuda.amp.autocast():
                logits = model(imgs) # [B, 3]
                loss = criterion(logits, labels)
                loss = loss / args.accum_iter

            loss_scaler.step(loss, optimizer, clip_grad=None)
            
            acc = (logits.argmax(dim=1) == labels).float().sum()
            train_loss += loss.item() * args.accum_iter * imgs.size(0)
            train_acc += acc.item()
            total_train += imgs.size(0)
            
            pbar.set_postfix({'loss': loss.item() * args.accum_iter})

        scheduler.step()
        
        # --- Validation ---
        model.eval()
        val_loss = 0.0
        val_acc = 0.0
        total_val = 0
        trigger_tp = 0
        trigger_gt = 0

        with torch.no_grad():
            for imgs, labels in val_loader:
                imgs, labels = imgs.to(device), labels.to(device)
                
                with torch.cuda.amp.autocast():
                    logits = model(imgs)
                    loss = criterion(logits, labels)
                
                preds = logits.argmax(dim=1)
                val_loss += loss.item() * imgs.size(0)
                val_acc += (preds == labels).float().sum().item()
                total_val += imgs.size(0)
                
                is_trigger = (labels == 1)
                trigger_gt += is_trigger.sum().item()
                trigger_tp += (preds[is_trigger] == 1).sum().item()

        epoch_train_loss = train_loss / total_train
        epoch_train_acc = train_acc / total_train
        epoch_val_loss = val_loss / total_val
        epoch_val_acc = val_acc / total_val
        trigger_recall = trigger_tp / max(1, trigger_gt)
        cur_lr = optimizer.param_groups[0]['lr']

        print(f"Results: Train Loss {epoch_train_loss:.4f} Acc {epoch_train_acc:.3f} | "
              f"Val Loss {epoch_val_loss:.4f} Acc {epoch_val_acc:.3f} | "
              f"Trigger Recall {trigger_recall:.3f} (Best: {best_recall:.3f})")

        if args.wandb:
            wandb.log({
                "train/loss": epoch_train_loss,
                "train/acc": epoch_train_acc,
                "val/loss": epoch_val_loss,
                "val/acc": epoch_val_acc,
                "val/trigger_recall": trigger_recall,
                "lr": cur_lr,
                "epoch": epoch
            })

        if trigger_recall >= best_recall:
                    best_recall = trigger_recall
                    
                    # --- 修改开始 ---
                    # 原代码: 'model': model.state_dict() 会保存整个网络（含冻结的backbone）
                    # 新代码: 'head': model.head.state_dict() 只保存分类头的参数
                    save_dict = {
                        'head': model.head.state_dict(),  # 只保存 head
                        'epoch': epoch,
                        'args': args,
                        'best_recall': best_recall
                    }
                    torch.save(save_dict, os.path.join(out_dir, 'best_trigger_model.pth'))
                    # --- 修改结束 ---
                    
                    print(">>> Best Trigger Recall Model Saved (Head Only)!")

    if args.wandb:
        wandb.finish()

if __name__ == '__main__':
    main()