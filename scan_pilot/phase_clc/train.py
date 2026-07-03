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

# 引入你的模型定义 (假设 models_mae 在当前目录或 pythonpath下)
# 如果没有 models_mae，可以使用 timm 或 transformers 替代

from models import models_mae

class FocalLoss(nn.Module):
    def __init__(self, alpha=None, gamma=2.0, reduction='mean'):
        super(FocalLoss, self).__init__()
        # alpha 对应 class_weights
        self.alpha = alpha 
        self.gamma = gamma
        self.reduction = reduction

    def forward(self, inputs, targets):
        # inputs: [N, C], targets: [N]
        ce_loss = F.cross_entropy(inputs, targets, reduction='none', weight=self.alpha)
        pt = torch.exp(-ce_loss) # pt是模型对自己预测正确的概率
        
        # 核心公式：(1 - pt)^gamma * ce_loss
        # 越难分的样本，pt越低，(1-pt)越大，Loss权重越大
        focal_loss = ((1 - pt) ** self.gamma) * ce_loss

        if self.reduction == 'mean':
            return focal_loss.mean()
        elif self.reduction == 'sum':
            return focal_loss.sum()
        else:
            return focal_loss
# ==============================================================================
# 1. 定义数据加载器 (CarotidDataset)
# ==============================================================================
class CarotidDataset(Dataset):
    def __init__(self, root_dir, split='train', transform=None):
        self.root_dir = os.path.join(root_dir, split)
        self.transform = transform
        self.samples = [] 
        
        # 统计原始标签分布，方便debug
        self.raw_counts = {} 
        
        self._load_data()

    def _load_data(self):
        if not os.path.exists(self.root_dir):
            print(f"Warning: {self.root_dir} does not exist.")
            return
        
        seq_dirs = [d for d in os.listdir(self.root_dir) if os.path.isdir(os.path.join(self.root_dir, d))]
        
        # 允许的原始标签是 1, 2, 3
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
                        
                        # 1. 过滤非法标签
                        if raw_label not in valid_raw_labels:
                            continue 

                        # 2. 【关键修改】映射标签：1->0, 2->1, 3->2
                        mapped_label = raw_label - 1
                        
                        img_path = os.path.join(seq_path, 'images', item['name'])
                        if os.path.exists(img_path):
                            self.samples.append((img_path, mapped_label))
                            
                            # 记录一下原始标签数量，确认读对了
                            self.raw_counts[raw_label] = self.raw_counts.get(raw_label, 0) + 1
                            
            except Exception as e:
                pass
        
        print(f"Dataset Loaded '{os.path.basename(self.root_dir)}': Found raw labels {self.raw_counts}")
        print(f"  -> Mapped to 0, 1, 2 internally.")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img_path, label = self.samples[idx]
        image = Image.open(img_path).convert('RGB')
        if self.transform:
            image = self.transform(image)
        return image, label

    def get_class_counts(self):
        # 这里的 count 是映射后的 (0, 1, 2)
        counts = {0: 0, 1: 0, 2: 0}
        for _, label in self.samples:
            if label in counts:
                counts[label] += 1
        return counts
    
# ==============================================================================
# 2. 定义模型 ScanHead
# ==============================================================================
class ScanHead(nn.Module):
    def __init__(self, in_dim=768, hidden_dim=256, out_dim=3, 
                 dropout=0.2, use_spatial_attn=True, use_grid=True, **kwargs):
        super().__init__()
        # 简化版参数接收
        self.use_spatial_attn = use_spatial_attn
        self.use_grid = use_grid
        
        self.cls_norm = nn.LayerNorm(in_dim)
        self.patch_norm = nn.LayerNorm(in_dim)
        self.cls_proj = nn.Linear(in_dim, hidden_dim)
        self.patch_proj = nn.Linear(in_dim, hidden_dim)

        if use_spatial_attn:
            self.attn_ln = nn.LayerNorm(hidden_dim)
            self.attn = nn.MultiheadAttention(hidden_dim, num_heads=4, dropout=dropout, batch_first=True)

        if use_grid:
            self.coord_proj = nn.Conv2d(hidden_dim + 2, hidden_dim, 1) # +2 for coord
            self.dw3 = nn.Conv2d(hidden_dim, hidden_dim, 3, 1, 1, groups=hidden_dim)
            self.pw1 = nn.Conv2d(hidden_dim, hidden_dim, 1)
            self.spatial_act = nn.GELU()
            self.register_buffer('grid_xy', self._make_grid(14, 14), persistent=False)

        self.pool_reduce = nn.Linear(2 * hidden_dim, hidden_dim)
        self.fuse = nn.Linear(2 * hidden_dim, hidden_dim)
        
        # Classification Head (No Sigmoid at end for CrossEntropy)
        self.head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, out_dim)
        )
        self.apply(self._init_weights)

    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            nn.init.trunc_normal_(m.weight, std=0.02)
            if m.bias is not None: nn.init.zeros_(m.bias)
        elif isinstance(m, nn.LayerNorm):
            nn.init.ones_(m.weight)
            nn.init.zeros_(m.bias)
        elif isinstance(m, nn.Conv2d):
            nn.init.kaiming_normal_(m.weight, nonlinearity='linear')

    def _make_grid(self, H, W):
        y, x = torch.meshgrid(torch.linspace(-1, 1, H), torch.linspace(-1, 1, W), indexing='ij')
        return torch.stack([x, y], dim=0).unsqueeze(0)

    def forward(self, x_patches, x_cls=None):
        # x_patches: [B, 196, 768], x_cls: [B, 1, 768]
        cls_z = self.cls_proj(self.cls_norm(x_cls.squeeze(1))) if x_cls is not None else None
        p = self.patch_proj(self.patch_norm(x_patches))

        if self.use_spatial_attn:
            p = p + self.attn(self.attn_ln(p), self.attn_ln(p), self.attn_ln(p))[0]

        # Reshape to grid
        B, N, C = p.shape
        H = int(math.isqrt(N))
        fmap = p.view(B, H, H, C).permute(0, 3, 1, 2) # [B, C, H, H]

        if self.use_grid:
            grid = self.grid_xy.to(fmap.device).expand(B, -1, H, H)
            fmap = torch.cat([fmap, grid], dim=1)
            fmap = self.coord_proj(fmap)
            fmap = self.dw3(fmap)
            fmap = self.spatial_act(self.pw1(fmap))

        gap = F.adaptive_avg_pool2d(fmap, 1).flatten(1)
        gmp = F.adaptive_max_pool2d(fmap, 1).flatten(1)
        patch_z = self.pool_reduce(torch.cat([gap, gmp], dim=1))

        z = patch_z if cls_z is None else self.fuse(torch.cat([cls_z, patch_z], dim=1))
        return self.head(z)

class SimpleClassHead(nn.Module):
    def __init__(self, in_dim=768, out_dim=3, dropout=0.5):
        super(SimpleClassHead, self).__init__()
        # 这里的输入是 [Batch, 768]，直接进全连接
        self.bn = nn.BatchNorm1d(in_dim)
        self.dropout = nn.Dropout(dropout)
        self.fc = nn.Linear(in_dim, out_dim)
        
        # 初始化权重
        nn.init.normal_(self.fc.weight, std=0.01)
        nn.init.constant_(self.fc.bias, 0)

    def forward(self, x):
        # x shape: [Batch, 768]
        x = self.bn(x)
        x = self.dropout(x)
        x = self.fc(x)
        return x
    
# ==============================================================================
# 3. 组合模型 (VirMAE Backbone + ScanHead)
# ==============================================================================
class CarotidClassifier(nn.Module):
    # def __init__(self, args, backbone, head):
    #     super().__init__()
    #     self.backbone = backbone
    #     self.head = head
    #     self.layer_idx = args.transformer_layers[0] # e.g. 9

    # def forward(self, x):
    #     # MAE Forward
    #     # 假设 models_mae 的 forward_encoder 返回 (latent, mask, ids_restore)
    #     # 或者我们需要获取中间层特征
    #     # 这里适配常见的 MAE 接口，如果你的 models_mae 不同请调整
        
    #     # 调用 backbone 提取特征
    #     # 通常 mae.forward_encoder 会返回 cls_token 和 patch_tokens
    #     latent, _, _ = self.backbone.forward_encoder(x, mask_ratio=0.0)
        
    #     # latent shape: [B, 197, 768] (1 cls + 196 patches)
    #     cls_token = latent[:, 0:1, :]
    #     patch_tokens = latent[:, 1:, :]
        
    #     logits = self.head(patch_tokens, cls_token)
    #     return logits

    def __init__(self, backbone_name='mae_vit_base_patch16', num_classes=3):
            super().__init__()
            # ... Backbone加载代码保持不变 ...
            self.backbone = getattr(models_mae, backbone_name)(norm_pix_loss=False)
            
            # 【修改点 1】：换用新的 SimpleClassHead
            # 原来是: self.head = PoseHead4(...) 
            self.head = SimpleClassHead(in_dim=768, out_dim=num_classes, dropout=0.5)

    def forward(self, x):
        # 1. 提取特征
        latent, mask, ids_restore = self.backbone.forward_encoder(x, mask_ratio=0)
        
        # 【修改点 2】：做 GAP (平均池化) 得到 2D 特征 [Batch, 768]
        # latent shape is [Batch, 197, 768] -> mean -> [Batch, 768]
        feature = latent.mean(dim=1) 
        
        # 3. 传入新的 Head
        out = self.head(feature)
        return out

def build_maemodel(args, device):
    print(f"Loading MAE Backbone: {args.maemodel} ...")
    model = getattr(models_mae, args.maemodel)(norm_pix_loss=False).to(device)
    
    if args.maecheckpoint and os.path.isfile(args.maecheckpoint):
        checkpoint = torch.load(args.maecheckpoint, map_location='cpu')
        msg = model.load_state_dict(checkpoint['model'], strict=False)
        print(f"MAE Pretrained Loaded: {msg}")
    
    return model

# ==============================================================================
# 4. 辅助函数
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
    # torch.backends.cudnn.deterministic = True # 可能会变慢

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
    parser.add_argument('--freeze_encoder', type=int, default=1, help="0=finetune all, 1=freeze backbone")
    parser.add_argument('--transformer_layers', default=[11], nargs='+', type=int) # 取最后一层

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
# 5. 主函数
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
    
    # WandB Init
    if args.wandb:
        wandb.init(project=args.project, name=args.run_name, config=vars(args))

    print(f"Training on {device}, Output: {out_dir}")

    # --- Dataset ---
    # MAE default size 224
    transform_train = T.Compose([
        T.Resize((224, 224)),
        T.RandomHorizontalFlip(), # 增强泛化
        T.ColorJitter(brightness=0.2, contrast=0.2), # 模拟超声增益变化
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

    # --- Class Weights (Handle Imbalance) ---
    counts = train_set.get_class_counts()
    print(f"Class Distribution: {counts}")
    # Weight = Total / (Num_Classes * Count)
    total_samples = sum(counts.values())
    weights = [total_samples / (3 * max(1, counts[i])) for i in range(3)]
    # 稍微增加 Trigger (Label 1) 的权重，因为它最关键且最难
    weights[1] *= 1.2 
    class_weights = torch.FloatTensor(weights).to(device)
    print(f"Using Class Weights: {weights}")

    # --- Model ---
    backbone = build_maemodel(args, device)
    head = ScanHead(in_dim=768, out_dim=3).to(device) # out_dim=3 for classification
    model = CarotidClassifier(args, backbone, head).to(device)

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
    #criterion = nn.CrossEntropyLoss(weight=class_weights,label_smoothing=0.2)
    criterion = FocalLoss(alpha=class_weights, gamma=2.0)

    # --- Scheduler ---
    def lr_lambda(epoch):
        if epoch < args.warmup_epochs:
            return (epoch + 1) / args.warmup_epochs
        # Cosine Decay
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
            
            # Metrics
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
        
        # Trigger Metrics (Label 1)
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
                
                # Recall for Trigger
                is_trigger = (labels == 1)
                trigger_gt += is_trigger.sum().item()
                trigger_tp += (preds[is_trigger] == 1).sum().item()

        # Stats
        epoch_train_loss = train_loss / total_train
        epoch_train_acc = train_acc / total_train
        epoch_val_loss = val_loss / total_val
        epoch_val_acc = val_acc / total_val
        trigger_recall = trigger_tp / max(1, trigger_gt)
        cur_lr = optimizer.param_groups[0]['lr']

        print(f"Results: Train Loss {epoch_train_loss:.4f} Acc {epoch_train_acc:.3f} | "
              f"Val Loss {epoch_val_loss:.4f} Acc {epoch_val_acc:.3f} | "
              f"Trigger Recall {trigger_recall:.3f} (Best: {best_recall:.3f})")

        # WandB Log
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

        # Save Best Model (Strategy: Save based on Trigger Recall primarily)
        # 如果你更在乎整体准确率，可以改判据为 epoch_val_acc
        if trigger_recall >= best_recall:
            best_recall = trigger_recall
            torch.save({
                'model': model.state_dict(),
                'epoch': epoch,
                'args': args
            }, os.path.join(out_dir, 'best_checkpoint.pth'))
            print(">>> Best Model Saved!")

    if args.wandb:
        wandb.finish()

if __name__ == '__main__':
    main()