import os
import argparse
import csv
import math
import datetime
import re
from pathlib import Path
import pandas as pd
import time
from itertools import islice, cycle

import torch
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
from torch.utils.data import DataLoader, RandomSampler
from torch.utils.tensorboard import SummaryWriter
import torchvision.transforms as T
import timm.optim.optim_factory as optim_factory
from tqdm import tqdm

# === 1. Import wandb ===
import wandb

from utils import (
    MyCarotidDataset,
    set_randomness,
    rot6d_to_matrix,
    geodesic_loss_squared,
)

from Mymodels import (
    models_mae, 
    SwinTransformer, 
    PoseViTCore, 
    GuidanceHead, 
    SwinTransformerV2, 
    TransformerPoseHead, 
    LitePoseHead,
    PoseHead2,
    PoseHead4,
    DeepSet2Image
)

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


def rename_key(k):
    k = k.replace('module.', '')
    if k.startswith('encoder.'):
        k = k[len('encoder.'):]
    k = re.sub(r'layers\.(\d+)\.', r'layers\1.', k)
    k = k.replace('.mlp.fc1.', '.mlp.linear1.').replace('.mlp.fc2.', '.mlp.linear2.')
    return k


_REPLACEMENTS = [
    (r'^module\.', ''),               
    (r'^encoder\.', ''),               
    (r'\.mlp\.fc1\.', '.mlp.linear1.'), 
    (r'\.mlp\.fc2\.', '.mlp.linear2.'),
    (r'layers\.(\d+)\.', r'layers\1.'),
]

def normalize_key(k: str) -> str:
    for pat, rep in _REPLACEMENTS:
        k = re.sub(pat, rep, k)
    return k

def build_swinmodel(args, device):
    model = SwinTransformer(
        in_chans=3, embed_dim=128, patch_size=(2, 2), window_size=(8,8),
        depths=(2, 2, 18, 2), num_heads=(4, 8, 16, 32),
        mlp_ratio=4., qkv_bias=True, spatial_dims=2, use_v2=True
    ).to(device)

    ckpt_path = Path(args.swincheckpoint)
    if not ckpt_path.is_file():
        print(f"can not find {ckpt_path},random init")
        return model

    raw = torch.load(ckpt_path, map_location='cpu')
    raw = raw.get('model', raw)  

    ckpt = {normalize_key(k): v for k, v in raw.items()}

    model_state = model.state_dict()
    loadable = {k: v for k, v in ckpt.items()
                if k in model_state and v.shape == model_state[k].shape}

    msg = model.load_state_dict(loadable, strict=False)


    loaded_set   = set(loadable.keys())
    missing_set  = set(msg.missing_keys)
    unused_set   = set(msg.unexpected_keys)

    print(f"load {len(loaded_set)} / {len(model_state)} tensors"
          f" | missing {len(missing_set)} | unused {len(unused_set)}")

    csv_path = ckpt_path.with_suffix('.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['status', 'key'])
        for k in sorted(loaded_set):   writer.writerow(['loaded',   k])
        for k in sorted(missing_set):  writer.writerow(['missing',  k])
        for k in sorted(unused_set):   writer.writerow(['unused',   k])
    print(f" load result saved to {csv_path}")

    return model


def build_swinmodel_origin(args, device):
    swin = SwinTransformerV2(
        img_size=256, patch_size=2, in_chans=3,
        num_classes=0,
        embed_dim=48, depths=(2, 2, 2, 2),
        num_heads=(3, 6, 12, 24),
        window_size=8, drop_path_rate=0.3
    ).to(device)

    ckpt_p = Path(args.swinocheckpoint)
    if not ckpt_p.is_file():
        print("checkpoint not found, use random init.")
        return swin

    raw_state = torch.load(ckpt_p, map_location="cpu")
    raw_state = raw_state.get("model", raw_state)
    enc_state = {k: v for k, v in raw_state.items() if not k.startswith("head.")}
    swin.load_state_dict(enc_state, strict=False)
    return swin



def build_maemodel(args, device):
    assert hasattr(models_mae, args.maemodel), \
        f"models_mae.py cannot find '{args.maemodel}'"

    model = getattr(models_mae, args.maemodel)(norm_pix_loss=False).to(device).eval()

    ckpt = torch.load(args.maecheckpoint,
                      map_location='cpu',
                      weights_only=False)          

    state_dict = ckpt["model"] if "model" in ckpt else ckpt
    missing, unexpected = model.load_state_dict(state_dict, strict=False)

    model_keys = set(model.state_dict().keys())
    ckpt_keys = set(state_dict.keys())

    loaded_set = (ckpt_keys & model_keys) - set(unexpected) - set(missing)
    missing_set = set(missing)
    unused_set = set(unexpected)

    ckpt_path = Path(args.maecheckpoint)
    csv_path = ckpt_path.with_suffix('.csv')

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["status", "key"])
        for k in sorted(loaded_set): writer.writerow(["loaded", k])
        for k in sorted(missing_set): writer.writerow(["missing", k])
        for k in sorted(unused_set): writer.writerow(["unused", k])

    print(f"load {len(loaded_set)} / {len(model_keys)} tensors"
          f" | missing {len(missing_set)} | unused {len(unused_set)}")
    print(f"load result saved to {csv_path}")
    return model



def build_dataset_transform(backbone, size):
    if backbone == 'swin':
        return T.Compose([T.ToTensor(), T.Resize((size, size))])
    elif backbone == 'swino':
        return T.Compose([
            T.ToTensor(),
            T.Resize(size, interpolation=T.InterpolationMode.BICUBIC),
        ])
    elif backbone == 'transformer':
        return T.Compose([
            T.ToTensor(),
            T.Resize((size, size), interpolation=T.InterpolationMode.BICUBIC, antialias=True),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
    else:
        raise ValueError(backbone)


def loss_weighted(pred, gt, beta=0.1, alpha=1.0):
    w = torch.exp(-alpha * gt)                    
    l = F.smooth_l1_loss(pred, gt, beta=beta, reduction='none')
    # print(gt, w)
    w = w.clamp(max=2)
    return (w * l).sum() / (w.sum() + 1e-8)


def forward_batch(img1, dT1e, dR1e, e1e, backbone, pick_indices, model, posehead, device):
    img1, dT1e, dR1e, e1e = [x.to(device, non_blocking=True) for x in (img1, dT1e, dR1e, e1e)]
    
    if backbone == 'swin': 
        embedding = model.forward_features(img1, feat_type='pyramid')[-1]
        embedding = [embedding]
    elif backbone == 'swino':
        embedding = model.forward_features2(img1)
    elif backbone == 'transformer': 
        x, feats, embedding = model.forward_encoder2(img1, pick_indices=pick_indices)
    else:
        raise ValueError(backbone)

    pose = posehead(feats[-1], x)
    # 原始逻辑是训练角度 e1e
    loss = loss_weighted(pose, e1e[:, 0].unsqueeze(1), beta=0.5)
    return loss, pose

def get_args():
    parser = argparse.ArgumentParser('carotid copilot JIUAN', add_help=False)

    parser.add_argument('--epochs', default=100, type=int)
    parser.add_argument('--batch_size', default=16, type=int)
    parser.add_argument('--accum_iter', default=1, type=int)
    parser.add_argument('--lr', default=5e-4, type=float)
    parser.add_argument('--blr', default=5e-4, type=float)
    parser.add_argument('--weight_decay', default=1e-3, type=float)
    parser.add_argument('--warmup_epochs', default=2, type=int)

    parser.add_argument('--backbone', default='transformer', choices=['swin', 'swino', 'transformer'])
    
    parser.add_argument('--freeze_encoder', type = int, default = 1)
    parser.add_argument("--freeze_backbone_epochs", type=int, default=1)

    parser.add_argument('--swincheckpoint', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/pretrain_weights/simmim/swin_0919_fintune.pth')
    parser.add_argument('--swinocheckpoint', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/EchoWorld-main/weights/swinv2_base_patch4_window12_192_22k.pth')
    parser.add_argument('--maecheckpoint', default='pretrain_weights/mae/carotid_base_checkpoint-500_0721.pth')
    parser.add_argument('--maemodel', default='mae_vit_base_patch16')
    parser.add_argument('--transformer_layers', default=[9])

    parser.add_argument('--data_rootdir', default='/home/mingcong/project/us_carotid/pose_pilot/raw_data/data_0704MC_carotid_copilot_angle_split/')                    
    parser.add_argument('--out_dir', default='./runs/angle')
    parser.add_argument('--num_workers', default=8, type=int)
    parser.add_argument('--seed', default=42, type=int)

    parser.add_argument('--tqdm', type=int, default=1, choices=[0, 1], help='1=show progress bar, 0=quiet')
    parser.add_argument('--stepsperepoch', type=int, default=None, help='None ⇒ iterate whole loader')

    # === 2. WandB Arguments ===
    parser.add_argument('--wandb', type=int, default=1, help='Enable wandb logging (1=True, 0=False)')
    parser.add_argument('--wandb_project', default='carotid_copilot', help='WandB project name')
    parser.add_argument('--wandb_run_name', default=None, help='WandB run name (optional)')
    parser.add_argument('--wandb_entity', default=None, help='WandB entity/username (optional)')
    
    return parser.parse_args()


def main():
    args = get_args()
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    cudnn.benchmark = True
    set_randomness(args.seed)

    # === 3. Initialize WandB ===
    if args.wandb:
        run_name = args.wandb_run_name if args.wandb_run_name else datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        wandb.init(
            project=args.wandb_project,
            entity=args.wandb_entity,
            name=run_name,
            config=args,
            reinit=True
        )

    if args.out_dir is None:
        args.out_dir = f'runs/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
    out_dir = args.out_dir + '/' + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    weights_dir = os.path.join(out_dir, 'weights')
    os.makedirs(weights_dir, exist_ok=True)

    log_writer = SummaryWriter(out_dir)

    if args.backbone == 'swin':
        model_backbone = build_swinmodel(args, device)
        img_size = 256
    elif args.backbone == 'swino':
        model_backbone = build_swinmodel_origin(args, device)
        img_size = 256
    else:
        model_backbone = build_maemodel(args, device)
        img_size = 224

    posehead = PoseHead4(
        in_dim = 768,
        out_dim = 1,
        ).to(device)

    # 如果需要记录模型结构到wandb
    # if args.wandb:
    #     wandb.watch(posehead, log='all', log_freq=100)

    transform = build_dataset_transform(args.backbone, img_size)
    train_set = MyCarotidDataset(
        root_dir=args.data_rootdir, 
        split='train', 
        image_interval=1,
        transform=transform, 
        normalize = False, 
        norm_mode="minmax_pad", 
        pad_ratio=0.05,
        aug_enable=False,
        shift_mode = "none",
        gt_policy = "pose"
        )
    
    val_set = MyCarotidDataset(
        root_dir=args.data_rootdir, 
        split='val', 
        image_interval=1,
        transform=transform, 
        normalize = False, 
        norm_mode="minmax_pad", 
        pad_ratio=0.05,
        aug_enable=False,
        shift_mode = "none",
        gt_policy = "pose"
        )

    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers, pin_memory=True)
    val_loader = DataLoader(val_set, batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers, pin_memory=True)

    eff_bs = args.batch_size * args.accum_iter
    if args.lr is None:
        args.lr = args.blr * eff_bs / 256

    backbone_params = [p for p in model_backbone.parameters() if p.requires_grad]
    head_params = list(posehead.parameters())
    param_groups = [
        dict(params=backbone_params, lr=args.lr),
        dict(params=head_params, lr=args.lr),
    ]

    optim = optim_factory.create_optimizer_v2(
        param_groups,
        opt='adamw',
        weight_decay=args.weight_decay,
        betas=(0.9, 0.95),
        filter_bias_and_bn=True
    )

    iters_per_epoch = args.stepsperepoch or len(train_loader)
    warmup_iters = args.warmup_epochs * iters_per_epoch
    total_iters = args.epochs * iters_per_epoch

    def lr_lambda(it):
        if it < warmup_iters:
            return it / warmup_iters
        progress = (it - warmup_iters) / max(1, total_iters - warmup_iters)
        cosine = 0.5 * (1 + math.cos(math.pi * progress))
        return 0.1 + 0.9 * cosine

    scheduler = torch.optim.lr_scheduler.LambdaLR(optim, lr_lambda)

    scaler = torch.cuda.amp.GradScaler(enabled=torch.cuda.is_available())
    best_val = float('inf')
    show_bar = bool(args.tqdm)

    print(f"Start training for {args.epochs} epochs")
    for epoch in range(args.epochs):

        freeze_backbone = (args.freeze_encoder == 0 and (epoch < args.freeze_backbone_epochs)) or  args.freeze_encoder

        if freeze_backbone:
            for p in model_backbone.parameters():
                p.requires_grad = False
            model_backbone.eval()
        else:
            for p in model_backbone.parameters():
                p.requires_grad = True
            model_backbone.train()

        posehead.train()
        loss_m = 0.
        Terr_real_sum_abs = 0.0 
        Terr_real_count   = 0   
        
        train_bar = tqdm(train_loader, total=iters_per_epoch, desc=f"Epoch {epoch}/{args.epochs}", ncols=100, disable=not show_bar)
        optim.zero_grad(set_to_none=True)
        
        for step, (img1, _T1, _Te, _R1, _Re, _dT1e, _dR1e, e1e) in enumerate(train_bar,1):
            with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):
                loss, pred = forward_batch(img1, _dT1e, _dR1e, e1e, args.backbone, args.transformer_layers, model_backbone, posehead, device)
                loss = loss / args.accum_iter

                # 真实误差计算 (基于角度)
                err_real = (pred.squeeze(-1).cpu() - e1e[:, 0]).abs() * 90
                Terr_real_sum_abs += err_real.sum().item()
                Terr_real_count += err_real.numel()
            
            scaler.scale(loss).backward()

            if (step + 1) % args.accum_iter == 0:
                scaler.unscale_(optim)
                torch.nn.utils.clip_grad_norm_( (p for g in optim.param_groups for p in g['params']),
                                max_norm=0.5, foreach=True, error_if_nonfinite=False )
                
                scaler.step(optim)
                scaler.update()
                optim.zero_grad(set_to_none=True)
                scheduler.step()

                loss_m += loss.item() * args.accum_iter
                if args.stepsperepoch and step + 1 >= args.stepsperepoch:
                    break

        train_total = loss_m / (step + 1)
        train_loss_real = Terr_real_sum_abs / max(1, Terr_real_count)
        current_lr = scheduler.get_last_lr()[0]

        # === Validation ===
        model_backbone.eval()
        posehead.eval()
        vl = 0.
        Verr_real_sum_abs = 0
        Verr_real_count = 0

        with torch.no_grad():
            val_bar = tqdm(val_loader, ncols=100, disable=not show_bar, desc="val")
            for img1, _T1, _Te, _R1, _Re, _dT1e, _dR1e, e1e in val_bar:
                loss, pred = forward_batch(img1, _dT1e, _dR1e, e1e, args.backbone, args.transformer_layers, model_backbone, posehead, device)
                pred = pred.clamp(0, 1)
                
                vl += loss.item()

                err_real = (pred.squeeze(-1).cpu() - e1e[:, 0]).abs() * 90
                Verr_real_sum_abs += err_real.sum().item()
                Verr_real_count += err_real.numel()

        val_total = vl / len(val_loader)
        val_loss_real = Verr_real_sum_abs / max(1, Verr_real_count)

        print(f"E{epoch:03d} | TrainLoss {train_total:.4f} | ValLoss {val_total:.4f} | TrainReal(deg) {train_loss_real:.4f} | ValReal(deg) {val_loss_real:.4f} | LR {current_lr:.2e}")

        # === 4. Logging ===
        log_writer.add_scalar('train loss', train_total, epoch)
        log_writer.add_scalar('val loss', val_total, epoch)
        log_writer.add_scalar('val real error', val_loss_real, epoch)
        
        if args.wandb:
            wandb.log({
                'train/loss': train_total,
                'train/real_error_deg': train_loss_real,
                'val/loss': val_total,
                'val/real_error_deg': val_loss_real,
                'lr': current_lr,
                'epoch': epoch
            })

        with open(os.path.join(out_dir, 'train_log.csv'), 'a', newline='') as f:
            csv.writer(f).writerow([epoch, train_total, val_total, train_loss_real, val_loss_real])

        # Save Best
        if val_total < best_val:
            best_val = val_total
            torch.save({
                'backbone': model_backbone.state_dict(),
                'posehead': posehead.state_dict(),
                'epoch': epoch,
                'args': args
            }, os.path.join(out_dir, 'weights/best.pth'))
    
    # === 5. Finish WandB ===
    if args.wandb:
        wandb.finish()
    
    print('Training completed.')

if __name__ == '__main__':
    main()