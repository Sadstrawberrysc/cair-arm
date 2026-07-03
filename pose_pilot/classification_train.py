import os
import argparse
import csv
import math
import datetime
import re
from pathlib import Path

import torch
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
import torchvision.transforms as T
import timm.optim.optim_factory as optim_factory
import timm

from utils import (
    ClsFolderDataset,
    set_randomness,
)

from Mymodels import (
    models_mae,
    SwinTransformer,
    SwinTransformerV2,
    PoseClsHead,
    PoseClsHead2,
)
from tqdm import tqdm

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
        in_chans=3, embed_dim=48, patch_size=(2, 2), window_size=(7, 7),
        depths=(2, 2, 2, 2), num_heads=(3, 6, 12, 24),
        mlp_ratio=4., qkv_bias=True, spatial_dims=2, use_v2=True
    ).to(device)

    ckpt_path = Path(args.swincheckpoint)
    if not ckpt_path.is_file():
        print(f"[swin] checkpoint not found, use random init.")
        return model

    raw = torch.load(ckpt_path, map_location='cpu')
    raw = raw.get('model', raw)
    ckpt = {normalize_key(k): v for k, v in raw.items()}

    model_state = model.state_dict()
    loadable = {k: v for k, v in ckpt.items() if k in model_state and v.shape == model_state[k].shape}
    msg = model.load_state_dict(loadable, strict=False)

    print(f"[swin] load {len(loadable)} tensors | missing {len(msg.missing_keys)} | unused {len(msg.unexpected_keys)}")
    return model

def build_swinmodel_origin(args, device):
    swin = SwinTransformerV2(
        img_size=256, patch_size=2, in_chans=3, num_classes=0,
        embed_dim=48, depths=(2, 2, 2, 2),
        num_heads=(3, 6, 12, 24), window_size=8, drop_path_rate=0.3
    ).to(device)

    ckpt_p = Path(args.swinocheckpoint)
    if not ckpt_p.is_file():
        print("[swino] checkpoint not found, use random init.")
        return swin

    raw_state = torch.load(ckpt_p, map_location="cpu")
    raw_state = raw_state.get("model", raw_state)
    enc_state = {k: v for k, v in raw_state.items() if not k.startswith("head.")}
    swin.load_state_dict(enc_state, strict=False)
    return swin

def build_maemodel(args, device):
    assert hasattr(models_mae, args.maemodel), f"models_mae.py cannot find '{args.maemodel}'"
    model = getattr(models_mae, args.maemodel)(norm_pix_loss=False).to(device).eval()

    ckpt = torch.load(args.maecheckpoint, map_location='cpu', weights_only=False)
    state_dict = ckpt["model"] if "model" in ckpt else ckpt
    model.load_state_dict(state_dict, strict=False)
    print("mae checkpoint loaded (non-strict).")
    return model

def build_dataset_transform(backbone, size):
    if backbone == 'swin':
        return T.Compose([
            T.Resize((size, size)),              
            T.ToTensor(),                        # 转 tensor
        ])
    elif backbone == 'swino':
        return T.Compose([
            T.Resize(size, interpolation=T.InterpolationMode.BICUBIC),
            T.ToTensor(),
        ])
    elif backbone == 'transformer':
        return T.Compose([
            T.Resize(size, interpolation=T.InterpolationMode.BICUBIC),
            T.CenterCrop(size),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
    else:
        raise ValueError(backbone)

def extract_feats(backbone, model, img, pick_indices):
    if backbone == 'swin':
        feats = model.forward_features(img, feat_type='pyramid')  
        return [feats[-1]]                                        
    elif backbone == 'swino':
        feat = model.forward_features2(img)                       
        return [feat]
    elif backbone == 'transformer':
        x, feats, embedding = model.forward_encoder2(img, mask_ratio=0.0, pick_indices=pick_indices)
        return x, feats, embedding                                             
    else:
        raise ValueError(backbone)

def forward_batch(img, labels, backbone, model, head, device, criterion, pick_indices):
    img = img.to(device, non_blocking=True)
    labels = labels.to(device, non_blocking=True)

    x, feats, embedding = extract_feats(backbone, model, img, pick_indices)    
    logits = head(x)                                          
    loss = criterion(logits, labels)

    with torch.no_grad():
        preds = torch.argmax(logits, dim=1)
        acc = (preds == labels).float().mean().item()
    return loss, acc, logits


def get_args():
    parser = argparse.ArgumentParser('classification trainer', add_help=False)
    parser.add_argument('--epochs', default=50, type=int)
    parser.add_argument('--batch_size', default=16, type=int)
    parser.add_argument('--accum_iter', default=1, type=int)
    parser.add_argument('--lr', default=5e-4, type=float)
    parser.add_argument('--blr', default=5e-4, type=float)
    parser.add_argument('--weight_decay', default=1e-3, type=float)
    parser.add_argument('--warmup_epochs', default=5, type=int)

    parser.add_argument('--backbone', default='transformer', choices=['swin', 'swino', 'transformer'])

    parser.add_argument('--freeze_encoder', type = int, default = 1)
    parser.add_argument("--freeze_backbone_epochs", type=int, default=1)

    parser.add_argument('--swincheckpoint', default='path/to/swin.pt')
    parser.add_argument('--swinocheckpoint', default='path/to/swinv2.pth')
    parser.add_argument('--maecheckpoint', default='/home/mingcong/project/embodied/US_standerview/embodiedUS/carotidcopilot_handover/pretrain_weights/mae/carotid_base_checkpoint-500_0721.pth')
    parser.add_argument('--maemodel', default='mae_vit_base_patch16')
    parser.add_argument('--transformer_layers', nargs='+', type=int, default=[9], help='用于 transformer 的层索引')

    parser.add_argument('--data_rootdir', default='/home/mingcong/project/embodied/US_standerview/embodiedUS/carotidcopilot_handover/data/data_classification')
    parser.add_argument('--out_dir', default='./runs/classification')
    parser.add_argument('--num_workers', default=8, type=int)
    parser.add_argument('--seed', default=42, type=int)

    parser.add_argument('--tqdm', type=int, default=1, choices=[0, 1])
    parser.add_argument('--stepsperepoch', type=int, default=None)

    parser.add_argument('--class_names', nargs='+', default=['diagonally', 'long'])
    return parser.parse_args()

def main():
    args = get_args()
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    cudnn.benchmark = True
    set_randomness(args.seed)

    if args.out_dir is None:
        args.out_dir = f'runs/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
    out_dir = args.out_dir +'/'+ datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(os.path.join(out_dir, 'weights'), exist_ok=True)
    log_writer = SummaryWriter(out_dir)

    if args.backbone == 'swin':
        model_backbone = build_swinmodel(args, device)
        img_size = 256
        in_dim = 48 * 8
        num_levels = 1
    elif args.backbone == 'swino':
        model_backbone = build_swinmodel_origin(args, device)
        img_size = 256
        in_dim = 48 * 8
        num_levels = 1
    else:
        model_backbone = build_maemodel(args, device)
        img_size = 224
        in_dim = 768 
        num_levels = len(args.transformer_layers)

    # if args.freeze_encoder:
    #     for p in model_backbone.parameters():
    #         p.requires_grad = False

    class_names = args.class_names
    num_classes = len(class_names)
    head = PoseClsHead2(
        in_dim=in_dim,
        num_classes=num_classes, 
        binary_mode="ce", 
    ).to(device)

    transform = build_dataset_transform(args.backbone, img_size)

    train_set = ClsFolderDataset(
        root_dir=args.data_rootdir,
        class_names=class_names,
        split='train',
        transform=transform,
        augment=True,
    )
    val_set = ClsFolderDataset(
        root_dir=args.data_rootdir,
        class_names=class_names,
        split='val',
        transform=transform,
        augment=False,
    )
    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True,
                              num_workers=args.num_workers, pin_memory=True)
    val_loader = DataLoader(val_set, batch_size=args.batch_size, shuffle=False,
                            num_workers=args.num_workers, pin_memory=True)

    eff_bs = args.batch_size * args.accum_iter
    lr = args.lr if args.lr is not None else args.blr * eff_bs / 256

    backbone_params = [p for p in model_backbone.parameters() if p.requires_grad]
    head_params = list(head.parameters())
    param_groups = [dict(params=backbone_params, lr=lr),
                    dict(params=head_params, lr=lr)]
    optim = timm.optim.optim_factory.create_optimizer_v2(
        param_groups, opt='adamw',
        weight_decay=args.weight_decay, betas=(0.9, 0.95),
        filter_bias_and_bn=True
    )

    iters_per_epoch = args.stepsperepoch or len(train_loader)
    warmup_iters = args.warmup_epochs * iters_per_epoch
    total_iters = max(1, args.epochs * iters_per_epoch)
    def lr_lambda(it):
        if it < warmup_iters:
            return it / max(1, warmup_iters)
        progress = (it - warmup_iters) / max(1, total_iters - warmup_iters)
        return 0.1 + 0.9 * 0.5 * (1 + math.cos(math.pi * progress))
    scheduler = torch.optim.lr_scheduler.LambdaLR(optim, lr_lambda)
    scaler = torch.cuda.amp.GradScaler(enabled=torch.cuda.is_available())
    criterion = torch.nn.CrossEntropyLoss()

    best_val_acc = 0.0
    show_bar = bool(args.tqdm)

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

        head.train()

        sum_loss, sum_acc, n_batches = 0.0, 0.0, 0
        train_bar = tqdm(train_loader, total=iters_per_epoch, desc=f"Epoch {epoch}/{args.epochs}", ncols=100)
        optim.zero_grad(set_to_none=True)

        for step, (img, labels) in enumerate(train_bar, 1):
            with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):
                loss, acc, _ = forward_batch(
                    img, labels, args.backbone, model_backbone, head, device, criterion, args.transformer_layers
                )
                loss = loss / args.accum_iter

            scaler.scale(loss).backward()

            if (step % args.accum_iter) == 0:
                scaler.unscale_(optim)
                torch.nn.utils.clip_grad_norm_(
                    (p for g in optim.param_groups for p in g['params']),
                    max_norm=0.5, foreach=True, error_if_nonfinite=False
                )
                scaler.step(optim)
                scaler.update()
                optim.zero_grad(set_to_none=True)
                scheduler.step()

            sum_loss += loss.item() * args.accum_iter
            sum_acc  += acc
            n_batches += 1
            train_bar.set_postfix(loss=sum_loss/n_batches, acc=sum_acc/n_batches)

            if args.stepsperepoch and step >= args.stepsperepoch:
                break

        train_loss = sum_loss / max(1, n_batches)
        train_acc  = sum_acc  / max(1, n_batches)

        # Val 
        model_backbone.eval()
        head.eval()
        vl_sum, va_sum, nv = 0.0, 0.0, 0
        with torch.no_grad():
            val_bar = tqdm(val_loader, ncols=100, disable=not show_bar, desc="val")
            for img, labels in val_bar:
                loss, acc, _ = forward_batch(
                    img, labels, args.backbone, model_backbone, head, device, criterion, args.transformer_layers
                )
                vl_sum += loss.item()
                va_sum += acc
                nv += 1
                val_bar.set_postfix(loss=vl_sum/nv, acc=va_sum/nv)

        val_loss = vl_sum / max(1, nv)
        val_acc  = va_sum / max(1, nv)

        print(f"E{epoch:03d} | Train {train_loss:.4f}/{train_acc:.4f} | "
              f"Val {val_loss:.4f}/{val_acc:.4f} | LR {scheduler.get_last_lr()[0]:.2e}")

        log_writer.add_scalar('train/loss', train_loss, epoch)
        log_writer.add_scalar('train/acc', train_acc, epoch)
        log_writer.add_scalar('val/loss', val_loss, epoch)
        log_writer.add_scalar('val/acc', val_acc, epoch)

        with open(os.path.join(out_dir, 'train_log.csv'), 'a', newline='') as f:
            csv.writer(f).writerow([epoch, train_loss, train_acc, val_loss, val_acc])

        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save({
                'backbone': model_backbone.state_dict(),
                'clshead':  head.state_dict(),
                'epoch': epoch,
                'class_names': class_names,
            }, os.path.join(out_dir, 'weights/best.pth'))
            print(f"Best acc={best_val_acc:.4f} saved.")

    print('Training completed. biu')

if __name__ == '__main__':
    main()
