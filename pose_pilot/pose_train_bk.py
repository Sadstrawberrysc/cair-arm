import os
import argparse
import csv
import math
import datetime
import re
from pathlib import Path
import pandas as pd

import torch
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
from torch.utils.data import DataLoader, RandomSampler
from torch.utils.tensorboard import SummaryWriter
import torchvision.transforms as T
import timm.optim.optim_factory as optim_factory

from utils import (
    MyCarotidDataset,
    set_randomness,
    rot6d_to_matrix,
    geodesic_loss_squared,
    build_inv_normalizer_from_dataset,
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
    DeepSet2Image,
    PoseHeadFlex,
    PoseHead3,
    PoseHead4,
    )
from itertools import islice, cycle
from tqdm import tqdm


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
        in_chans=3, embed_dim=128, patch_size=(2, 2), window_size=(8, 8),
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

    loaded_set = set(loadable.keys())
    missing_set = set(msg.missing_keys)
    unused_set = set(msg.unexpected_keys)

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
            # T.Resize(size, interpolation=T.InterpolationMode.BICUBIC),
            # T.CenterCrop(size),
            T.Resize((size, size), interpolation=T.InterpolationMode.BICUBIC, antialias=True),
            T.Normalize(mean = [0.485, 0.456, 0.406], std = [0.229, 0.224, 0.225]),
        ])
    else:
        raise ValueError(backbone)



def forward_batch(img1, dT1e, dR1e, e1e, backbone, pick_indices, model, posehead, device):
    img1, dT1e, dR1e, e1e = [x.to(device, non_blocking=True) for x in (img1, dT1e, dR1e, e1e)]
    x=None
    if backbone == 'swin':
        embedding = model.forward_features(img1, feat_type='pyramid')[-1]
        embedding = [embedding]
    elif backbone == 'swino':
        embedding = model.forward_features2(img1)
    elif backbone == 'transformer':
        x, feats, embedding = model.forward_encoder2(img1, mask_ratio=0.0, pick_indices=pick_indices)
    else:
        raise ValueError(backbone)

    pred = posehead(feats[-1],x)
    # print(pose.squeeze(-1).shape, dT1e[:,1].shape)
    loss_l1 = F.smooth_l1_loss(pred.squeeze(-1), dT1e[:,1], beta=0.2)

    range_ratio = 0.001
    bound_open = ((pred -1 ).relu()**2 + (-pred).relu()**2).mean()
    # loss = loss + range_ratio * bound_open
    return loss_l1, bound_open, pred



def get_args():
    parser = argparse.ArgumentParser('carotid copilot Jiuan Chen', add_help=False)
    parser.add_argument('--epochs', default=50, type=int)
    parser.add_argument('--batch_size', default=32, type=int)
    parser.add_argument('--accum_iter', default=1, type=int)
    parser.add_argument('--lr', default=8e-4, type=float)
    parser.add_argument('--blr', default=5e-4, type=float)
    parser.add_argument('--weight_decay', default=1e-3, type=float)
    parser.add_argument('--warmup_epochs', default=0, type=int)


    # backbone 设置
    parser.add_argument('--backbone', default='transformer', choices=['swin', 'swino', 'transformer'])
    parser.add_argument('--freeze_encoder', type = int, default = 1)
    parser.add_argument("--freeze_backbone_epochs", type=int, default=1)

    parser.add_argument('--swincheckpoint', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/EchoWorld-main/weights/swinckpHYF_140.pt')
    parser.add_argument('--swinocheckpoint', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/EchoWorld-main/weights/swinv2_base_patch4_window12_192_22k.pth')
    
    parser.add_argument('--maecheckpoint', default='/home/mingcong/project/embodied/US_standerview/embodiedUS/carotidcopilot_handover/pretrain_weights/mae/carotid_base_checkpoint-500_0721.pth')
    parser.add_argument('--maemodel', default='mae_vit_base_patch16')
    parser.add_argument('--transformer_layers', default=[9])

    parser.add_argument('--data_rootdir', default='/home/mingcong/project/embodied/US_standerview/embodiedUS/carotidcopilot_handover/carotid_copilot_new/datasets/pose_data')                    
    parser.add_argument('--out_dir', default='./runs/pose')
    parser.add_argument('--num_workers', default=8, type=int)
    parser.add_argument('--seed', default=42, type=int)

    parser.add_argument('--tqdm', type=int, default=1, choices=[0, 1], help='1=show progress bar, 0=quiet')
    parser.add_argument('--stepsperepoch', type=int, default=None, help='None ⇒ iterate whole loader')
    
    parser.add_argument('--normalize', type = int, default= 1)
    parser.add_argument('--norm_mode', type = str, default= "minmax_pad")
    parser.add_argument('--pad_ratio', type = float, default= 0.05)
    
    return parser.parse_args()


def main():
    args = get_args()
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    cudnn.benchmark = True
    set_randomness(args.seed)

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


    # if args.freeze_encoder:
    #     for p in model_backbone.parameters():
    #         p.requires_grad = False

    backbone_outlayersnum =  len(args.transformer_layers)
    posehead = PoseHead4(
        in_dim = 768,
        out_dim = 1,
        ).to(device)
    
    # deepset = DeepSet2Image().to(device)

    # 数据集
    transform = build_dataset_transform(args.backbone, img_size)
    train_set = MyCarotidDataset(
        root_dir=args.data_rootdir, 
        split='train', 
        image_interval=1,
        transform=transform, 
        normalize = True, 
        norm_mode=args.norm_mode, 
        pad_ratio=args.pad_ratio,

        flip_expand=True,
        p_crop = 0.0,
        crop_scale_rangeW=(0.99, 1.0),

        shift_mode='dynamic', 
        dynamic_k=0,
        gt_policy='keep_dt1f',          # 'keep_dt1f' , 'pixel_dis'
        ptx_only = 0,
        )
    
    val_set = MyCarotidDataset(
        root_dir=args.data_rootdir, 
        split='val', 
        image_interval=1,
        transform=transform, 
        normalize = True, 
        norm_mode=args.norm_mode, 
        pad_ratio=args.pad_ratio,
        aug_enable= False,     

        p_crop = 0,
        p_noise = 0,

        gt_policy='keep_dt1f', 
        shift_mode='pregen', 
        pregen_k = 0,
        ptx_only = 0,
        )
    
    inv_dT1e = build_inv_normalizer_from_dataset(key = "dT_1e", 
                                                 stats_path= args.data_rootdir, 
                                                 mode = args.norm_mode, 
                                                 pad_ratio = args.pad_ratio, 
                                                 feat_axis=-1, 
                                                 index=[1])


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
        # deepset.train()
        loss_m, loss_abs_m, r_m = 0., 0., 0.

        Terr_real_sum_abs = 0.0 
        Terr_real_count   = 0   
        tr_axis_sum = torch.zeros(2, dtype=torch.float64)
        tr_axis_cnt = 0


        train_bar = tqdm(train_loader, total=iters_per_epoch, desc=f"Epoch {epoch}/{args.epochs}", ncols=100)
        optim.zero_grad(set_to_none=True)
        for step, (img1, _T1, _Te, _R1, _Re, _dT1e, _dR1e, e1e) in enumerate(train_bar,1):
            # deepset_feats = deepset(last_embedding)
            with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):

                loss, _, pose_pred = forward_batch(img1, _dT1e, _dR1e, e1e, args.backbone, args.transformer_layers, model_backbone,posehead, device)
                loss = loss / args.accum_iter

            scaler.scale(loss).backward()

            with torch.no_grad():
                pred_xy_norm = pose_pred
                gt_xy_norm = _dT1e[:, [1]]
                pred_real = inv_dT1e(pred_xy_norm)
                gt_real = inv_dT1e(gt_xy_norm)
                err_real = (pred_real.cpu() - gt_real.cpu()).abs()        # [B,2]
                Terr_real_sum_abs += err_real.sum().item()
                Terr_real_count += err_real.numel()


            if (step + 1) % args.accum_iter == 0:
                scaler.unscale_(optim)
                # torch.nn.utils.clip_grad_norm_(trainable_params, 2.0)
                torch.nn.utils.clip_grad_norm_( (p for g in optim.param_groups for p in g['params']),
                                max_norm=2.0, foreach=True, error_if_nonfinite=False )
                
                scaler.step(optim)
                scaler.update()
                optim.zero_grad(set_to_none=True)
                scheduler.step()

                loss_abs_m += (pose_pred.squeeze(-1) - _dT1e[:,1].to(device)).abs().mean().item()

                loss_m += loss.item() * args.accum_iter
                if args.stepsperepoch and step + 1 >= args.stepsperepoch:
                    break

        train_total = loss_m / (step + 1)
        train_abs_total = loss_abs_m / (step + 1)
        train_loss_real = Terr_real_sum_abs / max(1, Terr_real_count)

        model_backbone.eval()
        posehead.eval()
        vl_s = 0
        bound_open_all = 0
        va_sum_abs = 0.0
        va_count   = 0

        with torch.no_grad():
            val_bar = tqdm(val_loader, ncols=100, disable=not show_bar, desc="val")
            for img1, _T1, _Te, _R1, _Re, _dT1e, _dR1e, e1e in val_bar:
                loss, bound_open, pose_pred = forward_batch(img1, _dT1e, _dR1e, e1e, args.backbone, args.transformer_layers, model_backbone, posehead, device)
                pose_pred = pose_pred.clamp(0, 1)
                
                
                bound_open_all += bound_open.item()         
                vl_s += loss.item()

                pred_real = inv_dT1e(pose_pred)
                gt_real = inv_dT1e(_dT1e[:, [1]])
                err = (pred_real.cpu() - gt_real.cpu()).abs()
                va_sum_abs += err.sum().item()
                va_count += err.numel()


        val_total = vl_s / len(val_loader)
        bound_open_val = bound_open_all / len(val_loader)
        val_loss_real = va_sum_abs / max(1, va_count)

        print(f"E{epoch:03d} | "
              f"Train {train_total:.4f}, {train_abs_total:.4f} | Val {val_total:.4f}, {bound_open_val:.4f} | LR {scheduler.get_last_lr()[0]:.2e} | "
              f"real Train={train_loss_real:.4f}"
              f"real Val={val_loss_real:.4f}")

        log_writer.add_scalar('train loss', train_total, epoch)
        log_writer.add_scalar('val loss', val_total,   epoch)

        log_writer.add_scalar('train loss real', train_loss_real, epoch)
        log_writer.add_scalar('val loss real', val_loss_real, epoch)


        with open(os.path.join(out_dir, 'train_log.csv'), 'a', newline='') as f:
            csv.writer(f).writerow([epoch, train_total, val_total, train_loss_real, val_loss_real])

        if val_loss_real < best_val:
            best_val = val_loss_real
            torch.save({
                'backbone': model_backbone.state_dict(),
                'posehead':posehead.state_dict(),
                'epoch': epoch,
            }, os.path.join(out_dir, 'weights/best.pth'))
    print('Training completed.')


if __name__ == '__main__':
    main()


