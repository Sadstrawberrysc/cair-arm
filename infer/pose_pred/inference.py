# import os
# import csv
# import math
# import argparse

# import cv2
# import numpy as np
# import torch
# import torch.nn.functional as F
# from utils import (
#     MyCarotidDataset,
#     set_randomness,
#     rot6d_to_matrix,
#     geodesic_loss_squared,
# )                  
# from torch.utils.data import Dataset, DataLoader
# import torchvision.transforms as T

# from utils import rot6d_to_matrix
# from Mymodels import (
#     models_mae,
#     SwinTransformer,
#     SwinTransformerV2,
#     PoseHead2,
#     PoseViTCore,
#     GuidanceHead,
#     DeepSet2Image,
#     PoseClsHead
# )
# from pose_train import build_dataset_transform, forward_batch

# from utils import (
#     build_inv_normalizer_from_dataset,
# )

# def _normalize_key(k):
#     import re
#     replacements = [
#         (r'^module\.', ''),
#         (r'^encoder\.', ''),
#         (r'\.mlp\.fc1\.', '.mlp.linear1.'),
#         (r'\.mlp\.fc2\.', '.mlp.linear2.'),
#         (r'layers\.(\d+)\.', r'layers\1.'),
#     ]
#     for pat, rep in replacements:
#         k = re.sub(pat, rep, k)
#     return k



# def build_swinmodel(args, device):
#     model = SwinTransformer(
#         in_chans=3, embed_dim=48, patch_size=(2, 2), window_size=(7, 7),
#         depths=(2, 2, 2, 2), num_heads=(3, 6, 12, 24),
#         mlp_ratio=4., qkv_bias=True, spatial_dims=2, use_v2=True
#     ).to(device).eval()

#     ckpt_path = args.swincheckpoint
#     if os.path.isfile(ckpt_path):
#         raw = torch.load(ckpt_path, map_location='cpu')
#         raw = raw.get('model', raw)
#         ckpt = {_normalize_key(k): v for k, v in raw.items()}
#         loadable = {k: v for k, v in ckpt.items()
#                     if k in model.state_dict() and v.shape == model.state_dict()[k].shape}
#         msg = model.load_state_dict(loadable, strict=False)
#         print(f"swin pretrain loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
#     else:
#         print(f"swin pretrain not found: {ckpt_path}, random init.")
#     return model

# def build_swinmodel_origin(args, device):
#     swin = SwinTransformerV2(
#         img_size=256, patch_size=2, in_chans=3,
#         num_classes=0,
#         embed_dim=48, depths=(2, 2, 2, 2),
#         num_heads=(3, 6, 12, 24),
#         window_size=8, drop_path_rate=0.3
#     ).to(device).eval()

#     ckpt_p = args.swinocheckpoint
#     if os.path.isfile(ckpt_p):
#         raw_state = torch.load(ckpt_p, map_location="cpu")
#         raw_state = raw_state.get("model", raw_state)
#         enc_state = {k: v for k, v in raw_state.items() if not k.startswith("head.")}
#         swin.load_state_dict(enc_state, strict=False)
#         print("[swino pretrain loaded.")
#     else:
#         print(f"swino pretrain not found: {ckpt_p}, random init.")
#     return swin

# def build_maemodel(args, device):
#     assert hasattr(models_mae, args.maemodel), f"models_mae.py cannot find '{args.maemodel}'"
#     model = getattr(models_mae, args.maemodel)(norm_pix_loss=False).to(device).eval()
#     ckpt = torch.load(args.maecheckpoint, map_location='cpu', weights_only=False)
#     state_dict = ckpt["model"] if "model" in ckpt else ckpt
#     msg = model.load_state_dict(state_dict, strict=False)
#     print(f"mae pretrain loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
#     return model


# def center_crop_numpy(img, size):
#     h, w = img.shape[:2]
#     th, tw = size, size
#     y1 = max(0, (h - th) // 2)
#     x1 = max(0, (w - tw) // 2)
#     return img[y1:y1+th, x1:x1+tw, :]

# def preprocess_cv2(img_bgr, backbone, size):
#     if backbone in ('swin', 'swino', 'transformer'):
#         interp = cv2.INTER_CUBIC  
#         img_bgr = cv2.resize(img_bgr, (size, size), interpolation=interp)
#     else:
#         raise ValueError(backbone)

#     if backbone == 'transformer':

#         img_bgr = cv2.resize(img_bgr, (size, size), interpolation=cv2.INTER_CUBIC)
#         img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
#         mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
#         std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
#         img_rgb = (img_rgb - mean) / std
#     else:
#         img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

#     chw = np.transpose(img_rgb, (0, 1, 2)).transpose(2, 0, 1) 
#     tensor = torch.from_numpy(chw).unsqueeze(0)
#     return tensor

# def get_args():
#     ap = argparse.ArgumentParser("DSB")
#     ap.add_argument('--input', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/datasets/data_0704MC_carotid_copilot_pose_split/test/20250626141823')
#     ap.add_argument('--out_dir', default='./runs/infer_out')

#     ap.add_argument('--backbone', default='transformer', choices=['swin','swino','transformer'])
#     ap.add_argument('--swincheckpoint', default='/path/to/swin_pretrain.pt')
#     ap.add_argument('--swinocheckpoint', default='/path/to/swino_pretrain.pth')
#     ap.add_argument('--maecheckpoint', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/pretrain_weights/mae/carotid_base_checkpoint-500_0721.pth')
#     ap.add_argument('--maemodel', default='mae_vit_base_patch16')
#     ap.add_argument('--transformer_layers', default=[5])

#     ap.add_argument('--posehead_ckpt', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/pose/20250909_153802/weights/best.pth')
#     ap.add_argument('--anglehead_ckpt', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/angle/20250911_171603/weights/best.pth')
#     ap.add_argument('--classhead_ckpt', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/classification/20250909_145451/weights/best.pth')

#     ap.add_argument('--class_names', nargs='+', default=['diagonally', 'long'])
#     ap.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')

#     return ap.parse_args()

# @torch.no_grad()
# def extract_embedding(backbone, model, img_tensor, pick_indices):
#     if backbone == 'swin':
#         feat = model.forward_features(img_tensor, feat_type='pyramid')[-1]
#         return feat
#     elif backbone == 'swino':
#         feat = model.forward_features2(img_tensor)
#         return feat
#     elif backbone == 'transformer':
#         x, emb = model.forward_encoder2(img_tensor, mask_ratio=0.0, pick_indices=pick_indices)
#         return emb
#     else:
#         raise ValueError(backbone)


# def list_images(input_path):
#     exts = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif'}
#     if os.path.isdir(input_path):
#         files = sorted(os.listdir(input_path))
#         files = [os.path.join(input_path, f) for f in files
#                  if os.path.splitext(f)[1].lower() in exts]
#         return files
#     elif os.path.isfile(input_path):
#         ext = os.path.splitext(input_path)[1].lower()
#         if ext in exts:
#             return [input_path]
#         else:
#             raise ValueError(f"Unsupported image extension: {ext}")
#     else:
#         raise FileNotFoundError(input_path)


# def main():
#     args = get_args()
#     device = torch.device(args.device)
    
#     stats_path = "/home/jiuan/AAAworkspace/Myproject/slicelocalization/datasets/data_0704MC_carotid_copilot_pose_split/norm_stats_all.pt"
#     inv_dT1e = build_inv_normalizer_from_dataset(key = "dT_1e", 
#                                                  stats_path= stats_path, 
#                                                  mode = "minmax_pad", 
#                                                  pad_ratio = 0.05, 
#                                                  feat_axis=-1, 
#                                                  index=[0, 1])

#     img_size = 256 if args.backbone in ('swin', 'swino') else 224

#     if args.backbone == 'swin':
#         backbone = build_swinmodel(args, device)
#     elif args.backbone == 'swino':
#         backbone = build_swinmodel_origin(args, device)
#     else:
#         backbone = build_maemodel(args, device)
    
#     backbone_outlayersnum =  len(args.transformer_layers)
#     posehead = PoseHead2(
#         in_dim = 768,
#         num_levels = backbone_outlayersnum,
#         per_level = 256,
#         fuse_mid = 256,
#         out_dim = 2,
#         dropout = 0.1,
#         ).to(device).eval()
    
#     anglehead = PoseHead2(
#         in_dim = 768,
#         num_levels = backbone_outlayersnum,
#         per_level = 256,
#         fuse_mid = 256,
#         out_dim = 3,
#         dropout = 0.1,
#         ).to(device).eval()
    
#     num_classes = len(args.class_names)
#     classhead = PoseClsHead(
#         in_dim=768,
#         num_levels=backbone_outlayersnum,
#         per_level=256,
#         fuse_mid=256,
#         num_classes=num_classes, 
#         binary_mode="ce", 
#         dropout=0.1,
#     ).to(device).eval()

#     posehead_ckpt = torch.load(args.posehead_ckpt, map_location='cpu', weights_only=False)
#     anglehead_ckpt = torch.load(args.anglehead_ckpt, map_location='cpu', weights_only=False)
#     classhead_ckpt = torch.load(args.classhead_ckpt, map_location='cpu', weights_only=False)

#     if 'backbone' in posehead_ckpt:
#         missing, unexpected = backbone.load_state_dict(posehead_ckpt['backbone'], strict=False)
#         print(f"backbone fine-tuned loaded: missing={len(missing)} unexpected={len(unexpected)}")
#     else:
#         print("posehead_ckpt ckpt has no 'backbone' — using only pretrain weights.")

#     if 'posehead' in posehead_ckpt:
#         missing, unexpected = posehead.load_state_dict(posehead_ckpt['posehead'], strict=False)
#         print(f"posehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
#     if 'posehead' in anglehead_ckpt:
#         missing, unexpected = anglehead.load_state_dict(anglehead_ckpt['posehead'], strict=False)
#         print(f"anglehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
#     if 'clshead' in classhead_ckpt:
#         missing, unexpected = classhead.load_state_dict(classhead_ckpt['clshead'], strict=False)
#         print(f"classhead loaded: missing={len(missing)} unexpected={len(unexpected)}")
#     else:
#         print("finetune ckpt has NO 'posehead'. The head will be random.")


#     out_dir = args.out_dir
#     os.makedirs(out_dir, exist_ok=True)


#     paths = list_images(args.input)
#     print(f"found {len(paths)} images.")
#     for p in paths:


#         img_bgr = cv2.imread(p, cv2.IMREAD_COLOR)
#         if img_bgr is None:
#             print(f"[warn] failed to read image: {p}")
#             continue

#         img_tensor = preprocess_cv2(img_bgr, args.backbone, img_size).to(device)

#         with torch.no_grad():
        
#             emb = extract_embedding(args.backbone, backbone, img_tensor, pick_indices = args.transformer_layers)
#             pose = posehead(emb)
#             pred_xy_norm = pose[:, [0, 1]]
#             pred_xy_real = inv_dT1e(pred_xy_norm)


#             angle = anglehead(emb)*90

#             classlogits = classhead(emb)
#             classpreds = torch.argmax(classlogits, dim=1)
#             print("pose:",pred_xy_real, " angle:", angle, " class:",classpreds)

# if __name__ == "__main__":
#     main()


import os
import csv
import math
import argparse

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from utils import (
    set_randomness,
    rot6d_to_matrix,
    geodesic_loss_squared,
)                  
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as T

import torch.backends.cudnn as cudnn
from torch.backends.cuda import sdp_kernel

from utils import rot6d_to_matrix
from Mymodels import (
    models_mae,
    SwinTransformer,
    SwinTransformerV2,
    PoseHead2,
    PoseHead4,
    PoseViTCore,
    GuidanceHead,
    DeepSet2Image,
    PoseClsHead,
    PoseClsHead2,
)

from utils import (
    build_inv_normalizer_from_dataset,
)

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



def build_swinmodel(args, device):
    model = SwinTransformer(
        in_chans=3, embed_dim=128, patch_size=(2, 2), window_size=(8, 8),
        depths=(2, 2, 18, 2), num_heads=(4, 8, 16, 32),
        mlp_ratio=4., qkv_bias=True, spatial_dims=2, use_v2=True
    ).to(device).eval()

    ckpt_path = args.swincheckpoint
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

def build_swinmodel_origin(args, device):
    swin = SwinTransformerV2(
        img_size=256, patch_size=2, in_chans=3,
        num_classes=0,
        embed_dim=48, depths=(2, 2, 2, 2),
        num_heads=(3, 6, 12, 24),
        window_size=8, drop_path_rate=0.3
    ).to(device).eval()

    ckpt_p = args.swinocheckpoint
    if os.path.isfile(ckpt_p):
        raw_state = torch.load(ckpt_p, map_location="cpu")
        raw_state = raw_state.get("model", raw_state)
        enc_state = {k: v for k, v in raw_state.items() if not k.startswith("head.")}
        swin.load_state_dict(enc_state, strict=False)
        print("[swino pretrain loaded.")
    else:
        print(f"swino pretrain not found: {ckpt_p}, random init.")
    return swin

def build_maemodel(args, device):
    assert hasattr(models_mae, args.maemodel), f"models_mae.py cannot find '{args.maemodel}'"
    model = getattr(models_mae, args.maemodel)(norm_pix_loss=False).to(device).eval()
    ckpt = torch.load(args.maecheckpoint, map_location='cpu', weights_only=False)
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

        # img_bgr = center_crop_numpy(img_bgr, size)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img_rgb = (img_rgb - mean) / std
    else:
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

    chw = np.transpose(img_rgb, (0, 1, 2)).transpose(2, 0, 1) 
    tensor = torch.from_numpy(chw).unsqueeze(0)
    return tensor



def get_args():
    ap = argparse.ArgumentParser("Pose inference")
    ap.add_argument('--input', default='/home/cair-jacen/embodiedUS/carotid_copilot251015/20250626141737/20250626_141748758.jpg')
    ap.add_argument('--out_dir', default='./runs/infer_out')

    ap.add_argument('--backbone', default='transformer', choices=['swin','swino','transformer'])
    ap.add_argument('--swincheckpoint', default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/pretrain_weights/simmim/swin_ckp_final_2.pt')
    ap.add_argument('--swinocheckpoint', default='/path/to/swino_pretrain.pth')
    ap.add_argument('--maecheckpoint', default='/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/carotid_base_checkpoint-500_0721.pth')
    ap.add_argument('--maemodel', default='mae_vit_base_patch16')
    ap.add_argument('--transformer_layers', default=[9])

    ap.add_argument('--posehead_ckpt', default='/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/pose/best.pth')
    ap.add_argument('--anglehead_ckpt', default='/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/angle/best.pth')
    ap.add_argument('--classhead_ckpt', default='/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/classification/best.pth')

    ap.add_argument('--class_names', nargs='+', default=['diagonally', 'long'])
    ap.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')

    return ap.parse_args()

# @torch.no_grad()
def extract_embedding(backbone, model, img_tensor, pick_indices):
    if backbone == 'swin':
        feat = model.forward_features(img_tensor, feat_type='pyramid')[-1]
        return [feat]
    
    elif backbone == 'swino':
        feat = model.forward_features2(img_tensor)
        return feat
    elif backbone == 'transformer':
        x, feats, emb = model.forward_encoder2(img_tensor, mask_ratio=0.0, pick_indices=pick_indices)
        return x, feats, emb
    else:
        raise ValueError(backbone)


def list_images(input_path):
    exts = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif'}
    if os.path.isdir(input_path):
        files = sorted(os.listdir(input_path))
        files = [os.path.join(input_path, f) for f in files
                 if os.path.splitext(f)[1].lower() in exts]
        return files
    elif os.path.isfile(input_path):
        ext = os.path.splitext(input_path)[1].lower()
        if ext in exts:
            return [input_path]
        else:
            raise ValueError(f"Unsupported image extension: {ext}")
    else:
        raise FileNotFoundError(input_path)

import time


def speed_setup():
    # 与训练对齐/更强
    torch.set_float32_matmul_precision("high")     # 允许TF32更快的matmul
    torch.backends.cuda.matmul.allow_tf32 = True
    cudnn.benchmark = True                         # 固定分辨率时应该打开
    # 强制使用更快的注意力内核（PyTorch 2.x）
    # sdp_kernel(enable_flash=True, enable_math=False, enable_mem_efficient=True)


def main():
    args = get_args()
    device = torch.device(args.device)
    cudnn.benchmark = True
    # speed_setup()
    stats_path = "/home/cair-jacen/embodiedUS/carotid_copilot251015/weights/norm_stats_all.pt"
    inv_dT1e = build_inv_normalizer_from_dataset(key = "dT_1e", 
                                                 stats_path= stats_path, 
                                                 mode = "minmax_pad", 
                                                 pad_ratio = 0.05, 
                                                 feat_axis=-1, 
                                                 index=[1])

    img_size = 256 if args.backbone in ('swin', 'swino') else 224

    if args.backbone == 'swin':
        backbone = build_swinmodel(args, device)
    elif args.backbone == 'swino':
        backbone = build_swinmodel_origin(args, device)
    else:
        backbone = build_maemodel(args, device)
    
    backbone_outlayersnum =  len(args.transformer_layers)
    posehead = PoseHead4(
        in_dim = 768,
        out_dim = 1,
        ).to(device).eval()
    
    anglehead = PoseHead4(
        in_dim = 768,
        out_dim = 1,
        ).to(device).eval()

    num_classes = len(args.class_names)
    classhead = PoseClsHead2(
        in_dim=768,
        num_classes=num_classes, 
        binary_mode="ce", 
    ).to(device).eval()

    posehead_ckpt = torch.load(args.posehead_ckpt, map_location='cpu', weights_only=False)
    anglehead_ckpt = torch.load(args.anglehead_ckpt, map_location='cpu', weights_only=False)
    classhead_ckpt = torch.load(args.classhead_ckpt, map_location='cpu', weights_only=False)

    # if 'backbone' =='swin':
    #     backbone_ckpt = torch.load(args.posehead_ckpt, map_location='cpu', weights_only=False)
    #     missing, unexpected = backbone.load_state_dict(posehead_ckpt['backbone'], strict=False)
    #     print(f"backbone fine-tuned loaded: missing={len(missing)} unexpected={len(unexpected)}")
    # else:
    #     print("posehead_ckpt ckpt has no 'backbone' — using only pretrain weights.")

    if 'posehead' in posehead_ckpt:
        missing, unexpected = posehead.load_state_dict(posehead_ckpt['posehead'], strict=False)
        print(f"posehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    if 'posehead' in anglehead_ckpt:
        missing, unexpected = anglehead.load_state_dict(anglehead_ckpt['posehead'], strict=False)
        print(f"anglehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    if 'clshead' in classhead_ckpt:
        missing, unexpected = classhead.load_state_dict(classhead_ckpt['clshead'], strict=False)
        print(f"classhead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    else:
        print("finetune ckpt has NO 'posehead'. The head will be random.")


    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)

    # paths = list_images(args.input)
    # print(f"found {len(paths)} images.")
    # for p in paths:

    img_bgr = cv2.imread(args.input, cv2.IMREAD_COLOR)
    if img_bgr is None:
        print(f"[warn] failed to read image: {args.input}")
        exit()

    img_tensor = preprocess_cv2(img_bgr, args.backbone, img_size).to(device)
    
    with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):
        dummy_img = torch.rand_like(img_tensor).to(device)  # 模拟输入
        for _ in range(5):
            extract_embedding(args.backbone, backbone, dummy_img, pick_indices=args.transformer_layers)
            
        time1 = time.time()
        # with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):
        # emb = extract_embedding(args.backbone, backbone, img_tensor, pick_indices = args.transformer_layers)
        x, feats, emb = extract_embedding(args.backbone, backbone, img_tensor, pick_indices=args.transformer_layers)
        time2 = time.time()
        print((time2 - time1))  # 平均时间
        
    pose = posehead(feats[-1], x)
    pred_xy_norm = pose
    pred_xy_real = inv_dT1e(pred_xy_norm)


    angle = anglehead(feats[-1], x)*90

    classlogits = classhead(x)
    classpreds = torch.argmax(classlogits, dim=1)
    print("pose:",pred_xy_real, " angle:", angle, " class:",classpreds)

if __name__ == "__main__":
    main()



