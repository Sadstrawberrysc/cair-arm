#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, csv, random, math, json
import numpy as np
import cv2
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset
from PIL import Image
from scipy.spatial.transform import Rotation as R

cv2.setNumThreads(0)
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp")
POS_SCALE = 1.0

def safe_float(x, default=np.nan):
    try:
        if x is None:
            return default
        s = str(x).strip()
        if s == '' or s.lower() in ('nan', 'none', 'null'):
            return default
        return float(s)
    except Exception:
        return default

def safe_int(x, default=0):
    try:
        if x is None:
            return default
        s = str(x).strip()
        if s == '':
            return default
        return int(float(s))
    except Exception:
        return default

def is_image_file(path):
    return os.path.isfile(path) and os.path.splitext(path)[1].lower() in IMAGE_EXTS


def read_image_rgb(path):
    # PIL 保持通道顺序为 RGB
    with Image.open(path) as im:
        rgb = np.asarray(im.convert("RGB"), dtype=np.uint8)
    return rgb.copy() if not rgb.flags.writeable else rgb


def split_pose(v):
    return v[0:3], v[3:12].reshape(3, 3)


def euler_from_rotmat_np(rotmat3x3, order='xyz', degrees=True):
    return R.from_matrix(rotmat3x3).as_euler(order, degrees=degrees)


def clip_int(v, lo, hi):
    return int(max(lo, min(hi, int(round(v)))))


def translate_image(img, dx, dy):
    if dx == 0 and dy == 0:
        return img.copy()
    h, w = img.shape[:2]
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_NEAREST,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def rect_intersects(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    return not (ax2 <= bx1 or bx2 <= ax1 or ay2 <= by1 or by2 <= ay1)


def sample_idx_back(x, p):
    a = -p * x
    b = -(1 - p) * x
    lo = math.ceil(min(a, b))
    hi = math.floor(max(a, b))
    if lo <= hi:
        return random.randint(lo, hi)
    r = random.uniform(min(a, b), max(a, b))
    return int(round(r))


def occlude_outside_bbox(img, bbox, occ_max, size_min, size_max, rng_trials=50):
    if occ_max <= 0 or size_max <= 0:
        return img
    h, w = img.shape[:2]
    x1 = clip_int(bbox[0], 0, w-1)
    y1 = clip_int(bbox[1], 0, h-1)
    x2 = clip_int(bbox[2], 0, w-1)
    y2 = clip_int(bbox[3], 0, h-1)
    avoid = None if (x2 <= x1 or y2 <= y1) else (x1, y1, x2, y2)

    k = random.randint(0, occ_max)
    out = img
    for _ in range(k):
        placed = False
        for _ in range(rng_trials):
            L = min(w, h)
            bw = int(random.uniform(size_min, size_max) * L)
            bh = int(random.uniform(size_min, size_max) * L)
            bw = max(2, min(bw, w))
            bh = max(2, min(bh, h))
            x = random.randint(0, max(0, w - bw))
            y = random.randint(0, max(0, h - bh))
            rect = (x, y, x + bw, y + bh)
            if avoid is not None and rect_intersects(rect, avoid):
                continue
            out[y:y+bh, x:x+bw] = 0
            placed = True
            break
        if not placed:
            break
    return out

class MyCarotidPoseDataset(Dataset):
    def __init__(self,
                 root_dir,
                 split="train",
                 image_interval=1,
                 transform=None,
                 normalize=True,
                 norm_mode="minmax_pad",
                 pad_ratio=0.05,
                 euler_degrees=True,
                 euler_order='zyx',
                 return_end_image=False,
                 include_last=True,
                 flip_expand=False,

                 aug_enable=True,
                 p_flip=0.5,
                 p_crop=0.3,
                 p_elastic=0.10,
                 p_noise=0.5,
                 crop_scale_rangeH=(0.8, 1.0),
                 crop_scale_rangeW=(0.8, 1.0),
                 elastic_alpha_px=(1.0, 3.0),
                 elastic_sigma_px=(3.0, 6.0),
                 noise_std=(0.01, 0.03),

                 # 动静脉 shift 增强
                 shift_mode='dynamic',  # 设为 'none' 关闭位移增强
                 shift_probe_h=1.0,
                 shift_probe_v=1.0,
                 yolo_max_dx=-1,
                 yolo_max_dy=-1,
                 yolo_occ_max=5,
                 yolo_occ_size_min=0.06,
                 yolo_occ_size_max=0.12,
                 dynamic_k=8,                    
                 pregen_k=8,                     

                 # 策略控制
                 # 'pixel_dis': 强制利用 YOLO 计算目标距离 (需要 YOLO ok=1)
                 # 'pose' / 'original': 使用原始机械臂位姿差值 (忽略 YOLO 状态)
                 gt_policy='pixel_dis',          
                 ptx_only = 1,                   
                 ptx_center=300.0,
                 ptx_scale=6.25e-5,
                 return_dict=False):
                 
        super().__init__()
        self.root_dir = os.path.abspath(root_dir)
        self.split = split
        self.image_interval = max(1, int(image_interval))
        self.transform = transform
        self.normalize = bool(normalize)
        self.norm_mode = norm_mode
        self.pad_ratio = float(pad_ratio)
        self.euler_degrees = bool(euler_degrees)
        self.euler_order = euler_order
        self.return_end_image = bool(return_end_image)
        self.include_last = bool(include_last)
        self.flip_expand = bool(flip_expand)
        self.return_dict = bool(return_dict)


        self.aug_enable = bool(aug_enable)
        self.aug_cfg = {
            "out_hw": None,
            "p_flip": float(p_flip),
            "p_crop": float(p_crop),
            "p_elastic": float(p_elastic),
            "p_noise": float(p_noise),
            "crop_scale_rangeH": crop_scale_rangeH,
            "crop_scale_rangeW": crop_scale_rangeW,
            "elastic_alpha_px": elastic_alpha_px,
            "elastic_sigma_px": elastic_sigma_px,
            "noise_std": noise_std,
        }

        self.shift_mode = str(shift_mode)
        self.shift_probe_h = float(shift_probe_h)
        self.shift_probe_v = float(shift_probe_v)
        self.yolo_max_dx = int(yolo_max_dx)
        self.yolo_max_dy = int(yolo_max_dy)
        self.yolo_occ_max = int(yolo_occ_max)
        self.yolo_occ_size_min = float(yolo_occ_size_min)
        self.yolo_occ_size_max = float(yolo_occ_size_max)
        self.dynamic_k = max(0, int(dynamic_k))
        self.pregen_k = max(0, int(pregen_k))

        self.gt_policy = str(gt_policy)
        self.ptx_only = bool(ptx_only)        
        self.ptx_center = float(ptx_center)
        self.ptx_scale = float(ptx_scale)

        self.stats = self.load_norm_stats() if self.normalize else {}
        keys = ("T", "Tend", "dT_1e")
        self.use_stats = {k: all(self.stats.get(f"{k}_"+s) is not None for s in ("mean","std","min","rng"))
                          for k in keys} if self.normalize else {k: False for k in keys}

        self.seq_meta = {}
        self.samples = []        # (seq_dir, idx, last, flip_flag, aug_tag)
        split_dir = os.path.join(self.root_dir, self.split)
        if not os.path.isdir(split_dir):
            raise FileNotFoundError(f"split dir not found: {split_dir}")

        seq_names = sorted([d for d in os.listdir(split_dir) if os.path.isdir(os.path.join(split_dir, d))])
        
        for seq in seq_names:
            sd = os.path.join(split_dir, seq)
            csv_path = None
            for fn in os.listdir(sd):
                if fn.lower().endswith('.csv'):
                    csv_path = os.path.join(sd, fn)
                    break
            if csv_path is None:
                continue

            imgs, poses, dt1e, dt1e_valid, dt1f, dt1f_valid, yolo_block = self.read_csv_pose(csv_path, sd)
            last = len(imgs) - 1
            self.seq_meta[sd] = {
                "imgs": imgs,
                "pose": poses,
                "last": last,
                "dt1e": dt1e,
                "dt1e_valid": dt1e_valid,
                "dt1f": dt1f,
                "dt1f_valid": dt1f_valid,
                "yolo": yolo_block, 
            }

            end_i = last if self.include_last else last - 1
            
            # --- 核心修改区域：样本生成循环 ---
            for i in range(0, end_i + 1, self.image_interval):
                # 1. 获取 YOLO 状态
                ok = int(yolo_block['ok'][i]) if yolo_block is not None else 0

                # 2. 【修改】根据 gt_policy 决定是否强制过滤
                # 如果 gt_policy 是 'pixel_dis'，则必须要求 YOLO 检测到血管 (ok=1)
                # 如果 gt_policy 是其他 (如 'pose', 'original')，则不做硬性过滤
                if self.gt_policy == 'pixel_dis' and ok != 1:
                    continue
                
                # 3. 添加原始图片样本 (Base Sample)
                base = (sd, i, last, False, 'base')
                self.samples.append(base)

                if self.flip_expand:
                    self.samples.append((sd, i, last, True, 'base'))

                # 4. 添加动态平移扩充样本 (Dynamic Shift Samples)
                # 【修改】增加 and ok == 1 条件
                # 只有 YOLO 正常检测到，才有“目标点”可供我们进行平移增强
                if self.shift_mode == 'dynamic' and self.dynamic_k > 0 and ok == 1:
                    for k in range(0, self.dynamic_k):
                        self.samples.append((sd, i, last, False, f'dyn{k}'))

        self.env_y1, self.env_y2 = self.compute_global_env_y()

        # 预生成模式的处理
        self.pregen_shifts = {}
        if self.gt_policy == 'pixel_dis' and self.shift_mode == 'pregen' and self.pregen_k > 0:
            aug_list = []
            for idx, s in enumerate(list(self.samples)):
                sd, i, last, flip_flag, tag = s
                
                # 获取该帧 YOLO 状态
                yb = self.seq_meta[sd]['yolo']
                ok = int(yb['ok'][i]) if yb is not None else 0

                # 只有 YOLO 正常才生成预生成 shift 样本
                if ok == 1:
                    for k in range(self.pregen_k):
                        dx, dy = self.sample_dx_dy_for_index(sd, i)
                        self.pregen_shifts[(sd, i, k)] = (dx, dy)
                        aug_list.append((sd, i, last, flip_flag, f'pregen{k}'))
            self.samples.extend(aug_list)


    def read_csv_pose(self, csv_path, img_dir):
        def read_dt_triplet(row, keys):
            vals = []
            success = True
            for k in keys:
                if k in row and row[k] not in (None, ""):
                    try:
                        vals.append(float(row[k]) * POS_SCALE)
                    except Exception:
                        success = False
                        break
                else:
                    success = False
                    break
            if success and len(vals) == 3:
                return vals, True
            return [float('nan')]*3, False

        imgs, poses = [], []
        dt1e_list, dt1e_valid = [], []
        dt1f_list, dt1f_valid = [], []

        y_ok, y_ptx, y_pty = [], [], []
        y_bx1, y_by1, y_bx2, y_by2 = [], [], [], []

        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                img_name = row.get('img')
                if not img_name:
                    continue
                if not is_image_file(os.path.join(img_dir, img_name)):
                    # 尝试不同扩展名
                    base = os.path.splitext(img_name)[0]
                    found = None
                    for ext in IMAGE_EXTS:
                        cand = os.path.join(img_dir, base+ext)
                        if os.path.isfile(cand):
                            found = cand
                            img_name = os.path.basename(cand)
                            break
                    if not found:
                        continue

                # 位姿 3 + 9
                Txyz = [float(row[k]) * POS_SCALE for k in ("x","y","z")]
                R9 = [float(row[k]) for k in ("r00","r01","r02","r10","r11","r12","r20","r21","r22")]

                v1e, s1e = read_dt_triplet(row, ("dx1e","dy1e","dz1e"))
                v1f, s1f = read_dt_triplet(row, ("dx1f","dy1f","dz1f"))


                ok = safe_int(row.get('yolo_ok'), 0) # 建议开启 safe_int 读取
                ptx = safe_float(row.get('yolo_pt_x'), 0)
                pty = safe_float(row.get('yolo_pt_y'), 0)
                bx1 = safe_float(row.get('yolo_big_x1'), 0)
                by1 = safe_float(row.get('yolo_big_y1'), 0)
                bx2 = safe_float(row.get('yolo_big_x2'), 0)
                by2 = safe_float(row.get('yolo_big_y2'), 0)


                imgs.append(img_name)
                poses.append(Txyz + R9)
                dt1e_list.append(v1e)
                dt1e_valid.append(s1e)
                dt1f_list.append(v1f)
                dt1f_valid.append(s1f)

                y_ok.append(ok)
                y_ptx.append(ptx)
                y_pty.append(pty)
                y_bx1.append(bx1)
                y_by1.append(by1)
                y_bx2.append(bx2)
                y_by2.append(by2)

        if not imgs:
            raise ValueError(f"{csv_path} 有效行为空")

        poses  = np.asarray(poses,  np.float32)
        dt1e   = np.asarray(dt1e_list, np.float32)
        dt1f   = np.asarray(dt1f_list, np.float32)
        v1e_mk = np.asarray(dt1e_valid, np.bool_)
        v1f_mk = np.asarray(dt1f_valid, np.bool_)

        yolo_block = {
            'ok':   np.asarray(y_ok,  np.int32),
            'ptx':  np.asarray(y_ptx, np.float32),
            'pty':  np.asarray(y_pty, np.float32),
            'bx1':  np.asarray(y_bx1, np.float32),
            'by1':  np.asarray(y_by1, np.float32),
            'bx2':  np.asarray(y_bx2, np.float32),
            'by2':  np.asarray(y_by2, np.float32),
        }

        return imgs, poses, dt1e, v1e_mk, dt1f, v1f_mk, yolo_block


    def load_norm_stats(self):
        fname = "norm_stats_all.pt"
        pt = os.path.join(self.root_dir, fname)
        js = os.path.splitext(pt)[0] + '.json'
        if os.path.isfile(pt):
            return torch.load(pt, map_location='cpu')
        if os.path.isfile(js):
            with open(js, 'r') as f:
                jd = json.load(f)
            return {k: torch.tensor(v) for k, v in jd.items()}
        # print(f"[normalize disabled] stats file not found: '{pt}' or '{js}'")
        self.normalize = False
        return {}

    def compute_global_env_y(self):
        y1_all, y2_all = [], []
        for sd, meta in self.seq_meta.items():
            yb = meta['yolo']
            ok = (yb['ok'] == 1)
            if np.any(ok):
                y1 = yb['by1'][ok]
                y2 = yb['by2'][ok]
                if y1.size and y2.size:
                    y1_all.append(np.nanmin(y1))
                    y2_all.append(np.nanmax(y2))
        if len(y1_all) == 0:
            return None, None
        return float(np.nanmin(y1_all)), float(np.nanmax(y2_all))

    def elastic_deform_numpy_v2(self, img_chw):
        mode = 'gaussian'
        strength = 'mild'
        alpha_px = self.aug_cfg['elastic_alpha_px']
        sigma_px = self.aug_cfg['elastic_sigma_px']
        img = img_chw.detach().cpu().numpy().transpose(1,2,0)
        H,W = img.shape[:2]
        a = np.random.uniform(*alpha_px)
        s = np.random.uniform(*sigma_px)
        dx = np.random.randn(H,W).astype(np.float32)
        dy = np.random.randn(H,W).astype(np.float32)
        ksize = int(2*round(3*s)+1)
        dx = cv2.GaussianBlur(dx,(ksize,ksize),sigmaX=s)
        dy = cv2.GaussianBlur(dy,(ksize,ksize),sigmaX=s)
        dx *= a
        dy *= a
        xx,yy = np.meshgrid(np.arange(W,dtype=np.float32), np.arange(H,dtype=np.float32))
        map_x = (xx+dx).astype(np.float32)
        map_y=(yy+dy).astype(np.float32)
        img_def = cv2.remap(img,map_x,map_y,interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT101)
        out = torch.from_numpy(img_def.transpose(2,0,1)).to(img_chw.dtype)
        return out

    def augment_basic(self, img_chw, force_flip=None):

        cfg = self.aug_cfg
        out_hw = cfg['out_hw']
        p_flip = cfg['p_flip']
        p_crop = cfg['p_crop']
        p_elastic = cfg['p_elastic']
        p_noise = cfg['p_noise']
        crop_scale_rangeH = cfg['crop_scale_rangeH']
        crop_scale_rangeW = cfg['crop_scale_rangeW']
        noise_std = cfg['noise_std']

        C,H,W = img_chw.shape
        flipped = False
        # flip
        do_flip = bool(force_flip) if force_flip is not None else (random.random()<p_flip)
        if do_flip:
            img_chw = torch.flip(img_chw, dims=[-1])
            flipped=True

        # crop
        if random.random() < p_crop:
            sH = random.uniform(*crop_scale_rangeH)
            sW = random.uniform(*crop_scale_rangeW)
            ch = max(1,int(round(H*sH)))
            cw = max(1,int(round(W*sW)))
            y0 = 0 if H==ch else random.randint(0,H-ch)
            x0 = 0 if W==cw else random.randint(0,W-cw)
            img_chw = img_chw[:, y0:y0+ch, x0:x0+cw]

        # elastic
        if random.random() < p_elastic:
            img_chw = self.elastic_deform_numpy_v2(img_chw)

        # resize 回到 out_hw
        if out_hw is not None and img_chw.shape[-2:] != tuple(out_hw):
            img_chw = F.interpolate(img_chw.unsqueeze(0), size=out_hw, mode='bilinear', align_corners=False).squeeze(0)

        # noise
        if random.random() < p_noise:
            std = random.uniform(*noise_std)
            noise = torch.randn_like(img_chw) * std
            img_chw = torch.clamp(img_chw + noise, 0.0, 1.0)
        return img_chw, flipped



    def sample_dx_dy_for_index(self, seq_dir, i):
        yb = self.seq_meta[seq_dir]['yolo']
        ptx = float(yb['ptx'][i])
        pty = float(yb['pty'][i])
        x1 = float(yb['bx1'][i])
        y1 = float(yb['by1'][i])
        x2 = float(yb['bx2'][i])
        y2 = float(yb['by2'][i])
        img_path = os.path.join(seq_dir, self.seq_meta[seq_dir]['imgs'][i])
        img = read_image_rgb(img_path)
        h,w = img.shape[:2]

        # 水平范围 关键点不出界
        lo_dx = int(math.ceil(-ptx))
        hi_dx = int(math.floor((w-1)-ptx))
        if self.yolo_max_dx > 0:
            lo_dx = max(lo_dx, -self.yolo_max_dx)
            hi_dx = min(hi_dx, self.yolo_max_dx)
        dx = 0
        if self.shift_probe_h>0 and lo_dx<=hi_dx and random.random()<self.shift_probe_h:
            dx = random.randint(lo_dx, hi_dx)
        # 纵向范围 关键点不出界 ∩ 活动范围约束
        dy = 0
        if self.shift_probe_v>0 and random.random()<self.shift_probe_v:
            # 关键点不出界
            r_key = (int(math.ceil(-pty)), int(math.floor((h-1)-pty)))
            ranges = [r_key]
            # 活动范围
            if self.env_y1 is not None and self.env_y2 is not None:
                r_env = (int(math.ceil(self.env_y1 - y1)), int(math.floor(self.env_y2 - y2)))
                ranges.append(r_env)
            # 交集
            lo, hi = ranges[0]
            for r in ranges[1:]:
                lo = max(lo, r[0])
                hi = min(hi, r[1])
            if self.yolo_max_dy > 0:
                lo = max(lo, -self.yolo_max_dy)
                hi = min(hi, self.yolo_max_dy)
            if lo <= hi:
                dy = random.randint(lo, hi)
        return dx, dy

    def norm_tensor(self, key, x):
        if not self.normalize or not self.use_stats.get(key, False):
            return x
        if self.norm_mode == 'zscore':
            mean = self.stats[f'{key}_mean'].to(x)
            std  = self.stats[f'{key}_std'].to(x).clamp_min(1e-12)
            return (x-mean)/std
        elif self.norm_mode in ('minmax','minmax_pad'):
            vmin = self.stats[f'{key}_min'].to(x)
            vrng = self.stats[f'{key}_rng'].to(x).clamp_min(1e-12)
            if self.norm_mode=='minmax_pad':
                pad = vrng*self.pad_ratio
                vmin=vmin-pad
                vrng=vrng+2*pad
            return ((x-vmin)/vrng).clamp(0.0,1.0)
        elif self.norm_mode in ('sym_minmax','sym_minmax_pad'):
            if f'{key}_mid' in self.stats and f'{key}_half' in self.stats:
                mid=self.stats[f'{key}_mid'].to(x)
                half=self.stats[f'{key}_half'].to(x).clamp_min(1e-12)
            else:
                vmin=self.stats[f'{key}_min'].to(x)
                vrng=self.stats[f'{key}_rng'].to(x).clamp_min(1e-12)
                mid=vmin+0.5*vrng
                half=0.5*vrng
            if self.norm_mode=='sym_minmax_pad':
                half = half*(1+2*self.pad_ratio)
            return ((x-mid)/half).clamp(-1.0,1.0)
        elif self.norm_mode == 'maxabs':
            if f'{key}_maxabs' in self.stats:
                s=self.stats[f'{key}_maxabs'].to(x).clamp_min(1e-12)
            else:
                vmin=self.stats[f'{key}_min'].to(x)
                vmax=vmin+self.stats[f'{key}_rng'].to(x)
                s=torch.maximum(vmin.abs(), vmax.abs()).clamp_min(1e-12)
            return (x/s).clamp(-1.0,1.0)
        else:
            raise ValueError(f'Unknown norm_mode: {self.norm_mode}')

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        # 1. 解包索引信息
        seq_dir, i, last, flip_flag, tag = self.samples[index]
        meta = self.seq_meta[seq_dir]
        img_name = meta['imgs'][i]
        img_path = os.path.join(seq_dir, img_name)
        
        # 2. 读取图像
        rgb = read_image_rgb(img_path)   # HWC RGB uint8
        H, W = rgb.shape[:2]

        # 3. 读取并计算基础位姿 (Pose)
        vi, ve = meta['pose'][i], meta['pose'][last]
        Ti, Ri = split_pose(vi)
        Te, Re = split_pose(ve)
        
        # 转 Tensor
        Ti = torch.from_numpy(np.asarray(Ti, np.float32))
        Te = torch.from_numpy(np.asarray(Te, np.float32))
        Ri = torch.from_numpy(np.asarray(Ri, np.float32))
        Re = torch.from_numpy(np.asarray(Re, np.float32))
        
        # 计算相对位姿变化 (Ground Truth for Motion)
        dR_1e = torch.from_numpy((Ri.numpy().T @ Re.numpy()).astype(np.float32))
        dT_1e_calc = torch.from_numpy((Ri.numpy().T @ (Te.numpy()-Ti.numpy()))).float()

        # 默认使用计算出的 dT，稍后会被 pixel_dis 覆盖
        dT_1e = dT_1e_calc
        if meta.get('dt1f_valid', None) is not None and bool(meta['dt1f_valid'][i]):
            dT_1e = torch.from_numpy(meta['dt1f'][i].astype(np.float32))

        # 4. 读取 YOLO 信息
        yb = meta['yolo']
        # 注意：在 __init__ 中我们已经保证了进入这里的样本:
        # 要么是 ok=1, 要么是 ok!=1 但 gt_policy!='pixel_dis'
        y_ok = int(yb['ok'][i]) if yb is not None else 0
        pt_x = float(yb['ptx'][i]) if yb is not None else float('nan')
        pt_y = float(yb['pty'][i]) if yb is not None else float('nan')
        bx1, by1, bx2, by2 = [float(yb[k][i]) if yb is not None else float('nan') for k in ('bx1','by1','bx2','by2')]

        # =======================================================
        # 【核心修改区域：平移增强与坐标同步】
        # =======================================================
        dx_applied, dy_applied = 0, 0
        
        # 只有当需要基于视觉距离控制且 YOLO 有效时，才进行相关增强计算
        if self.gt_policy == 'pixel_dis' and y_ok == 1:
            # A. 决定平移量
            if tag == 'base':
                # 原图不平移，保持 dx=0
                pass
            elif self.shift_mode == 'dynamic' and tag.startswith('dyn'):
                dx_applied, dy_applied = self.sample_dx_dy_for_index(seq_dir, i)
            elif self.shift_mode == 'pregen' and tag.startswith('pregen'):
                k = int(tag.replace('pregen',''))
                dx_applied, dy_applied = self.pregen_shifts.get((seq_dir,i,k), (0,0))

            # B. 应用平移到图像 (如果有位移)
            if dx_applied != 0 or dy_applied != 0:
                rgb = translate_image(rgb, dx_applied, dy_applied)

                # 你的特殊抖动逻辑 (back shift)
                dx_applied_back = sample_idx_back(dx_applied, 0.2)
                rgb = translate_image(rgb, dx_applied_back, 0)
                
                # 更新总位移量
                dx_applied = dx_applied + dx_applied_back
                
                # 外部 Mask (遮挡掉移位产生的黑边以及非 ROI 区域)
                # 注意：这里要用更新后的坐标来计算遮挡框
                bbox_after = (bx1+dx_applied, by1+dy_applied, bx2+dx_applied, by2+dy_applied)
                rgb = occlude_outside_bbox(rgb, bbox_after, self.yolo_occ_max, self.yolo_occ_size_min, self.yolo_occ_size_max)

            # C. 【重要】无论是否有位移，都要更新 YOLO 坐标
            # 对于 base 样本，dx=0，pt_x 不变，这是对的
            # 对于 aug 样本，dx!=0，pt_x 随之改变
            pt_x += dx_applied
            pt_y += dy_applied
            # (如果后续需要用到 box，这里也可以更新 bx1 等，但在 current logic 中主要用 pt_x)

        # =======================================================
        # 【核心修改区域：强制重写标签】
        # =======================================================
        if self.gt_policy == 'pixel_dis':
            # 只要能看到血管 (y_ok=1)，就强制使用视觉伺服逻辑计算标签
            # 这里的 pt_x 已经是经过上面 (pt_x += dx_applied) 修正后的最终位置
            if y_ok == 1 and not math.isnan(pt_x):
                # 目标：将 pt_x 移到 ptx_center
                # 假设机械臂坐标系定义：y 方向控制左右移动
                # 误差 = 目标中心 - 当前位置
                # 乘上 scale (像素转毫米)
                dist_y_mm = (- pt_x + self.ptx_center) * self.ptx_scale
                
                # 构造新的标签向量 [x, y, z] -> 这里假设控制 y 轴
                dT_1e = torch.tensor([0.0, dist_y_mm, 0.0], dtype=torch.float32)
                
            # 如果 y_ok=0，保持原有的 dT_1e (这里取决于 __init__ 是否放行了 ok=0 的数据)

        elif self.gt_policy in ('keep_dt1f', 'pose', 'original'):
            pass 
        else:
            raise ValueError(f"Unknown gt_policy: {self.gt_policy}")

        # 5. 欧拉角处理 (保持不变)
        # e = euler_from_rotmat_np(dR_1e.numpy(), order=self.euler_order, degrees=self.euler_degrees)
        # e = np.abs(e)
        # if self.euler_degrees:
        #     e = np.where(e>90, 180-e, e)
        #     e = np.clip(e, 0, 90)
        #     e = e/90.0
        # else:
        #     half_pi = np.pi/2
        #     pi=np.pi
        #     e = np.where(e>half_pi, pi-e, e)
        #     e = np.clip(e, 0, half_pi)
        #     e = e/half_pi
        # e1e = torch.from_numpy(e.astype(np.float32))
        e1e=0.0

        # 6. 图像增强与转换 (Transforms)
        img_chw = torch.from_numpy(rgb).permute(2,0,1).float()/255.0
        self.aug_cfg['out_hw'] = img_chw.shape[-2:]
        
        if self.aug_enable:
            force_flip = flip_flag if self.flip_expand else None
            img_chw, flipped = self.augment_basic(img_chw, force_flip=force_flip)
            # 如果发生了翻转，Y 轴的运动方向也要反转
            if flipped:
                dT_1e[..., 1] = -dT_1e[..., 1]

        if self.transform is not None:
            # 转换为 numpy uint8 供 transform 使用 (如 Normalize)
            t = self.transform((img_chw.clamp(0,1).permute(1,2,0).cpu().numpy()*255).astype(np.uint8))
            img_chw = t if isinstance(t, torch.Tensor) else torch.from_numpy(t).permute(2,0,1).float()/255.0

        # 7. 归一化输出
        Ti_n = self.norm_tensor('T', Ti)
        Te_n = self.norm_tensor('Tend', Te)
        dT_n = self.norm_tensor('dT_1e', dT_1e)

        if self.return_end_image:
            img_end_np = read_image_rgb(os.path.join(seq_dir, meta['imgs'][last]))
            img_end = torch.from_numpy(img_end_np).permute(2,0,1).float()/255.0
            return (img_chw, img_end, Ti_n, Te_n, Ri, Re, dT_n, dR_1e, e1e)
        else:
            return (img_chw, Ti_n, Te_n, Ri, Re, dT_n, dR_1e, e1e)

def save_side_by_side(img_l, img_r, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if isinstance(img_l, torch.Tensor):
        img_l = (img_l.clamp(0,1).permute(1,2,0).cpu().numpy()*255).astype(np.uint8)
    if isinstance(img_r, torch.Tensor):
        img_r = (img_r.clamp(0,1).permute(1,2,0).cpu().numpy()*255).astype(np.uint8)
    cat = cv2.hconcat([
        cv2.cvtColor(img_l, cv2.COLOR_RGB2BGR),
        cv2.cvtColor(img_r, cv2.COLOR_RGB2BGR)
    ])
    cv2.imwrite(path, cat)

class MyCarotidDataset(Dataset):
    def __init__(self,
                 root_dir,
                 split="train",
                 image_interval=1,
                 transform=None,
                 normalize=True,
                 norm_mode="minmax_pad",
                 pad_ratio=0.05,
                 euler_degrees=True,
                 euler_order='zyx',
                 return_end_image=False,
                 include_last=True,
                 flip_expand=False,

                 aug_enable=True,
                 p_flip=0.5,
                 p_crop=0.3,
                 p_elastic=0.10,
                 p_noise=0.5,
                 crop_scale_rangeH=(0.8, 1.0),
                 crop_scale_rangeW=(0.8, 1.0),
                 elastic_alpha_px=(1.0, 3.0),
                 elastic_sigma_px=(3.0, 6.0),
                 noise_std=(0.01, 0.03),

                 # 动静脉 shift 增强
                 shift_mode='dynamic',  # 设为 'none' 关闭位移增强
                 shift_probe_h=1.0,
                 shift_probe_v=1.0,
                 yolo_max_dx=-1,
                 yolo_max_dy=-1,
                 yolo_occ_max=5,
                 yolo_occ_size_min=0.06,
                 yolo_occ_size_max=0.12,
                 dynamic_k=8,                    
                 pregen_k=8,                     

                 # 策略控制
                 # 'pixel_dis': 强制利用 YOLO 计算目标距离 (需要 YOLO ok=1)
                 # 'pose' / 'original': 使用原始机械臂位姿差值 (忽略 YOLO 状态)
                 gt_policy='pixel_dis',          
                 ptx_only = 1,                   
                 ptx_center=300.0,
                 ptx_scale=6.25e-5,
                 return_dict=False):
                 
        super().__init__()
        self.root_dir = os.path.abspath(root_dir)
        self.split = split
        self.image_interval = max(1, int(image_interval))
        self.transform = transform
        self.normalize = bool(normalize)
        self.norm_mode = norm_mode
        self.pad_ratio = float(pad_ratio)
        self.euler_degrees = bool(euler_degrees)
        self.euler_order = euler_order
        self.return_end_image = bool(return_end_image)
        self.include_last = bool(include_last)
        self.flip_expand = bool(flip_expand)
        self.return_dict = bool(return_dict)


        self.aug_enable = bool(aug_enable)
        self.aug_cfg = {
            "out_hw": None,
            "p_flip": float(p_flip),
            "p_crop": float(p_crop),
            "p_elastic": float(p_elastic),
            "p_noise": float(p_noise),
            "crop_scale_rangeH": crop_scale_rangeH,
            "crop_scale_rangeW": crop_scale_rangeW,
            "elastic_alpha_px": elastic_alpha_px,
            "elastic_sigma_px": elastic_sigma_px,
            "noise_std": noise_std,
        }

        self.shift_mode = str(shift_mode)
        self.shift_probe_h = float(shift_probe_h)
        self.shift_probe_v = float(shift_probe_v)
        self.yolo_max_dx = int(yolo_max_dx)
        self.yolo_max_dy = int(yolo_max_dy)
        self.yolo_occ_max = int(yolo_occ_max)
        self.yolo_occ_size_min = float(yolo_occ_size_min)
        self.yolo_occ_size_max = float(yolo_occ_size_max)
        self.dynamic_k = max(0, int(dynamic_k))
        self.pregen_k = max(0, int(pregen_k))

        self.gt_policy = str(gt_policy)
        self.ptx_only = bool(ptx_only)        
        self.ptx_center = float(ptx_center)
        self.ptx_scale = float(ptx_scale)

        self.stats = self.load_norm_stats() if self.normalize else {}
        keys = ("T", "Tend", "dT_1e")
        self.use_stats = {k: all(self.stats.get(f"{k}_"+s) is not None for s in ("mean","std","min","rng"))
                          for k in keys} if self.normalize else {k: False for k in keys}

        self.seq_meta = {}
        self.samples = []        # (seq_dir, idx, last, flip_flag, aug_tag)
        split_dir = os.path.join(self.root_dir, self.split)
        if not os.path.isdir(split_dir):
            raise FileNotFoundError(f"split dir not found: {split_dir}")

        seq_names = sorted([d for d in os.listdir(split_dir) if os.path.isdir(os.path.join(split_dir, d))])
        
        for seq in seq_names:
            sd = os.path.join(split_dir, seq)
            csv_path = None
            for fn in os.listdir(sd):
                if fn.lower().endswith('.csv'):
                    csv_path = os.path.join(sd, fn)
                    break
            if csv_path is None:
                continue

            imgs, poses, dt1e, dt1e_valid, dt1f, dt1f_valid, yolo_block = self.read_csv_pose(csv_path, sd)
            last = len(imgs) - 1
            self.seq_meta[sd] = {
                "imgs": imgs,
                "pose": poses,
                "last": last,
                "dt1e": dt1e,
                "dt1e_valid": dt1e_valid,
                "dt1f": dt1f,
                "dt1f_valid": dt1f_valid,
                "yolo": yolo_block, 
            }

            end_i = last if self.include_last else last - 1
            
            # --- 核心修改区域：样本生成循环 ---
            for i in range(0, end_i + 1, self.image_interval):
                # 1. 获取 YOLO 状态
                ok = int(yolo_block['ok'][i]) if yolo_block is not None else 0

                # 2. 【修改】根据 gt_policy 决定是否强制过滤
                # 如果 gt_policy 是 'pixel_dis'，则必须要求 YOLO 检测到血管 (ok=1)
                # 如果 gt_policy 是其他 (如 'pose', 'original')，则不做硬性过滤
                if self.gt_policy == 'pixel_dis' and ok != 1:
                    continue
                
                # 3. 添加原始图片样本 (Base Sample)
                base = (sd, i, last, False, 'base')
                self.samples.append(base)

                if self.flip_expand:
                    self.samples.append((sd, i, last, True, 'base'))

                # 4. 添加动态平移扩充样本 (Dynamic Shift Samples)
                # 【修改】增加 and ok == 1 条件
                # 只有 YOLO 正常检测到，才有“目标点”可供我们进行平移增强
                if self.shift_mode == 'dynamic' and self.dynamic_k > 0 and ok == 1:
                    for k in range(0, self.dynamic_k):
                        self.samples.append((sd, i, last, False, f'dyn{k}'))

        self.env_y1, self.env_y2 = self.compute_global_env_y()

        # 预生成模式的处理
        self.pregen_shifts = {}
        if self.gt_policy == 'pixel_dis' and self.shift_mode == 'pregen' and self.pregen_k > 0:
            aug_list = []
            for idx, s in enumerate(list(self.samples)):
                sd, i, last, flip_flag, tag = s
                
                # 获取该帧 YOLO 状态
                yb = self.seq_meta[sd]['yolo']
                ok = int(yb['ok'][i]) if yb is not None else 0

                # 只有 YOLO 正常才生成预生成 shift 样本
                if ok == 1:
                    for k in range(self.pregen_k):
                        dx, dy = self.sample_dx_dy_for_index(sd, i)
                        self.pregen_shifts[(sd, i, k)] = (dx, dy)
                        aug_list.append((sd, i, last, flip_flag, f'pregen{k}'))
            self.samples.extend(aug_list)


    def read_csv_pose(self, csv_path, img_dir):
        def read_dt_triplet(row, keys):
            vals = []
            success = True
            for k in keys:
                if k in row and row[k] not in (None, ""):
                    try:
                        vals.append(float(row[k]) * POS_SCALE)
                    except Exception:
                        success = False
                        break
                else:
                    success = False
                    break
            if success and len(vals) == 3:
                return vals, True
            return [float('nan')]*3, False

        imgs, poses = [], []
        dt1e_list, dt1e_valid = [], []
        dt1f_list, dt1f_valid = [], []

        y_ok, y_ptx, y_pty = [], [], []
        y_bx1, y_by1, y_bx2, y_by2 = [], [], [], []

        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                img_name = row.get('img')
                if not img_name:
                    continue
                if not is_image_file(os.path.join(img_dir, img_name)):
                    # 尝试不同扩展名
                    base = os.path.splitext(img_name)[0]
                    found = None
                    for ext in IMAGE_EXTS:
                        cand = os.path.join(img_dir, base+ext)
                        if os.path.isfile(cand):
                            found = cand
                            img_name = os.path.basename(cand)
                            break
                    if not found:
                        continue

                # 位姿 3 + 9
                Txyz = [float(row[k]) * POS_SCALE for k in ("x","y","z")]
                R9 = [float(row[k]) for k in ("r00","r01","r02","r10","r11","r12","r20","r21","r22")]

                v1e, s1e = read_dt_triplet(row, ("dx1e","dy1e","dz1e"))
                v1f, s1f = read_dt_triplet(row, ("dx1f","dy1f","dz1f"))


                ok = safe_int(row.get('yolo_ok'), 0) # 建议开启 safe_int 读取
                ptx = safe_float(row.get('yolo_pt_x'), 0)
                pty = safe_float(row.get('yolo_pt_y'), 0)
                bx1 = safe_float(row.get('yolo_big_x1'), 0)
                by1 = safe_float(row.get('yolo_big_y1'), 0)
                bx2 = safe_float(row.get('yolo_big_x2'), 0)
                by2 = safe_float(row.get('yolo_big_y2'), 0)


                imgs.append(img_name)
                poses.append(Txyz + R9)
                dt1e_list.append(v1e)
                dt1e_valid.append(s1e)
                dt1f_list.append(v1f)
                dt1f_valid.append(s1f)

                y_ok.append(ok)
                y_ptx.append(ptx)
                y_pty.append(pty)
                y_bx1.append(bx1)
                y_by1.append(by1)
                y_bx2.append(bx2)
                y_by2.append(by2)

        if not imgs:
            raise ValueError(f"{csv_path} 有效行为空")

        poses  = np.asarray(poses,  np.float32)
        dt1e   = np.asarray(dt1e_list, np.float32)
        dt1f   = np.asarray(dt1f_list, np.float32)
        v1e_mk = np.asarray(dt1e_valid, np.bool_)
        v1f_mk = np.asarray(dt1f_valid, np.bool_)

        yolo_block = {
            'ok':   np.asarray(y_ok,  np.int32),
            'ptx':  np.asarray(y_ptx, np.float32),
            'pty':  np.asarray(y_pty, np.float32),
            'bx1':  np.asarray(y_bx1, np.float32),
            'by1':  np.asarray(y_by1, np.float32),
            'bx2':  np.asarray(y_bx2, np.float32),
            'by2':  np.asarray(y_by2, np.float32),
        }

        return imgs, poses, dt1e, v1e_mk, dt1f, v1f_mk, yolo_block


    def load_norm_stats(self):
        fname = "norm_stats_all.pt"
        pt = os.path.join(self.root_dir, fname)
        js = os.path.splitext(pt)[0] + '.json'
        if os.path.isfile(pt):
            return torch.load(pt, map_location='cpu')
        if os.path.isfile(js):
            with open(js, 'r') as f:
                jd = json.load(f)
            return {k: torch.tensor(v) for k, v in jd.items()}
        # print(f"[normalize disabled] stats file not found: '{pt}' or '{js}'")
        self.normalize = False
        return {}

    def compute_global_env_y(self):
        y1_all, y2_all = [], []
        for sd, meta in self.seq_meta.items():
            yb = meta['yolo']
            ok = (yb['ok'] == 1)
            if np.any(ok):
                y1 = yb['by1'][ok]
                y2 = yb['by2'][ok]
                if y1.size and y2.size:
                    y1_all.append(np.nanmin(y1))
                    y2_all.append(np.nanmax(y2))
        if len(y1_all) == 0:
            return None, None
        return float(np.nanmin(y1_all)), float(np.nanmax(y2_all))

    def elastic_deform_numpy_v2(self, img_chw):
        mode = 'gaussian'
        strength = 'mild'
        alpha_px = self.aug_cfg['elastic_alpha_px']
        sigma_px = self.aug_cfg['elastic_sigma_px']
        img = img_chw.detach().cpu().numpy().transpose(1,2,0)
        H,W = img.shape[:2]
        a = np.random.uniform(*alpha_px)
        s = np.random.uniform(*sigma_px)
        dx = np.random.randn(H,W).astype(np.float32)
        dy = np.random.randn(H,W).astype(np.float32)
        ksize = int(2*round(3*s)+1)
        dx = cv2.GaussianBlur(dx,(ksize,ksize),sigmaX=s)
        dy = cv2.GaussianBlur(dy,(ksize,ksize),sigmaX=s)
        dx *= a
        dy *= a
        xx,yy = np.meshgrid(np.arange(W,dtype=np.float32), np.arange(H,dtype=np.float32))
        map_x = (xx+dx).astype(np.float32)
        map_y=(yy+dy).astype(np.float32)
        img_def = cv2.remap(img,map_x,map_y,interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT101)
        out = torch.from_numpy(img_def.transpose(2,0,1)).to(img_chw.dtype)
        return out

    def augment_basic(self, img_chw, force_flip=None):

        cfg = self.aug_cfg
        out_hw = cfg['out_hw']
        p_flip = cfg['p_flip']
        p_crop = cfg['p_crop']
        p_elastic = cfg['p_elastic']
        p_noise = cfg['p_noise']
        crop_scale_rangeH = cfg['crop_scale_rangeH']
        crop_scale_rangeW = cfg['crop_scale_rangeW']
        noise_std = cfg['noise_std']

        C,H,W = img_chw.shape
        flipped = False
        # flip
        do_flip = bool(force_flip) if force_flip is not None else (random.random()<p_flip)
        if do_flip:
            img_chw = torch.flip(img_chw, dims=[-1])
            flipped=True

        # crop
        if random.random() < p_crop:
            sH = random.uniform(*crop_scale_rangeH)
            sW = random.uniform(*crop_scale_rangeW)
            ch = max(1,int(round(H*sH)))
            cw = max(1,int(round(W*sW)))
            y0 = 0 if H==ch else random.randint(0,H-ch)
            x0 = 0 if W==cw else random.randint(0,W-cw)
            img_chw = img_chw[:, y0:y0+ch, x0:x0+cw]

        # elastic
        if random.random() < p_elastic:
            img_chw = self.elastic_deform_numpy_v2(img_chw)

        # resize 回到 out_hw
        if out_hw is not None and img_chw.shape[-2:] != tuple(out_hw):
            img_chw = F.interpolate(img_chw.unsqueeze(0), size=out_hw, mode='bilinear', align_corners=False).squeeze(0)

        # noise
        if random.random() < p_noise:
            std = random.uniform(*noise_std)
            noise = torch.randn_like(img_chw) * std
            img_chw = torch.clamp(img_chw + noise, 0.0, 1.0)
        return img_chw, flipped



    def sample_dx_dy_for_index(self, seq_dir, i):
        yb = self.seq_meta[seq_dir]['yolo']
        ptx = float(yb['ptx'][i])
        pty = float(yb['pty'][i])
        x1 = float(yb['bx1'][i])
        y1 = float(yb['by1'][i])
        x2 = float(yb['bx2'][i])
        y2 = float(yb['by2'][i])
        img_path = os.path.join(seq_dir, self.seq_meta[seq_dir]['imgs'][i])
        img = read_image_rgb(img_path)
        h,w = img.shape[:2]

        # 水平范围 关键点不出界
        lo_dx = int(math.ceil(-ptx))
        hi_dx = int(math.floor((w-1)-ptx))
        if self.yolo_max_dx > 0:
            lo_dx = max(lo_dx, -self.yolo_max_dx)
            hi_dx = min(hi_dx, self.yolo_max_dx)
        dx = 0
        if self.shift_probe_h>0 and lo_dx<=hi_dx and random.random()<self.shift_probe_h:
            dx = random.randint(lo_dx, hi_dx)
        # 纵向范围 关键点不出界 ∩ 活动范围约束
        dy = 0
        if self.shift_probe_v>0 and random.random()<self.shift_probe_v:
            # 关键点不出界
            r_key = (int(math.ceil(-pty)), int(math.floor((h-1)-pty)))
            ranges = [r_key]
            # 活动范围
            if self.env_y1 is not None and self.env_y2 is not None:
                r_env = (int(math.ceil(self.env_y1 - y1)), int(math.floor(self.env_y2 - y2)))
                ranges.append(r_env)
            # 交集
            lo, hi = ranges[0]
            for r in ranges[1:]:
                lo = max(lo, r[0])
                hi = min(hi, r[1])
            if self.yolo_max_dy > 0:
                lo = max(lo, -self.yolo_max_dy)
                hi = min(hi, self.yolo_max_dy)
            if lo <= hi:
                dy = random.randint(lo, hi)
        return dx, dy

    def norm_tensor(self, key, x):
        if not self.normalize or not self.use_stats.get(key, False):
            return x
        if self.norm_mode == 'zscore':
            mean = self.stats[f'{key}_mean'].to(x)
            std  = self.stats[f'{key}_std'].to(x).clamp_min(1e-12)
            return (x-mean)/std
        elif self.norm_mode in ('minmax','minmax_pad'):
            vmin = self.stats[f'{key}_min'].to(x)
            vrng = self.stats[f'{key}_rng'].to(x).clamp_min(1e-12)
            if self.norm_mode=='minmax_pad':
                pad = vrng*self.pad_ratio
                vmin=vmin-pad
                vrng=vrng+2*pad
            return ((x-vmin)/vrng).clamp(0.0,1.0)
        elif self.norm_mode in ('sym_minmax','sym_minmax_pad'):
            if f'{key}_mid' in self.stats and f'{key}_half' in self.stats:
                mid=self.stats[f'{key}_mid'].to(x)
                half=self.stats[f'{key}_half'].to(x).clamp_min(1e-12)
            else:
                vmin=self.stats[f'{key}_min'].to(x)
                vrng=self.stats[f'{key}_rng'].to(x).clamp_min(1e-12)
                mid=vmin+0.5*vrng
                half=0.5*vrng
            if self.norm_mode=='sym_minmax_pad':
                half = half*(1+2*self.pad_ratio)
            return ((x-mid)/half).clamp(-1.0,1.0)
        elif self.norm_mode == 'maxabs':
            if f'{key}_maxabs' in self.stats:
                s=self.stats[f'{key}_maxabs'].to(x).clamp_min(1e-12)
            else:
                vmin=self.stats[f'{key}_min'].to(x)
                vmax=vmin+self.stats[f'{key}_rng'].to(x)
                s=torch.maximum(vmin.abs(), vmax.abs()).clamp_min(1e-12)
            return (x/s).clamp(-1.0,1.0)
        else:
            raise ValueError(f'Unknown norm_mode: {self.norm_mode}')

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        # 1. 解包索引信息
        seq_dir, i, last, flip_flag, tag = self.samples[index]
        meta = self.seq_meta[seq_dir]
        img_name = meta['imgs'][i]
        img_path = os.path.join(seq_dir, img_name)
        
        # 2. 读取图像
        rgb = read_image_rgb(img_path)   # HWC RGB uint8
        H, W = rgb.shape[:2]

        # 3. 读取并计算基础位姿 (Pose)
        vi, ve = meta['pose'][i], meta['pose'][last]
        Ti, Ri = split_pose(vi)
        Te, Re = split_pose(ve)
        
        # 转 Tensor
        Ti = torch.from_numpy(np.asarray(Ti, np.float32))
        Te = torch.from_numpy(np.asarray(Te, np.float32))
        Ri = torch.from_numpy(np.asarray(Ri, np.float32))
        Re = torch.from_numpy(np.asarray(Re, np.float32))
        
        # 计算相对位姿变化 (Ground Truth for Motion)
        dR_1e = torch.from_numpy((Ri.numpy().T @ Re.numpy()).astype(np.float32))
        dT_1e_calc = torch.from_numpy((Ri.numpy().T @ (Te.numpy()-Ti.numpy()))).float()

        # 默认使用计算出的 dT，稍后会被 pixel_dis 覆盖
        dT_1e = dT_1e_calc
        if meta.get('dt1f_valid', None) is not None and bool(meta['dt1f_valid'][i]):
            dT_1e = torch.from_numpy(meta['dt1f'][i].astype(np.float32))

        # 4. 读取 YOLO 信息
        yb = meta['yolo']
        # 注意：在 __init__ 中我们已经保证了进入这里的样本:
        # 要么是 ok=1, 要么是 ok!=1 但 gt_policy!='pixel_dis'
        y_ok = int(yb['ok'][i]) if yb is not None else 0
        pt_x = float(yb['ptx'][i]) if yb is not None else float('nan')
        pt_y = float(yb['pty'][i]) if yb is not None else float('nan')
        bx1, by1, bx2, by2 = [float(yb[k][i]) if yb is not None else float('nan') for k in ('bx1','by1','bx2','by2')]

        # =======================================================
        # 【核心修改区域：平移增强与坐标同步】
        # =======================================================
        dx_applied, dy_applied = 0, 0
        
        # 只有当需要基于视觉距离控制且 YOLO 有效时，才进行相关增强计算
        if self.gt_policy == 'pixel_dis' and y_ok == 1:
            # A. 决定平移量
            if tag == 'base':
                # 原图不平移，保持 dx=0
                pass
            elif self.shift_mode == 'dynamic' and tag.startswith('dyn'):
                dx_applied, dy_applied = self.sample_dx_dy_for_index(seq_dir, i)
            elif self.shift_mode == 'pregen' and tag.startswith('pregen'):
                k = int(tag.replace('pregen',''))
                dx_applied, dy_applied = self.pregen_shifts.get((seq_dir,i,k), (0,0))

            # B. 应用平移到图像 (如果有位移)
            if dx_applied != 0 or dy_applied != 0:
                rgb = translate_image(rgb, dx_applied, dy_applied)

                # 你的特殊抖动逻辑 (back shift)
                dx_applied_back = sample_idx_back(dx_applied, 0.2)
                rgb = translate_image(rgb, dx_applied_back, 0)
                
                # 更新总位移量
                dx_applied = dx_applied + dx_applied_back
                
                # 外部 Mask (遮挡掉移位产生的黑边以及非 ROI 区域)
                # 注意：这里要用更新后的坐标来计算遮挡框
                bbox_after = (bx1+dx_applied, by1+dy_applied, bx2+dx_applied, by2+dy_applied)
                rgb = occlude_outside_bbox(rgb, bbox_after, self.yolo_occ_max, self.yolo_occ_size_min, self.yolo_occ_size_max)

            # C. 【重要】无论是否有位移，都要更新 YOLO 坐标
            # 对于 base 样本，dx=0，pt_x 不变，这是对的
            # 对于 aug 样本，dx!=0，pt_x 随之改变
            pt_x += dx_applied
            pt_y += dy_applied
            # (如果后续需要用到 box，这里也可以更新 bx1 等，但在 current logic 中主要用 pt_x)

        # =======================================================
        # 【核心修改区域：强制重写标签】
        # =======================================================
        if self.gt_policy == 'pixel_dis':
            # 只要能看到血管 (y_ok=1)，就强制使用视觉伺服逻辑计算标签
            # 这里的 pt_x 已经是经过上面 (pt_x += dx_applied) 修正后的最终位置
            if y_ok == 1 and not math.isnan(pt_x):
                # 目标：将 pt_x 移到 ptx_center
                # 假设机械臂坐标系定义：y 方向控制左右移动
                # 误差 = 目标中心 - 当前位置
                # 乘上 scale (像素转毫米)
                dist_y_mm = (- pt_x + self.ptx_center) * self.ptx_scale
                
                # 构造新的标签向量 [x, y, z] -> 这里假设控制 y 轴
                dT_1e = torch.tensor([0.0, dist_y_mm, 0.0], dtype=torch.float32)
                
            # 如果 y_ok=0，保持原有的 dT_1e (这里取决于 __init__ 是否放行了 ok=0 的数据)

        elif self.gt_policy in ('keep_dt1f', 'pose', 'original'):
            pass 
        else:
            raise ValueError(f"Unknown gt_policy: {self.gt_policy}")

        # 5. 欧拉角处理 (保持不变)
        e = euler_from_rotmat_np(dR_1e.numpy(), order=self.euler_order, degrees=self.euler_degrees)
        e = np.abs(e)
        if self.euler_degrees:
            e = np.where(e>90, 180-e, e)
            e = np.clip(e, 0, 90)
            e = e/90.0
        else:
            half_pi = np.pi/2
            pi=np.pi
            e = np.where(e>half_pi, pi-e, e)
            e = np.clip(e, 0, half_pi)
            e = e/half_pi
        e1e = torch.from_numpy(e.astype(np.float32))

        # 6. 图像增强与转换 (Transforms)
        img_chw = torch.from_numpy(rgb).permute(2,0,1).float()/255.0
        self.aug_cfg['out_hw'] = img_chw.shape[-2:]
        
        if self.aug_enable:
            force_flip = flip_flag if self.flip_expand else None
            img_chw, flipped = self.augment_basic(img_chw, force_flip=force_flip)
            # 如果发生了翻转，Y 轴的运动方向也要反转
            if flipped:
                dT_1e[..., 1] = -dT_1e[..., 1]

        if self.transform is not None:
            # 转换为 numpy uint8 供 transform 使用 (如 Normalize)
            t = self.transform((img_chw.clamp(0,1).permute(1,2,0).cpu().numpy()*255).astype(np.uint8))
            img_chw = t if isinstance(t, torch.Tensor) else torch.from_numpy(t).permute(2,0,1).float()/255.0

        # 7. 归一化输出
        Ti_n = self.norm_tensor('T', Ti)
        Te_n = self.norm_tensor('Tend', Te)
        dT_n = self.norm_tensor('dT_1e', dT_1e)

        if self.return_end_image:
            img_end_np = read_image_rgb(os.path.join(seq_dir, meta['imgs'][last]))
            img_end = torch.from_numpy(img_end_np).permute(2,0,1).float()/255.0
            return (img_chw, img_end, Ti_n, Te_n, Ri, Re, dT_n, dR_1e, e1e)
        else:
            return (img_chw, Ti_n, Te_n, Ri, Re, dT_n, dR_1e, e1e)

def save_side_by_side(img_l, img_r, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if isinstance(img_l, torch.Tensor):
        img_l = (img_l.clamp(0,1).permute(1,2,0).cpu().numpy()*255).astype(np.uint8)
    if isinstance(img_r, torch.Tensor):
        img_r = (img_r.clamp(0,1).permute(1,2,0).cpu().numpy()*255).astype(np.uint8)
    cat = cv2.hconcat([
        cv2.cvtColor(img_l, cv2.COLOR_RGB2BGR),
        cv2.cvtColor(img_r, cv2.COLOR_RGB2BGR)
    ])
    cv2.imwrite(path, cat)