import os
import shutil
import argparse
import re
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

# ==========================================
# 1. 基础工具函数 (保持稳健的时间解析)
# ==========================================

def ts_str_to_ms(ts: str) -> float:
    """
    兼容两种格式的时间戳解析，返回秒(float)或毫秒
    为了和参考程序一致，这里我们返回 秒 (float)，方便计算
    """
    ts = ts.strip()
    dt = None
    try:
        # 标准格式: 2025-06-25 17:41:22.951
        dt = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f")
    except ValueError:
        try:
            # 紧凑格式: 20250625_174122951
            if "_" in ts:
                parts = ts.split("_")
                # 只有当后面部分足够长才解析
                if len(parts) > 1 and len(parts[1]) >= 6:
                    date_part = parts[0]
                    time_part = parts[1]
                    # 补齐毫秒，防止位数不足
                    if len(time_part) < 6: 
                        time_part = time_part.ljust(6, '0')
                    
                    dt_str = date_part + time_part[:6] # YYYYMMDDHHMMSS
                    ms_part = time_part[6:] if len(time_part) > 6 else "0"
                    
                    dt = datetime.strptime(dt_str, "%Y%m%d%H%M%S")
                    dt = dt.replace(microsecond=int(ms_part) * 1000)
        except ValueError:
            pass
            
    if dt is None:
        # 最后的兜底尝试
        try:
             dt = datetime.strptime(ts, "%Y%m%d_%H%M%S%f")
        except:
             raise ValueError(f"Unknown timestamp format: {ts}")

    return dt.timestamp()

def read_txt(txt_path: Path):
    """读取 txt 姿态数据"""
    ts_list, vecs_list, raw_lines = [], [], []
    
    with open(txt_path, "r") as f:
        lines = f.readlines()
        
    for line in lines:
        line = line.strip()
        if not line: continue
        parts = line.split()
        
        try:
            # 自动判断列结构
            if len(parts) > 2 and "-" in parts[0]:
                ts_str = f"{parts[0]} {parts[1]}"
                nums_start = 2
            else:
                ts_str = parts[0]
                nums_start = 1
            
            ts_sec = ts_str_to_ms(ts_str)
            
            # 解析数字
            nums_str = parts[nums_start:]
            curr_vec = []
            for s in nums_str:
                s = s.replace(",", "")
                if s: curr_vec.append(float(s))
            
            # 确保数据维度足够 (Pos(3) + Rot(9) + Force(3) = 15)
            # 参考程序是按 15 列读的
            if len(curr_vec) >= 15:
                ts_list.append(ts_sec)
                vecs_list.append(curr_vec)
                raw_lines.append(line)
                
        except (ValueError, IndexError):
            continue

    if not ts_list:
        return None, None, None

    return np.array(ts_list, dtype=np.float64), np.array(vecs_list, dtype=np.float64), raw_lines

# ==========================================
# 2. 核心数学逻辑 (完全依照参考程序)
# ==========================================

def relative_pose(pos1, rot1, pos2, rot2):
    """
    计算从 pose1 到 pose2 的相对变换 (在 pose1 的局部坐标系下)
    参考程序逻辑：
    t_rel = R1.T @ (pos2 - pos1)
    R_rel = R1.T @ R2
    """
    R1 = np.array(rot1)
    R2 = np.array(rot2)
    
    # 局部坐标系下的旋转差
    R_rel = R1.T @ R2
    # 局部坐标系下的位移差
    t_rel = R1.T @ (np.array(pos2) - np.array(pos1))
    
    # 转为旋转向量
    rvec_rel = R.from_matrix(R_rel).as_rotvec()
    
    return np.concatenate([t_rel, rvec_rel])

def geodesic_angle(R_curr, R_target):
    """用于降采样的角度距离计算"""
    Rt = np.matmul(R_curr.transpose(0, 2, 1), R_target)
    trace = np.trace(Rt, axis1=-2, axis2=-1)
    cos_th = np.clip((trace - 1) / 2, -1.0, 1.0)
    return np.arccos(cos_th)

# ==========================================
# 3. 序列处理主逻辑
# ==========================================

def process_sequence(seq_dir, dst_seq_dir, angle_step_deg=1.0):
    txt_files = list(seq_dir.glob("*.txt"))
    img_files = sorted([p for p in seq_dir.iterdir() if p.suffix.lower() in [".jpg", ".png", ".jpeg"]])
    
    if not txt_files or not img_files:
        return

    # 1. 读取数据
    # ts_arr 是秒(float), vecs 是原始数值
    ts_arr, vecs, raw_lines = read_txt(txt_files[0])
    if ts_arr is None:
        return

    # 注意：参考程序没有 *100 的操作，这里保持原始数据读取
    # 如果你的 txt 里单位是米，这里读出来就是米。
    
    # 2. 降采样计算 (决定保留哪些帧)
    poses_R_mat = vecs[:, 3:12].reshape(-1, 3, 3)
    
    # 计算相对于最后一帧的角度距离
    dists = geodesic_angle(poses_R_mat, poses_R_mat[-1])
    D_deg = np.degrees(dists[0])
    
    targets_deg = np.arange(0.0, D_deg + 1e-6, angle_step_deg)
    targets_rad = np.radians(targets_deg)
    
    idx_keep = []
    for t in targets_rad:
        idx = np.abs(dists - t).argmin()
        idx_keep.append(idx)
    idx_keep = np.unique(idx_keep)
    idx_keep.sort()
    
    # 3. 准备输出
    dst_seq_dir.mkdir(parents=True, exist_ok=True)
    
    # 建立 图片时间戳 -> 图片路径 的映射
    img_ts_map = {}
    img_ts_keys = []
    for img_path in img_files:
        try:
            t = ts_str_to_ms(img_path.stem)
            img_ts_map[t] = img_path
            img_ts_keys.append(t)
        except:
            pass
    img_ts_arr = np.array(img_ts_keys)
    if len(img_ts_arr) == 0: return

    rows = []
    kept_raw_indices = [] # 用于最后生成 txt

    # 4. 遍历保留帧，生成“当前帧 -> 下一帧”的数据对
    # 参考程序逻辑是 process(i, i+1)，这里我们在降采样后的序列中做同样的事
    for k in range(len(idx_keep) - 1):
        idx_curr = idx_keep[k]
        idx_next = idx_keep[k+1]

        # 提取当前帧数据
        pos1 = vecs[idx_curr, 0:3]
        rot1 = vecs[idx_curr, 3:12].reshape(3, 3)
        force1 = vecs[idx_curr, 12:15]

        # 提取下一帧数据
        pos2 = vecs[idx_next, 0:3]
        rot2 = vecs[idx_next, 3:12].reshape(3, 3)
        force2 = vecs[idx_next, 12:15]

        # === 筛选逻辑 (参考程序核心) ===
        # 如果当前或下一时刻的 Z轴力 > -1.0 (没用力按)，则跳过
        if force1[2] > -1.0 or force2[2] > -1.0:
            continue

        # === 计算逻辑 (参考程序核心) ===
        # 1. 绝对姿态用 Euler XYZ (参考程序: euler1 = R...as_euler('xyz'))
        euler1 = R.from_matrix(rot1).as_euler('xyz')

        # 2. 相对 Delta (Local Frame)
        delta = relative_pose(pos1, rot1, pos2, rot2) # [dx, dy, dz, drx, dry, drz]

        # 3. 匹配图片
        # 找到 idx_curr 对应时间戳最近的图片
        ts_val = ts_arr[idx_curr]
        closest_img_idx = np.abs(img_ts_arr - ts_val).argmin()
        closest_ts = img_ts_arr[closest_img_idx]
        
        # 可选：检查时间差是否过大 (例如 > 0.1s)
        # if abs(closest_ts - ts_val) > 0.1: continue

        src_img_path = img_ts_map[closest_ts]
        img_name = src_img_path.name
        
        # 复制图片
        shutil.copy2(src_img_path, dst_seq_dir / img_name)
        
        # 收集 CSV 行
        # 列顺序: ['img', 'x', 'y', 'z', 'rx', 'ry', 'rz', 'dx', 'dy', 'dz', 'drx', 'dry', 'drz', 'fx', 'fy', 'fz']
        row = {
            'img': img_name,
            'x': pos1[0], 'y': pos1[1], 'z': pos1[2],
            'rx': euler1[0], 'ry': euler1[1], 'rz': euler1[2], # Euler XYZ
            'dx': delta[0], 'dy': delta[1], 'dz': delta[2],    # Local Translation
            'drx': delta[3], 'dry': delta[4], 'drz': delta[5], # Local Rotation Vector
            'fx': force1[0], 'fy': force1[1], 'fz': force1[2]
        }
        rows.append(row)
        kept_raw_indices.append(idx_curr)

    # 5. 保存 TXT (对应保留下来的帧)
    if kept_raw_indices:
        with open(dst_seq_dir / txt_files[0].name, "w") as f:
            for idx in kept_raw_indices:
                f.write(raw_lines[idx] + "\n")

    # 6. 保存 CSV
    if rows:
        df = pd.DataFrame(rows)
        cols_order = ['img', 'x', 'y', 'z', 'rx', 'ry', 'rz', 
                      'dx', 'dy', 'dz', 'drx', 'dry', 'drz', 
                      'fx', 'fy', 'fz']
        # 确保列存在
        final_cols = [c for c in cols_order if c in df.columns]
        df = df[final_cols]
        
        csv_name = f"pose.csv"
        csv_path = dst_seq_dir / csv_name
        df.to_csv(csv_path, index=False)
        print(f"[{seq_dir.name}] Saved {len(df)} samples to {csv_name}")
    else:
        print(f"[{seq_dir.name}] No valid samples after filtering (Force > -1.0).")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--src_root", type=str, required=True, help="原始数据根目录")
    parser.add_argument("--dst_root", type=str, required=True, help="输出数据根目录")
    parser.add_argument("--angle_step", type=float, default=1.0, help="降采样角度步长")
    args = parser.parse_args()

    src_root = Path(args.src_root)
    dst_root = Path(args.dst_root)
    dst_root.mkdir(parents=True, exist_ok=True)
    
    seq_dirs = sorted([p for p in src_root.iterdir() if p.is_dir()])
    print(f"Found {len(seq_dirs)} sequences.")

    for seq_dir in tqdm(seq_dirs, desc="Processing"):
        if not list(seq_dir.glob("*.txt")):
            continue
            
        dst_seq_dir = dst_root / seq_dir.name
        process_sequence(seq_dir, dst_seq_dir, args.angle_step)

    print("\n[Done] All sequences processed.")

if __name__ == "__main__":
    main()