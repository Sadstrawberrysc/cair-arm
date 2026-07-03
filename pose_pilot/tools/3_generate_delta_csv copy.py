import os
import argparse
from glob import glob
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

def process_csv(csv_path):
    # 1. 读取 CSV
    df = pd.read_csv(csv_path)
    if len(df) == 0:
        return

    # 提取基础数据
    # 假设 rx, ry, rz 是之前脚本生成的欧拉角 (Euler XYZ, 弧度)
    pos = df[['x', 'y', 'z']].values
    eulers = df[['rx', 'ry', 'rz']].values
    
    # 2. 计算旋转矩阵 (Task 1: r00-r22)
    # 之前的脚本是用 as_euler('xyz') 保存的，所以这里用 'xyz' 还原
    r_objs = R.from_euler('xyz', eulers)
    r_mats = r_objs.as_matrix() # shape: (N, 3, 3)
    
    # 将 (N, 3, 3) 展平为 (N, 9) -> r00, r01, ... r22
    r_flat = r_mats.reshape(-1, 9)
    
    # 3. 计算 "当前 -> 结尾" 的相对变换 (Task 2: 1e后缀)
    # 获取最后一帧 (End Frame) 的数据
    pos_end = pos[-1]
    r_end_obj = r_objs[-1] # 单个 Rotation 对象
    
    # --- 计算相对位置 (Translation) ---
    # 公式: t_rel = R_curr^T * (P_end - P_curr)
    # 含义: 世界坐标系下的位移差，投影到当前帧的局部坐标系
    delta_pos_global = pos_end - pos  # (N, 3) 广播减法
    
    # 使用 apply(inverse=True) 等价于 乘转置矩阵
    t_1e = r_objs.apply(delta_pos_global, inverse=True)
    
    # --- 计算相对旋转 (Rotation) ---
    # 公式: R_rel = R_curr^T * R_end
    # scipy 支持批处理乘法: r_objs.inv() * r_end_obj
    r_rel_objs = r_objs.inv() * r_end_obj
    rvec_1e = r_rel_objs.as_rotvec() # (N, 3)
    
    # 4. 组装数据
    # 为了保证列顺序，我们手动构建新的 DataFrame
    
    # Group 1: 原始的基础位姿
    df_new = df[['img', 'x', 'y', 'z', 'rx', 'ry', 'rz']].copy()
    
    # Group 2: 新增 旋转矩阵 9 列
    rmat_cols = ['r00', 'r01', 'r02', 'r10', 'r11', 'r12', 'r20', 'r21', 'r22']
    df_rmat = pd.DataFrame(r_flat, columns=rmat_cols)
    
    # Group 3: 原始的 Step-by-Step Delta 和 Force
    # 注意：确保这些列在原始 csv 里存在
    step_cols = ['dx', 'dy', 'dz', 'drx', 'dry', 'drz', 'fx', 'fy', 'fz']
    df_step = df[step_cols].copy()
    
    # Group 4: 新增 Current-to-End Delta
    cols_1e = ['dx1e', 'dy1e', 'dz1e', 'drx1e', 'dry1e', 'drz1e']
    data_1e = np.hstack([t_1e, rvec_1e])
    df_1e = pd.DataFrame(data_1e, columns=cols_1e)
    
    # 横向拼接
    result_df = pd.concat([df_new, df_rmat, df_step, df_1e], axis=1)
    
    # 5. 保存
    # 保存为 delta_pose_force.csv，在同级目录
    save_path = csv_path.parent / 'delta_pose_force.csv'
    result_df.to_csv(save_path, index=False)
    # print(f"Saved: {save_path}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=str, required=True, help="包含各个日期文件夹的根目录")
    args = parser.parse_args()

    root_path = Path(args.root)
    
    # 递归查找所有的 pose.csv
    # 结构假设: root/20250625/pose.csv
    csv_files = list(root_path.rglob("pose.csv"))
    
    print(f"Found {len(csv_files)} pose.csv files.")
    
    for csv_file in tqdm(csv_files, desc="Enhancing Features"):
        try:
            process_csv(csv_file)
        except Exception as e:
            print(f"[Error] Failed to process {csv_file}: {e}")

    print("\n[Done] All csv files processed.")

if __name__ == "__main__":
    # 使用方法: python data_feature_enhance.py --root ./data_downsampled
    main()