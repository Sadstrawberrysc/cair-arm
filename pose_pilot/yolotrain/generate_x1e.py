import os
import pandas as pd
import numpy as np

def update_dx1e_column(root_dir, start_val, is_negative):
    # 1. 确定起始值的符号
    # 如果 is_negative 为 True，强制起始值为负；否则为正
    real_start_val = -abs(start_val) if is_negative else abs(start_val)
    target_end_val = 0.0

    print(f"配置确认: dx1e 将从 {real_start_val} 线性变化到 {target_end_val}")

    # 2. 遍历目录
    subdirs = sorted([d for d in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir, d))])
    
    count = 0
    for subdir in subdirs:
        subdir_path = os.path.join(root_dir, subdir)
        csv_path = os.path.join(subdir_path, 'delta_pose_force.csv')

        # 检查 CSV 是否存在
        if not os.path.exists(csv_path):
            print(f"跳过: {subdir} 中未找到 csv 文件")
            continue

        # 3. 读取 CSV
        df = pd.read_csv(csv_path)
        n_rows = len(df)

        if n_rows == 0:
            continue

        # 4. 生成线性数据
        # np.linspace(start, stop, num) 会生成包含 start 和 stop 的等差数列
        if n_rows > 1:
            # 正常情况：生成从 real_start_val 到 0.0 的 n_rows 个数
            # 比如 n=3, start=0.005 -> [0.005, 0.0025, 0.0]
            new_values = np.linspace(real_start_val, target_end_val, n_rows)
        else:
            # 特殊情况：如果只有1行数据，直接设为 0.0 或者起始值（此处设为起始值，可视需求修改）
            new_values = [real_start_val]

        # 5. 赋值给 dx1e 列
        df['dx1e'] = new_values

        # 6. 保存回 CSV
        # float_format='%.6f' 保证保留足够的小数位，防止 0.0025 变成 0.003
        df.to_csv(csv_path, index=False, float_format='%.6f')
        count += 1

    print(f"\n成功更新了 {count} 个文件夹中的 dx1e 列。")

if __name__ == "__main__":
    # ================= 配置区域 =================
    ROOT_DIR = 'translation_fake/ps'  # 数据集根目录
    
    START_VALUE = 0.005            # 起始值的绝对值大小
    USE_NEGATIVE = False           # 【选项】True: 从 -0.005 变到 0; False: 从 0.005 变到 0
    # ===========================================

    update_dx1e_column(ROOT_DIR, START_VALUE, USE_NEGATIVE)