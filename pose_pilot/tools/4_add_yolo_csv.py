import argparse
from pathlib import Path
import pandas as pd
from tqdm import tqdm

def add_placeholder_columns(csv_path):
    # 1. 读取 CSV
    df = pd.read_csv(csv_path)
    
    # 2. 定义需要新增的列名列表
    new_cols = [
        'dtheta_deg', 
        'yolo_big_x1', 'yolo_big_y1', 'yolo_big_x2', 'yolo_big_y2', 
        'yolo_pt_x', 'yolo_pt_y', 
        'yolo_ok', 
        'yolo_count'
    ]
    
    # 3. 批量赋值为 0
    # Pandas 会自动将 0 广播到所有行
    for col in new_cols:
        df[col] = 1
        
    # 4. 覆盖保存
    # float_format='%.6f' 可选，用于控制小数精度，这里默认即可
    df.to_csv(csv_path, index=False)

def main():
    parser = argparse.ArgumentParser(description="向 delta_pose_force.csv 添加 YOLO 占位列")
    parser.add_argument("--root", type=str, required=True, help="数据根目录")
    args = parser.parse_args()

    root_path = Path(args.root)
    
    # 递归查找所有目标 CSV
    csv_files = list(root_path.rglob("delta_pose_force.csv"))
    
    print(f"找到 {len(csv_files)} 个 delta_pose_force.csv 文件，准备处理...")
    
    # 使用 tqdm 显示进度条
    for csv_file in tqdm(csv_files, desc="Adding Columns"):
        try:
            add_placeholder_columns(csv_file)
        except Exception as e:
            print(f"\n[Error] 处理文件失败 {csv_file}: {e}")

    print("\n[Done] 所有文件已更新，新增列全部初始化为 0。")

if __name__ == "__main__":
    # 使用方法: python data_add_yolo_placeholder.py --root ./data_dataset
    main()