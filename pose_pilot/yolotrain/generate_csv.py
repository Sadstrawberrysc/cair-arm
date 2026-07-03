import os
import pandas as pd

def create_dataset_csv(root_dir):
    # 1. 定义表头
    columns = [
        'img', 'x', 'y', 'z', 'rx', 'ry', 'rz', 
        'r00', 'r01', 'r02', 'r10', 'r11', 'r12', 'r20', 'r21', 'r22', 
        'dx', 'dy', 'dz', 'drx', 'dry', 'drz', 
        'fx', 'fy', 'fz', 
        'dx1e', 'dy1e', 'dz1e', 'drx1e', 'dry1e', 'drz1e', 
        'dx1f', 'dy1f', 'dz1f', 'drx1f', 'dry1f', 'drz1f', 
        'dtheta_deg', 
        'yolo_big_x1', 'yolo_big_y1', 'yolo_big_x2', 'yolo_big_y2', 
        'yolo_pt_x', 'yolo_pt_y', 'yolo_ok', 'yolo_count'
    ]

    # 检查根目录是否存在
    if not os.path.exists(root_dir):
        print(f"错误: 目录 '{root_dir}' 不存在。")
        return

    # 2. 遍历根目录下的所有子文件夹
    # sorted保证处理顺序，os.listdir列出所有内容
    subdirs = sorted([d for d in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir, d))])
    
    print(f"找到 {len(subdirs)} 个子文件夹，准备开始处理...")

    for subdir in subdirs:
        subdir_path = os.path.join(root_dir, subdir)
        
        # 3. 获取该文件夹下的所有 jpg 图片
        # 这里假设图片后缀是 .jpg (忽略大小写)
        images = [f for f in os.listdir(subdir_path) if f.lower().endswith('.jpg')]
        images.sort() # 按文件名排序

        if not images:
            print(f"警告: 文件夹 {subdir} 中没有找到图片，跳过。")
            continue

        # 4. 构建 DataFrame
        # 首先创建包含图片名的列
        df = pd.DataFrame({'img': images})
        
        # 将其余列全部初始化为 0.0
        # 遍历除 'img' 之外的所有列名
        for col in columns[1:]:
            df[col] = 0.0

        # 5. 保存为 CSV
        csv_path = os.path.join(subdir_path, 'delta_pose_force.csv')
        
        # index=False 表示不保存行号，float_format='%.1f' 也可以用来控制小数位数，这里默认即可
        df.to_csv(csv_path, index=False)
        
        print(f"已生成: {csv_path} (包含 {len(df)} 行数据)")

    print("\n所有任务已完成！")

if __name__ == "__main__":
    # 设置你的根目录路径
    ROOT_DIR = 'translation_fake'
    create_dataset_csv(ROOT_DIR)