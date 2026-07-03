import os
import shutil

# --- 配置部分 ---
source_folder = '/home/mingcong/project/us_carotid/segmentation/carotid/data_mask/train'  # 你那个混合文件夹的路径
output_root = '/home/mingcong/project/us_carotid/segmentation/carotid/'                   # 整理后存放的位置

# 创建目标文件夹
img_dest = os.path.join(output_root, 'images')
mask_dest = os.path.join(output_root, 'masks')
os.makedirs(img_dest, exist_ok=True)
os.makedirs(mask_dest, exist_ok=True)

# 获取所有文件并排序
files = sorted(os.listdir(source_folder))

print(f"总共发现 {len(files)} 个文件。正在处理...")

# --- 模式 A: 如果文件名里包含 'mask' 字样 (推荐) ---
# 假设: 1.jpg, 1_mask.png
for f in files:
    src_path = os.path.join(source_folder, f)
    
    # 这一行是关键：根据文件名特征判断
    if 'mask' in f.lower():  
        shutil.copy(src_path, os.path.join(mask_dest, f))
    else:
        shutil.copy(src_path, os.path.join(img_dest, f))


print("整理完成！请检查 data/train 文件夹。")