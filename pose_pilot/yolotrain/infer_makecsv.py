import os
import pandas as pd
from ultralytics import YOLO
from tqdm import tqdm

# ================= 配置区域 =================
# 1. 数据的根目录 (你的原始文件夹路径)
ROOT_DIR = "./translation_fake" 

# 2. YOLO 模型路径
MODEL_PATH = "runs/detect/artery_v1/weights/best.pt"  # 请替换为你训练好的模型路径，例如 'runs/detect/train/weights/best.pt'

# 3. 置信度阈值
CONF_THRES = 0.5 

# 4. 目标类别 ID (通常单类别模型是 0)
TARGET_CLASS_ID = 0
# ===========================================

def process_csv_inplace(csv_path, folder_path, model):
    """
    读取 CSV -> 逐行推理 -> 更新 DataFrame -> 原地覆盖保存 CSV
    """
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"[Error] 无法读取 CSV: {csv_path}, 错误: {e}")
        return

    print(f"正在处理文件: {csv_path}")

# 1. 确保所有需要的列都存在，不存在则初始化为 0.0 (浮点数)
    target_cols = ['yolo_big_x1', 'yolo_big_y1', 'yolo_big_x2', 'yolo_big_y2', 
                   'yolo_pt_x', 'yolo_pt_y', 'yolo_ok']
    
    for col in target_cols:
        if col not in df.columns:
            # 修改点1：赋值为 0.0，告诉 pandas 这是一个存小数的列
            df[col] = 0.0 
        
        # 修改点2：即使列已经存在（可能全是0被读成了int），也强制转为float
        df[col] = df[col].astype(float)

    # 2. 遍历 CSV 的每一行进行推理
    # 使用 tqdm 显示进度
    for index, row in tqdm(df.iterrows(), total=df.shape[0], leave=False, desc="Updating rows"):
        img_name = str(row['img'])
        img_path = os.path.join(folder_path, img_name)

        # 默认值：未检测到 (全0)
        update_values = {
            'yolo_big_x1': 0, 'yolo_big_y1': 0, 'yolo_big_x2': 0, 'yolo_big_y2': 0,
            'yolo_pt_x': 0,   'yolo_pt_y': 0,
            'yolo_ok': 0
        }

        # 只有当图片文件存在时才进行推理
        if os.path.exists(img_path):
            try:
                # 核心：save=False 确保不生成图片，save_txt=False 确保不生成txt
                results = model.predict(img_path, conf=CONF_THRES, save=False, save_txt=False, verbose=False)
                
                if len(results) > 0 and len(results[0].boxes) > 0:
                    # 寻找符合类别的最佳框
                    best_box = None
                    for box in results[0].boxes:
                        if int(box.cls[0]) == TARGET_CLASS_ID:
                            best_box = box
                            break # 找到第一个符合的就停止 (通常置信度最高的排在前面)
                    
                    if best_box is not None:
                        # 提取坐标
                        x1, y1, x2, y2 = best_box.xyxy[0].tolist()
                        # 计算中心点
                        cx = (x1 + x2) / 2.0
                        cy = (y1 + y2) / 2.0
                        
                        update_values = {
                            'yolo_big_x1': round(float(x1), 2),
                            'yolo_big_y1': round(float(y1), 2),
                            'yolo_big_x2': round(float(x2), 2),
                            'yolo_big_y2': round(float(y2), 2),
                            'yolo_pt_x':   round(float(cx), 2),
                            'yolo_pt_y':   round(float(cy), 2),
                            'yolo_ok':     1
                        }
            except Exception as e:
                # 图片损坏或推理出错，保持默认0
                pass
        
        # 3. 将结果填入 DataFrame
        for col, val in update_values.items():
            df.at[index, col] = val

    # 4. 原地保存覆盖
    df.to_csv(csv_path, index=False)
    print(f"✅ 已更新并保存: {csv_path}\n")


def main():
    # 1. 加载模型 (只加载一次)
    print(f"正在加载模型: {MODEL_PATH} ...")
    try:
        model = YOLO(MODEL_PATH)
    except Exception as e:
        print(f"模型加载失败: {e}")
        return

    # 2. 递归查找所有 delta_pose_force.csv
    target_file = "delta_pose_force.csv"
    csv_files_found = []

    for root, dirs, files in os.walk(ROOT_DIR):
        if target_file in files:
            full_path = os.path.join(root, target_file)
            csv_files_found.append((full_path, root))

    print(f"共发现 {len(csv_files_found)} 个 CSV 文件待处理。")

    # 3. 逐个处理
    for csv_path, folder_path in csv_files_found:
        process_csv_inplace(csv_path, folder_path, model)

    print("所有任务完成！")

if __name__ == "__main__":
    main()