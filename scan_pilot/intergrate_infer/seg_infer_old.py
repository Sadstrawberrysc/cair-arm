import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt
import segmentation_models_pytorch as smp
import os
import glob

# ================= 🔧 配置区域 (改这里就行) =================

# 1. 模型路径
MODEL_PATH = './latest_model.pth' 

# 2. 输入图片文件夹 (会自动寻找里面的所有 jpg/png)
INPUT_DIR = r'./data_coco/test/'

# 3. 输出结果保存到哪里
OUTPUT_DIR = './inference_results'

# 4. 类别名称 (修改为你实际的类别名)
CLASS_NAMES = ['Background', 'Class 1: artery', 'Class 2: artery_seg']

# 5. 颜色定义 [R, G, B]
# 0类:黑色, 1类:黄色, 2类:紫色
COLOR_MAP = [
    [0, 0, 0],       
    [255, 255, 0],   
    [128, 0, 128]    
]

ENCODER = 'resnet34'
ENCODER_WEIGHTS = 'imagenet'
N_CLASSES = 3
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
# ===========================================================

def colorize_mask(mask, color_map):
    color_mask = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    for class_id, color in enumerate(color_map):
        color_mask[mask == class_id] = color
    return color_mask

def predict_batch(model, input_dir, output_dir, device):
    # 确保输出文件夹存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 寻找所有图片
    image_paths = glob.glob(os.path.join(input_dir, '*.*'))
    valid_exts = ['.jpg', '.jpeg', '.png', '.bmp']
    image_paths = [p for p in image_paths if os.path.splitext(p)[1].lower() in valid_exts]
    
    print(f"📂 找到 {len(image_paths)} 张图片，开始处理...")

    model.eval()
    
    for idx, image_path in enumerate(image_paths):
        filename = os.path.basename(image_path)
        print(f"[{idx+1}/{len(image_paths)}] 正在处理: {filename} ...")
        
        # 1. 读取
        image = cv2.imread(image_path)
        if image is None: continue
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        original_image = image.copy()
        h, w = image.shape[:2]

        # 2. 预处理
        image = image.astype('float32') / 255.0
        image = image.transpose(2, 0, 1)
        x_tensor = torch.from_numpy(image).unsqueeze(0).to(device)

        # 3. Padding (补全为32的倍数)
        import torch.nn.functional as F
        pad_h = (32 - h % 32) % 32
        pad_w = (32 - w % 32) % 32
        if pad_h > 0 or pad_w > 0:
            x_tensor = F.pad(x_tensor, (0, pad_w, 0, pad_h), mode='constant', value=0)

        # 4. 预测
        with torch.no_grad():
            output = model(x_tensor)
            pred_mask = torch.argmax(output, dim=1).squeeze(0).cpu().numpy()

        # 5. 去除 Padding
        if pad_h > 0 or pad_w > 0:
            pred_mask = pred_mask[:h, :w]

        # 6. 可视化合成
        colored_mask = colorize_mask(pred_mask, COLOR_MAP)
        overlay = cv2.addWeighted(original_image, 0.6, colored_mask, 0.4, 0)

        # 7. 用 Matplotlib 画图并保存
        fig = plt.figure(figsize=(12, 4))
        
        # 原图
        plt.subplot(1, 3, 1)
        plt.imshow(original_image)
        plt.title(f"Original: {filename}")
        plt.axis('off')
        
        # Mask
        plt.subplot(1, 3, 2)
        plt.imshow(colored_mask)
        plt.title("Prediction Mask")
        plt.axis('off')
        
        # 结果
        plt.subplot(1, 3, 3)
        plt.imshow(overlay)
        plt.title("Overlay Result")
        plt.axis('off')

        # 增加图例说明 (可选)
        plt.suptitle(f"Yellow: {CLASS_NAMES[1]} | Purple: {CLASS_NAMES[2]}", fontsize=12)

        # 保存图片而不是显示
        save_path = os.path.join(output_dir, f"result_{filename}")
        plt.savefig(save_path, bbox_inches='tight')
        plt.close(fig) # 关闭画布释放内存

    print(f"\n✅ 全部完成！结果已保存在: {output_dir}")

if __name__ == '__main__':
    # 加载模型
    print(f"正在加载模型: {MODEL_PATH}")
    model = smp.Unet(
        encoder_name=ENCODER, 
        encoder_weights=ENCODER_WEIGHTS, 
        classes=N_CLASSES, 
        activation=None
    )
    if os.path.exists(MODEL_PATH):
        model.load_state_dict(torch.load(MODEL_PATH))
        model.to(DEVICE)
    else:
        print("❌ 模型没找到！")
        exit()

    # 运行批量预测
    predict_batch(model, INPUT_DIR, OUTPUT_DIR, DEVICE)