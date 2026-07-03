import torch
import cv2
import numpy as np
import segmentation_models_pytorch as smp
import os
import torch.nn.functional as F

# ================= 🔧 配置区域 (改这里就行) =================

# 1. 模型路径
MODEL_PATH = 'weights/seg_model.pth' 

# 2. 视频源 (0 表示默认摄像头，也可以是视频文件路径)
VIDEO_SOURCE = 0

# 3. 类别名称 (修改为你实际的类别名)
CLASS_NAMES = ['Background', 'Class 1: artery', 'Class 2: artery_seg']

# 4. 颜色定义 [R, G, B]
# 0类:黑色, 1类:黄色, 2类:紫色
COLOR_MAP = [
    [0, 0, 0],       
    [0, 0, 0],   
    [255, 50, 0]    
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

def run_live_inference(model, device):
    # 初始化视频捕获
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    
    # 设置分辨率 (如果是摄像头)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920) # 可调整
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080) # 可调整
    
    if not cap.isOpened():
        print(f"❌ 无法打开视频源: {VIDEO_SOURCE}")
        return

    print("🎥 开始实时推理，按 'q' 键退出 ...")
    
    model.eval()

    with torch.no_grad():
        while True:
            ret, frame = cap.read()
            frame = cv2.flip(frame, 0)
            frame = frame[160:680,595:1415,:]            
            if not ret:
                print("⚠️ 无法读取帧 (视频结束或出错)")
                break

            # 备份原图用于显示
            original_image = frame.copy()
            h, w = frame.shape[:2]

            # 1. 预处理 (OpenCV读入是BGR, 需要转RGB)
            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 归一化并转Tensor
            image_tensor = image_rgb.astype('float32') / 255.0
            image_tensor = image_tensor.transpose(2, 0, 1) # HWC -> CHW
            x_tensor = torch.from_numpy(image_tensor).unsqueeze(0).to(device)

            # 2. Padding (补全为32的倍数，ResNet/UNet要求)
            pad_h = (32 - h % 32) % 32
            pad_w = (32 - w % 32) % 32
            
            if pad_h > 0 or pad_w > 0:
                x_tensor = F.pad(x_tensor, (0, pad_w, 0, pad_h), mode='constant', value=0)

            # 3. 预测
            output = model(x_tensor)
            pred_mask = torch.argmax(output, dim=1).squeeze(0).cpu().numpy()

            # 4. 去除 Padding
            if pad_h > 0 or pad_w > 0:
                pred_mask = pred_mask[:h, :w]

            # 5. 可视化合成
            # 将mask转为彩色图
            colored_mask = colorize_mask(pred_mask, COLOR_MAP)
            # OpenCV 使用 BGR。所以如果 COLOR_MAP 是 RGB，我们需要转成 BGR 给 cv2.imshow
            colored_mask_bgr = cv2.cvtColor(colored_mask, cv2.COLOR_RGB2BGR)
            
            # 叠加: 原图是BGR, colored_mask_bgr是BGR
            overlay = cv2.addWeighted(original_image, 0.6, colored_mask_bgr, 0.4, 0)

            # 6. 显示结果
            cv2.imshow('Real-time Segmentation', overlay)
            
            # 按 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    print("✅ 推理结束")

if __name__ == '__main__':
    # 加载模型
    print(f"正在加载模型: {MODEL_PATH}")
    # 这里需要确保 smp 库已安装且模型结构匹配
    try:
        model = smp.Unet(
            encoder_name=ENCODER, 
            encoder_weights=ENCODER_WEIGHTS, 
            classes=N_CLASSES, 
            activation=None
        )
        if os.path.exists(MODEL_PATH):
            model.load_state_dict(torch.load(MODEL_PATH))
            model.to(DEVICE)
            
            # 运行实时预测
            run_live_inference(model, DEVICE)
        else:
            print(f"❌ 模型文件不存在: {MODEL_PATH}")
            print("请检查 MODEL_PATH 配置。")
    except Exception as e:
        print(f"❌ 发生错误: {e}")
