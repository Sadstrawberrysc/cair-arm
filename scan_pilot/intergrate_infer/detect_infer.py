import os
import cv2
import torch # 需要导入 torch 来处理索引
from ultralytics import YOLO
from pathlib import Path

def main():
    # ---------------- 配置区域 ----------------
    # 1. 模型路径 (请修改为你训练好的最佳权重路径)
    # 建议使用绝对路径以防万一，例如: 'D:/projects/yolo_artery/runs/detect/artery_v1/weights/best.pt'
    model_path = '/home/cair/Projects/usproj/us_carotid/pose_pilot/yolotrain/runs/detect/artery_v1/weights/best.pt'

    # 2. 视频源 (0 表示默认摄像头，也可以是视频文件路径)
    video_source = 0

    # 3. 基础置信度阈值 (低于此分数的首先会被 YOLO 过滤掉)
    # 可以设置得稍低一些，反正后面只取最高
    conf_threshold = 0.5
    # ----------------------------------------

    # 加载模型
    # 确保路径存在，处理 Windows 路径可能的问题
    model_path = os.path.abspath(model_path)
    if not os.path.exists(model_path):
        print(f"错误: 找不到模型文件 {model_path}，请检查路径。")
        return

    print(f"正在加载模型: {model_path} ...")
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"模型加载失败: {e}")
        return

    # 初始化视频捕获
    camera_width = 1920
    camera_height = 1080
    camera_fps = 30
    cap = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        print(f"错误: 无法打开视频源 {video_source}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
    cap.set(cv2.CAP_PROP_FPS, camera_fps)
    

    print("开始处理视频流 (只保留最高置信度结果)，按 'q' 键退出 ...")

    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 0)
        frame = frame[160:680,595:1415,:]
        if not ret:
            print("视频流结束或无法读取帧")
            break

        # 5. 推理
        # results 是一个列表，包含批次中每张图的结果
        results = model(frame, conf=conf_threshold, verbose=False)
        result = results[0] # 我们是单张推理，取第一个结果即可

        # ================== 核心修改区域：过滤框 ==================
        # result.boxes 包含了所有检测到的框的信息
        num_boxes = len(result.boxes)

        if num_boxes > 1:
            # 如果检测到超过1个目标，需要筛选
            # 获取所有框的置信度 (这是一个 PyTorch tensor)
            confidences = result.boxes.conf

            # 找到最大置信度所在的索引值 (argmax 返回的是 tensor, 需要转成 python int)
            max_conf_idx = confidences.argmax().item()

            # 关键操作：修改 result.boxes
            # 我们只保留索引为 max_conf_idx 的那个框的数据
            # 注意：使用 [max_conf_idx] (列表包裹) 来进行索引，可以保持维度一致性
            result.boxes = result.boxes[max_conf_idx]
            
            # print(f"  [过滤] 原有 {num_boxes} 个检测，只保留置信度最高的第 {max_conf_idx} 个")

        elif num_boxes == 1:
            # 只有一个检测，无需操作
            pass
        else:
            # 没有检测到任何目标，plot() 会返回原图
            pass
        # =======================================================

        # 6. 获取绘制好框的图像 (numpy array)
        # 此时 plot() 只会画出我们筛选剩下的那一个最高分的框
        annotated_frame = result.plot()

        # 7. 显示图像
        cv2.imshow("Real-time Inference", annotated_frame)

        # 按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("已退出视频流处理。")

if __name__ == '__main__':
    main()