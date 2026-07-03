import os
import cv2
import torch # 需要导入 torch 来处理索引
from ultralytics import YOLO
from pathlib import Path

def main():
    # ---------------- 配置区域 ----------------
    # 1. 模型路径 (请修改为你训练好的最佳权重路径)
    # 建议使用绝对路径以防万一，例如: 'D:/projects/yolo_artery/runs/detect/artery_v1/weights/best.pt'
    model_path = 'runs/detect/artery_v1/weights/best.pt'

    # 2. 输入和输出根目录
    source_root = 'translation_fake'
    target_root = 'results_best_only' # 修改输出目录名，避免混淆

    # 3. 支持的图片格式
    img_formats = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')

    # 4. 基础置信度阈值 (低于此分数的首先会被 YOLO 过滤掉)
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

    print(f"开始处理 (只保留最高置信度结果): {source_root} -> {target_root}")

    count = 0
    processed_files = []

    # os.walk 递归遍历所有子文件夹
    for root, dirs, files in os.walk(source_root):
        for file in files:
            if file.lower().endswith(img_formats):
                # 构建路径
                src_path = os.path.join(root, file)
                rel_path = os.path.relpath(src_path, source_root)
                dst_path = os.path.join(target_root, rel_path)
                dst_dir = os.path.dirname(dst_path)

                if not os.path.exists(dst_dir):
                    os.makedirs(dst_dir)

                # 5. 推理
                # results 是一个列表，包含批次中每张图的结果
                results = model(src_path, conf=conf_threshold, verbose=False)
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
                    
                    # print(f"  [过滤] {rel_path}: 原有 {num_boxes} 个检测，只保留置信度最高的第 {max_conf_idx} 个")

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

                # 7. 保存图像
                # 确保图像数据不为空
                if annotated_frame is not None and annotated_frame.size > 0:
                    cv2.imwrite(dst_path, annotated_frame)
                    count += 1
                    if count % 10 == 0:
                        print(f"已处理 {count} 张图片...")
                else:
                    print(f"警告: 无法获取图像数据 {src_path}")

    print(f"\n全部完成！共处理 {count} 张图片。")
    print(f"结果已保存在: {os.path.abspath(target_root)}")

if __name__ == '__main__':
    main()