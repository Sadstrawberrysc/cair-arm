from ultralytics import YOLO
import os

def main():
    # 1. 加载模型
    # yolo11n.pt 会自动下载 (如果本地没有的话)
    # n=nano, s=small, m=medium, l=large, x=extra large
    print("正在加载 YOLO11 模型...")
    model = YOLO('yolo11x.pt') 

    # 2. 训练参数配置
    # data: 指向你的 data.yaml 文件的路径
    # epochs: 训练轮数，建议从 100 开始
    # imgsz: 图片输入尺寸，通常为 640
    # device: 0 表示使用第一个 GPU，如果没有 GPU 则设为 'cpu'
    # batch: 批次大小，显存不够可以调小 (如 8, 4)
    # project: 训练结果保存的根目录
    # name: 本次训练任务的名称
    
    print("开始训练...")
    results = model.train(
        data='data_artery/data.yaml',   # 确保这里路径正确，建议使用绝对路径
        epochs=300,         
        imgsz=640,          
        batch=16,           
        device=0,           # 如果使用 CPU，请改为 device='cpu'
        project='runs/detect',
        name='artery_v1',   
        exist_ok=True,      # 如果存在同名文件夹，是否覆盖/继续
        patience=30,        # 20轮没有提升则提前停止
        workers=4           # 数据加载线程数
    )

    print("训练完成！模型保存在 runs/detect/artery_v1/weights/best.pt")

    # 3. (可选) 在验证集上进行验证
    metrics = model.val()
    print(f"mAP50: {metrics.box.map50}")

if __name__ == '__main__':
    # Windows 下多进程运行必须放在 if __name__ == '__main__': 之下
    main()