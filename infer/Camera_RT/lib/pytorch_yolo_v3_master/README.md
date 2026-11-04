# YOLOv3 PyTorch：历史依赖

本目录保留 Ayoosh Kathuria 的 YOLOv3 检测实现，不属于当前 Camera_RT 活动链路；
当前入口使用 Faster R-CNN，见 [Camera_RT 架构](../../ARCHITECTURE.md)。
以下为上游旧环境与命令，不代表本仓库已验证其运行兼容性。

## 来源与环境

基于 [YOLOv3 论文](https://pjreddie.com/media/files/papers/YOLOv3.pdf)、
[Darknet](https://github.com/pjreddie/darknet) 和 [marvis 的 PyTorch 移植](https://github.com/marvis/pytorch-yolo2)。
上游实现仅含检测，历史依赖为 Python 3.5、PyTorch 0.4 和 OpenCV；PyTorch 0.3 不兼容。
[从零实现教程](https://blog.paperspace.com/how-to-implement-a-yolo-object-detector-in-pytorch/)保留供阅读。

## 历史命令

在本目录放置 [COCO yolov3.weights](https://pjreddie.com/media/files/yolov3.weights)，然后运行：

```bash
python detect.py --images imgs --det det
python detect.py -h
python detect.py --images imgs --det det --reso 320
python video_demo.py --video video.avi
python cam_demo.py
python detect.py --scales 1,3
```

| 参数 / 入口 | 含义 |
| --- | --- |
| `--images` / `--det` | 单图或输入目录 / 检测结果目录 |
| `--bs` | 批大小，其他参数见 help |
| `--reso` | 输入尺寸；大于 32 且为 32 的倍数，图片默认 416，相机旧入口默认 160 |
| `video_demo_half.py` | float16 视频推理实验，性能取决于硬件，需自行测量 |
| `cam_demo.py` | 默认 OpenCV 相机 0 |
| `--scales` | 选择检测尺度；上游曾标记该功能因重构禁用，应核对实际源码 |

其他权重见 [Darknet YOLO](https://pjreddie.com/darknet/yolo/)。旧示例使用 AVI，实际视频读取能力取决于 OpenCV 构建。
