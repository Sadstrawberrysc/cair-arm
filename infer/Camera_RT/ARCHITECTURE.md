# Camera_RT 人体/解剖相机服务架构

## 职责与边界

Camera_RT 是独立的 RGB-D 人体与解剖结构可视化原型。当前进程从 Intel RealSense 获取
彩色和深度帧，估计单个人体的 SMPL 姿态与体型，将同姿态的骨骼和血管网格投影到图像，
并把选定颈动脉顶点反投影为相机坐标点。

服务提供本地 OpenCV 窗口和按键触发的文本快照。它不读取外参、不连接机器人，也不发布
Base 目标；输出只能经 Calibration 的外参和摘要边界生成离线 Base 坐标文件。

## 数据流与模块

```text
RealSense color/depth ─→ align(depth→color) ─→ RGB image + depth map
                                  │
RGB image ─→ Faster R-CNN person box ─→ crop/bbox metadata ─→ CLIFF HR48
                                                            │
                                          rotation matrices + betas + camera
                                                            │
                     ┌──────────────────────────────────────┼───────────────┐
                     ↓                                      ↓               ↓
                  SMPL skin                         Anatomy skeleton   Anatomy artery
                     └────────────────────── PyTorch3D render ───────────────┘
                                                            │
                                             OpenCV overlay + artery pixels
                                                            │
                                     aligned depth + intrinsics + depth scale
                                                            ↓
                                         normal + camera-frame 3D path
                                                            │
                                           按 s 写 artery_path.txt
```

| 模块 | 职责 | 直接依赖 |
| --- | --- | --- |
| `cliff_demo.py` | 入口、设备生命周期、模型编排、渲染、反投影与文件输出 | RealSense、Torch、torchvision、PyTorch3D、OpenCV |
| `anatomy_layer.py` | 自定义解剖网格的姿态/体型蒙皮 | NumPy、pickle |
| `models/cliff_hr48` | 从图像 crop 和 bbox 信息回归 SMPL 参数 | Torch、HRNet、yacs |
| `common` | crop、相机参数换算、旋转表示和常量 | NumPy、OpenCV、Torch |
| `data` | CLIFF、SMPL、骨骼和血管本机资产 | 本机文件，不进入 Git |
| `tools` | 非侵入预检、资产摘要和格式测试 | Python 标准库及被检包 |
| `lib` | 未被当前入口引用的 legacy YOLOv3 | 不属于活动服务依赖 |

## 输入接口

- RealSense 彩色流和深度流；深度需要对齐到彩色像素。
- 彩色相机内参 `fx/fy/ppx/ppy` 与深度尺度。
- Torchvision Faster R-CNN COCO 权重；首次使用可能需要单独下载缓存。
- CLIFF HR48 权重、SMPL neutral 模型、均值参数及解剖模型资产。
- 当前模型只选择画面中面积最大的人体框。

## 输出接口与坐标

- `result`：骨骼和血管网格叠加画面。
- `color stream`：彩色图和血管采样点辅助显示。
- `artery_path.txt`：运行时生成、Git 忽略的空格分隔文本。

路径文件每行三个有限浮点数：第一行是局部表面法向量，后续行为血管采样点。点的单位为
米，采用 RealSense 光学坐标约定（X 向右、Y 向下、Z 向前）。第一行是方向，不应用平移。
采样点按图像像素纵坐标从大到小保存，第一采样点（文件第二行）是画面最下面的红点。
纵坐标相同则保留模型原始顺序。法向在这个首点附近估计。

旧文件名 `artery_path_rbt.txt` 没有当前生成器或已验证 schema，不能作为机器人坐标接口。
历史程序曾在读取同名文件后再次应用 camera-to-robot 变换，进一步说明该名称不具备坐标语义。

## 运行依赖与资产

当前基线复用 Conda `camera`：Python 3.8、Torch 1.13.1+cu117、torchvision
0.14.1+cu117、PyTorch3D 0.7.5、OpenCV 4.8.1 和 Open3D 0.17.0。完整版本由
`requirements.txt` 与 `tools/preflight.py` 共同刻画。

`data/` 包含七个活动资产并由 `tools/assets.sha256` 校验。目录被 Git 忽略；资产不得随源码
分发或提交。旧 YOLOv3 权重不是当前 Faster R-CNN 链路的依赖。

## 故障与安全边界

- 依赖或资产不完整时应在模型初始化前失败，不使用随机权重继续。
- GPU、CUDA 或 RealSense 不可用时运行预检返回 `BLOCKED`，不得记作链路通过。
- 无人体框、无有效深度、投影越界和设备退出清理尚未形成健壮边界，记录在 `PROGRESS.md`。
- 文本路径没有时效和设备字段，只能先经 Calibration 转换并生成摘要元数据；Robot 不得直接消费。
- 固定相机的 eye-to-hand 标定由相邻的 `infer/Calibration` 模块负责；Camera_RT 不读取外参，
  也不改变其相机坐标输出。Calibration 只有在独立验证通过后才允许离线生成 Base 坐标文件。
- 在外参正式验证前，只允许相机坐标可视化和离线检查，不允许机器人消费路径。

## 验证层级

1. `tools/preflight.py --static`：环境、包、源码和资产。
2. `tools/test_path_format.py`：旧夹具的格式 characterization，不验证空间正确性。
3. `tools/preflight.py --runtime`：GPU、CUDA、RealSense 和检测器缓存。
4. 后续功能验收：真实帧、人体框、叠加显示、深度有效性和新路径采集。
5. 标定验收：由 `infer/Calibration` 使用独立 eye-to-hand 数据集验证残差、方向和设备身份。

前三级通过不代表第四、第五级通过，也不代表机器人真机验收。
