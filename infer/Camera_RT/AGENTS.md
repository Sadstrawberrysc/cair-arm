# Camera_RT 项目指南

## 项目概览

本目录是 USPilot Control 的人体/解剖相机原型服务。
当前入口 `cliff_demo.py` 从 Intel RealSense 读取彩色图和对齐深度图，
估计人体姿态并在画面上叠加骨骼与颈动脉模型。

当前服务负责本地感知、显示和按键触发的相机坐标路径快照导出。
它不读取外参、不连接 RM75，也不直接生成或发布 Base 坐标目标。
这是工程调试程序，不是已经完成测量精度验收的医疗或机器人导航系统。

活动链路为：

```text
RealSense RGB-D
  → torchvision Faster R-CNN 人体框
  → CLIFF HR48 人体姿态与体型
  → SMPL + AnatomyModel 骨骼/血管网格
  → PyTorch3D 投影与 OpenCV 叠加
  → artery_path.txt（相机光学坐标，m）
  → 按 s 写入 artery_path.txt（法向 + 采样点）
```

主要文件：

- `cliff_demo.py`：当前实时入口与路径导出。
- `anatomy_layer.py`：骨骼/血管蒙皮和关节变换。
- `models/`、`common/`：CLIFF、HRNet 和图像/渲染辅助代码。
- `data/`：本机模型资产，已被 Git 忽略。
- `tools/`：预检、摘要清单和纯离线路径格式测试。
- `lib/`：旧 YOLOv3 实现，不属于当前入口活动链路。

详细边界见 `ARCHITECTURE.md`，当前状态见 `PROGRESS.md`，长期决定见
`DECISIONS.md`。

## 运行命令

先激活现有基线环境并进入本目录：

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/Camera_RT
```

首次接入或更换机器时先运行静态预检：

```bash
python tools/preflight.py --static
python -m unittest tools/test_path_format.py
```

连接 GPU 和 RealSense 后再运行只读设备预检：

```bash
python tools/preflight.py --runtime
```

只有静态和运行预检均无 `BLOCKED` 后，才进入功能调试：

```bash
python cliff_demo.py
```

当前入口按 `s` 保存 `artery_path.txt`；Esc 保存后退出，`q` 不保存并退出。
按 `s` 固化当前法向和采样点；下游只读取这一次快照，不使用逐帧实时消息。
正常退出和解释器退出都会释放 RealSense pipeline 与 OpenCV 窗口。

## 验证与硬约束

- `--static` 通过只证明解释器、依赖、源码语法和模型资产满足当前基线。
- `--runtime` 只枚举 GPU、CUDA、RealSense 和检测器缓存，不打开视频流。
- 不得把预检通过描述为推理、帧率、深度精度或真相机显示已经验收。
- 不得把 `tools/fixtures/` 中的旧机器数据用于运行、标定或机器人控制。
- `artery_path.txt` 第一行是相机坐标法向量，后续行是相机三维点，单位为米。
- `artery_path_rbt.txt` 没有可信生成器，文件名不能证明数据属于机器人坐标。
- 新机器、新位置或相机重新固定后，旧路径与旧外参全部失效。
- `infer/Calibration` 使用 accepted 外参把一次快照离线转换为 RM75 Base 坐标。
- 未验证相机内参、深度尺度、时间同步和外参残差前，不得生成机器人目标。
- 未经用户明确授权和现场安全确认，不得启动 `main_rm75` 或发送运动命令。
- Robot 只能消费 Calibration 生成且摘要匹配的 Base 路径，不能直接读取相机坐标。
- 相机功能调试不得通过启动机器人来验证坐标方向。
- 模型权重、SMPL 资产、视频、缓存和实时路径文件不得提交到 Git。
- 不要直接用当前 `requirements.txt` 批量覆盖已验证的 `camera` 环境。
- 修改依赖时先更新预检基线，并在 `DECISIONS.md` 记录兼容性决定。
- 每次只处理一个功能点；环境和静态验证通过后再改实时推理。
- 后续公共接口、坐标语义或标定格式变更必须同步模块和根级决策日志。
