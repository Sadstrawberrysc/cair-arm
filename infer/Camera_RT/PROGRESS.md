# Camera_RT 当前进度与运行边界

更新日期：2026-09-15

## 当前结论

Camera_RT 已有完整的 RealSense、Faster R-CNN、CLIFF、SMPL、骨骼/血管渲染和相机坐标
路径导出原型源码。2026-09-03 用户已在当前机器与位置确认程序可以运行；环境、模型资产、
设备预检和实时入口均已越过启动阻塞；外参已通过模块初验，但独立已知点和悬空验收未完成。

本服务不是机器人坐标服务。accepted `T_base_camera` 只能由 Calibration 离线转换读取；
任何旧路径、旧外参或未经摘要绑定的 Base 文件仍不得用于 Robot。

## 已完成

- 2026-09-15：数字人颈动脉采样顶点已从右侧 5 点切换为左侧 8 点；保持画面
  最下方点优先的排序和原有路径文件格式。已完成离线语法与单元检查，尚未现场
  核对左颈叠加位置及深度反投影。
- 按用户指定，将第一采样点定义为画面最下方红点；像素排序后再转三维坐标，法向随首点计算。
  已增加离线排序测试，尚未现场核对新采样顺序。

- 确认活动入口为 `cliff_demo.py`，人体检测使用 torchvision Faster R-CNN。
- 确认 `lib/` 下 YOLOv3 不在当前入口依赖图中。
- 复用本机 Conda `camera` 环境作为 Python 3.8/Torch 1.13.1+cu117 基线。
- 整理 `requirements.txt` 的重复、冲突和无效条目。
- 从历史工程装配七个活动模型资产，使用 SHA-256 固定本机资产身份。
- 将旧机器的两个路径文件隔离到 `tools/fixtures/`，只用于格式测试。
- 新增静态/运行预检和纯离线路径格式测试。
- 2026-09-02 静态预检以 `blocked=0` 通过；路径格式测试 2/2 通过。
- 2026-09-03 用户确认 `cliff_demo.py` 在新机器上可以运行。
- RealSense stream 已在 `pipeline.start()` 前配置；Esc/`q` 可退出并释放设备和窗口。

## 当前环境事实

| 项目 | 当前状态 |
| --- | --- |
| Conda | `camera`，Python 3.8 |
| Torch | `1.13.1+cu117` |
| torchvision | `0.14.1+cu117` |
| PyTorch3D | `0.7.5` |
| RealSense Python | `2.54.2.5684` 已安装 |
| NVIDIA | 宿主预检通过：RTX 4080 Laptop、驱动 535.129.03、显存 12282 MiB |
| CUDA runtime | 宿主预检通过：Torch 1.13.1+cu117、CUDA 11.7、RTX 4080 Laptop |
| RealSense device | 宿主 SDK 枚举通过：Intel RealSense D455，SDK serial `318122303626` |
| RealSense USB | sysfs 为 D455、5 Gb/s，USB descriptor serial `318523150496` |
| 检测器缓存 | Faster R-CNN checkpoint 已存在于 Torch hub cache，大小 167502836 bytes |
| PyTorch3D 来源 | 0.7.5 原生扩展由 `/home/cair-jacen/Pytorch/pytorch3d` editable checkout 提供 |

2026-09-02 在普通宿主终端重新运行 `python tools/preflight.py --runtime`，结果为
`blocked=0, warnings=2`：NVIDIA driver、Torch CUDA、RealSense kernel 和 RealSense SDK
device 全部通过。当时的两个警告分别是 PyTorch3D 使用外部 editable checkout，以及
Faster R-CNN checkpoint 尚未缓存；后者现已缓存，只剩 editable 安装可移植性警告。

此前 Codex 沙箱中的 3 项 `BLOCKED` 均由沙箱看不到 `/dev/nvidia*`、USBFS 和
systemd/udev bus 导致，不能视为宿主硬件故障。后续设备验收均应在普通宿主终端执行。
sysfs USB descriptor 与 librealsense SDK 返回了两个不同序列号；后续流选择和标定元数据以
SDK `camera_info.serial_number` 为运行身份，同时保留 USB descriptor serial 供物理排障，
不得静默将二者当作同一字段。

## 模型资产

活动资产位于被 Git 忽略的 `data/`：CLIFF HR48 checkpoint、SMPL mean、SMPL neutral、
骨骼 params/shapedir、血管 params/shapedir。它们来源于
`/home/cair-jacen/CGL/Demo/AutoScan/CarotidArtery/Camera_RT/data`，摘要记录在
`tools/assets.sha256`。未复制活动入口不使用的 YOLOv3 权重。

模型可用只表示文件身份匹配，不表示模型许可允许再分发，也不表示当前人群或新相机精度有效。

## 已知功能问题

- 入口计算了 `device`，但多个模型和张量仍硬编码 `.cuda()`。
- 未检测到人体时没有可靠的 `bbox is None` 分支。
- 像素裁剪只限制上界，负坐标可能错误索引深度图。
- 深度全部为零时，最近非零深度搜索可能失败。
- 路径输出没有时间戳、设备身份、内参摘要、置信度或坐标 schema 版本。
- `artery_path_rbt.txt` 没有当前生成器，历史含义不可信。

## 当前验证命令

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/Camera_RT
python tools/preflight.py --static
python -m unittest tools/test_path_format.py
python tools/preflight.py --runtime
```

静态预检、格式测试、宿主运行预检与入口启动均已通过。该结论证明依赖、资产、设备访问和
实时程序可用，但尚不代表叠加精度、深度精度或输出路径空间准确性已经验收。

## 下一阶段

1. 在目标画面稳定后按 `s` 固化一次新的相机路径快照。
2. 使用 `infer/Calibration/calibrate.py convert` 生成 Base 路径及摘要元数据。
3. 在 Robot 维护工具中固定选择第一点，先执行只读状态与 IK dry-run。
4. 独立完成 Probe TCP 与现场安全验收后，再单独授权一次低速 MoveJ。
