# Camera_RT 验证工具

本目录只包含离线检查和历史格式夹具，不包含相机—机器人标定或运动工具。

## 当前环境

当前基线为 Conda 环境 `camera`：Python 3.8、PyTorch 1.13.1+cu117、torchvision
0.14.1+cu117 和 PyTorch3D 0.7.5。先激活环境：

当前 PyTorch3D 是从 `/home/cair-jacen/Pytorch/pytorch3d` editable 安装的原生扩展；迁移或
清理该源码目录前必须先重建 PyTorch3D，否则环境中的元数据仍可能存在但扩展无法导入。

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/Camera_RT
```

重建环境前核对 [requirements.txt](../requirements.txt) 和 [preflight.py](preflight.py) 的兼容基线；
不要为消除提示批量重装已验证环境。历史依赖版本不代表全仓库版本锁。

## 预检

```bash
python tools/preflight.py --static
python tools/preflight.py --runtime
```

`--static` 检查环境、依赖、源码语法和七个本机模型资产，成功返回 `0`。`--runtime`
还枚举 NVIDIA GPU、CUDA、RealSense 和 Faster R-CNN 缓存；任何必需硬件被标为
`BLOCKED` 时返回 `1`。预检不会打开数据流、下载权重或写路径文件。

Codex/容器沙箱可能允许读取 `/proc` 和 `/sys`，但隐藏 `/dev/nvidia*`、`/dev/video*`、
USBFS 与 systemd/udev bus。此时内核枚举会显示 `PASS`，SDK 访问仍显示 `BLOCKED`；必须在
机器的普通管理员终端重新执行预检，不能据沙箱结果重装已存在的硬件驱动。

输出含义：

- `PASS`：检查项满足当前基线。
- `WARN`：不影响本项检查，但功能运行前仍需处理。
- `BLOCKED`：当前层级不能继续，保留原始错误用于定位。

宿主设备访问异常时先检查 `nvidia-smi`、USB 枚举和 SDK 错误，保留原始输出。
驱动、udev 权限和用户组由管理员在宿主处理；沙箱不可见设备不能作为重装驱动的依据。
设备与环境历史见 [Camera_RT 进展](../../../.runme/Progress/camera-rt-progress.md)。

## 路径格式测试

```bash
python -m unittest tools/test_path_format.py
```

采样顺序和深度反投影回归检查（不导入相机 / GPU 驱动、不连接设备）：

```bash
python -m unittest tools/test_sample_order.py tools/test_depth_projection.py
```

深度测试覆盖 Z16 米制转换、深度孔洞最近补点、全零深度拒绝和图像边界索引；
不代表模型定位、深度补点或输出坐标已通过现场精度验收。

`fixtures/` 中两份文件来自旧机器，只用于刻画格式。第一行是单位法向量，后续行是三维点。
测试只检查三列有限数值和基本结构，不认可其空间坐标，也不比较两份文件数值。

运行时 `artery_path.txt` 的点使用 RealSense 光学坐标系，单位为米。历史名称
`artery_path_rbt.txt` 没有可信生成器，不能据文件名认定为机器人坐标。
