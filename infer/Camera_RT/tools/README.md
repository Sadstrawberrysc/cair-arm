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

若未来重建环境，CUDA 版 Torch/torchvision 使用 `requirements.txt` 中配置的 PyTorch
索引；PyTorch3D 需要先按其 CUDA 11.7/PyTorch 1.13 兼容方式单独安装，例如：

```bash
conda install -c pytorch3d pytorch3d=0.7.5
pip install -r requirements.txt
```


不要在当前已验证环境中为消除提示而批量重装。

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

当前机器的宿主检查顺序：

```bash
sudo nvidia-modprobe -c=0
sudo nvidia-modprobe -u
sudo nvidia-modprobe -m
nvidia-smi
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
lsusb | grep -i realsense
python tools/preflight.py --runtime
```

若 `nvidia-modprobe` 后设备节点仍未出现，应先重启机器；不要在实时内核上直接卸载正在使用的
NVIDIA 模块。D455 已有 `/etc/udev/rules.d/99-realsense-libusb.rules`，产品 ID `0b5c` 已覆盖。
若宿主终端仍无权限，再由管理员执行 `udevadm control --reload-rules`，重新插拔 D455，并将当前
用户加入 `video` 组后重新登录。不要在 Codex 沙箱里执行这些系统级操作。

## 路径格式测试

```bash
python -m unittest tools/test_path_format.py
```

`fixtures/` 中两份文件来自旧机器，只用于刻画格式。第一行是单位法向量，后续行是三维点。
测试只检查三列有限数值和基本结构，不认可其空间坐标，也不比较两份文件数值。

运行时 `artery_path.txt` 的点使用 RealSense 光学坐标系，单位为米。历史名称
`artery_path_rbt.txt` 没有可信生成器，不能据文件名认定为机器人坐标。
