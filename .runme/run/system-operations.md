# USPilot 常用启动入口（Runme notebook）

> 本文件只作为入口索引。连接设备、常驻进程和运动命令均排除 Run All，并应在对应模块
> notebook 中核对前置条件后逐个执行。

## 工作树状态

```bash {"name":"git-status","excludeFromRunAll":"true"}
git -C /home/cair-jacen/uspilot_ctrl-main status --short
```

## Redis 7777 状态

```bash {"name":"redis-7777-ping","excludeFromRunAll":"true"}
redis-cli -h 127.0.0.1 -p 7777 ping
```

## 超声在线推理

```bash {"name":"start-ultrasound-inference","excludeFromRunAll":"true","interactive":"true","background":"false","closeTerminalOnSuccess":"false"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
cd /home/cair-jacen/uspilot_ctrl-main/intergrate_infer
python main_redis_seg_newphase_recovery_mode.py
```

## 接触点显示

本机使用 `carotid` 环境（已安装 PyVista 0.49.0）；显示模型取自
`infer/Robot/model/Lprobe-IFS.STL`。

```bash {"name":"start-contact-point-viewer","excludeFromRunAll":"true","interactive":"true","background":"false","closeTerminalOnSuccess":"false"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
cd /home/cair-jacen/uspilot_ctrl-main/infer/ContactPointShow
python -c 'import pyvista' || { echo 'ContactPointShow 需要在 carotid 环境安装 pyvista。' >&2; exit 1; }
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 python main.py
```

## SensorMonitor

```bash {"name":"start-sensor-monitor","excludeFromRunAll":"true","interactive":"true","background":"false","closeTerminalOnSuccess":"false"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
  python /home/cair-jacen/uspilot_ctrl-main/infer/SensorMonitor/main.py
```

Robot、Camera_RT、Camera_wrist 和标定的完整步骤见本目录对应 notebook。
