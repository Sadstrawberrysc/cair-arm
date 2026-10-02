# Camera_RT 启动与快照（Runme notebook）

> Camera_RT 只输出 RealSense 相机光学坐标快照，不启动机器人，也不直接生成 Base 目标。

## 静态预检

```bash {"name":"camera-rt-static-preflight","excludeFromRunAll":"true"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/Camera_RT
python tools/preflight.py --static
```

## 设备运行预检

```bash {"name":"camera-rt-runtime-preflight","excludeFromRunAll":"true"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/Camera_RT
python tools/preflight.py --runtime
```

## 启动功能入口

只有两级预检均无 `BLOCKED` 后才运行。窗口中按 `s` 保存 `artery_path.txt`；Esc 保存后
退出，`q` 不保存并退出。

```bash {"name":"start-camera-rt","excludeFromRunAll":"true","interactive":"true","background":"false","closeTerminalOnSuccess":"false"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/Camera_RT
python cliff_demo.py
```
