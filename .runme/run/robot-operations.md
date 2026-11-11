# 机器人与超声启动

按顺序执行；机器人和超声分别使用独立终端。

## 1. Redis

```bash {"excludeFromRunAll":"true","name":"ensure-redis-7777"}
if [[ "$(redis-cli -h 127.0.0.1 -p 7777 ping 2>/dev/null)" != PONG ]]; then
  redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
fi
```

## 2. 机器人

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-main-rm75"}
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./main_rm75
```

## 3. 超声

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-ultrasound-inference"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
cd /home/cair-jacen/uspilot_ctrl-main/intergrate_infer
python main_redis_seg_newphase_recovery_mode.py
```

在超声图像窗口中操作：

- `b`：启动／停止推理与接近控制，启用 Tool-Y 居中；每次切换都会关闭扫描。
- `m`：先按 `b`，再按 `m` 开启／关闭 Tool-X 扫描与 Tool-RZ 对齐（RZ 仅在 phase=1 时请求）。
- `t`：切换终止标志，结束本轮；再次按下取消该标志。
- `q`：发送终止命令并退出超声程序。
- `s`：切换分割显示。
- `y`：切换 YOLO 显示。
