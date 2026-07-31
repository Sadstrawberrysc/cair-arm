# RM75 构建与启动命令

> 无参数运行 `./main_rm75` 是真机运动模式，执行前确认探头悬空、急停可用。

## 1. 构建 main_rm75

```bash
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release
cmake --build infer/Robot/build --target main_rm75
```

## 2. 启动 Redis

```bash
redis-cli -h 127.0.0.1 -p 7777 ping
# 没有返回 PONG 时再启动
redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
```
## 3. 启动 main_rm75

```bash
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./main_rm75
```

停止：在终端按 `Ctrl+C`。

## 4. 启动视觉程序

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
cd /home/cair-jacen/uspilot_ctrl-main/intergrate_infer
python main_redis_seg_newphase_recovery_mode.py
```

视觉窗口：先按 `b` 启动接近和 Tool-Y，再按 `m` 启动 Tool-X 扫描；视觉模型在
phase=1 时才请求 RZ 对齐。按 `t` 结束本轮并回到 idle，按 `q` 发布 terminate 后退出。

## 5. 启动传感器监视

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh && conda activate carotid && LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 python /home/cair-jacen/uspilot_ctrl-main/infer/SensorMonitor/main.py
```

## 6. 停止顺序

```text
视觉窗口按 t（结束本轮）或 q（terminate 并退出）→ main_rm75 终端按 Ctrl+C
```
