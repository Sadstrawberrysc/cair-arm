# RM75 构建与启动命令

> 无参数运行 `./main_rm75` 是真机运动模式，执行前确认探头悬空、急停可用。

## 1. 构建 main_rm75

```bash
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release
cmake --build infer/Robot/build --target main_rm75 robot_offline_tests
```

构建后可先运行纯离线 CTest；这些测试不会连接 Redis、串口或机械臂：

```bash
ctest --test-dir infer/Robot/build --output-on-failure
```

CTest 覆盖有效配置、parser、freshness、基础控制/planner 拒绝边界和运行 schema，但不能替代后续
dry-run、标定或真机验收。

控制律、扫描、planner 和硬件安全门不提供命令行覆盖；它们由
`Rm75ControlConfig`、`Rm75ServoPlannerConfig`、`Rm75RuntimeSafetyConfig` 唯一持有。
命令行只配置设备、路径、模式、Redis、tare 和进程生命周期参数。

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

监视器自动选择数据源：单独运行时直连 `/dev/ttyUSB0`，显示 Sensor-frame 原始六轴
wrench；在 `main_rm75` 已启动后运行时，自动读取 Redis，原始值用实线、Tool-frame 补偿值用虚线。

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh && conda activate carotid && LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 python /home/cair-jacen/uspilot_ctrl-main/infer/SensorMonitor/main.py
```

直连模式独占串口。如果之后要启动 `main_rm75`，先退出监视器，启动 `main_rm75`，再重新
运行上述同一条监视命令。

## 6. 停止顺序

```text
视觉窗口按 t（结束本轮）或 q（terminate 并退出）→ main_rm75 终端按 Ctrl+C
```
