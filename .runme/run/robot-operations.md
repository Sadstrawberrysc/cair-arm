# RM75 启动与运行（Runme notebook）

> 本文件包含真机运动入口。所有连接设备、常驻进程和运动代码块均应逐个执行，
> 不得对本 notebook 使用 `runme run --all`。

> 无参数运行 `./main_rm75` 是真机运动模式，执行前确认探头悬空、急停可用。

## 1. 构建 main_rm75

```bash {"name":"build-main-rm75","excludeFromRunAll":"true"}
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release
cmake --build infer/Robot/build --target main_rm75
```

控制律、扫描、planner 和硬件安全门不提供命令行覆盖；它们由
`Rm75ControlConfig`、`Rm75ServoPlannerConfig`、`Rm75RuntimeSafetyConfig` 唯一持有。
命令行只配置设备、路径、模式、Redis、tare 和进程生命周期参数。

## 2. 启动 Redis

```bash {"name":"ensure-redis-7777","excludeFromRunAll":"true"}
if [[ "$(redis-cli -h 127.0.0.1 -p 7777 ping 2>/dev/null)" != PONG ]]; then
  redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
fi
redis-cli -h 127.0.0.1 -p 7777 ping
```
## 3. 启动 main_rm75

```bash {"name":"start-main-rm75","excludeFromRunAll":"true","interactive":"true","background":"false","closeTerminalOnSuccess":"false"}
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./main_rm75
```

停止：在终端按 `Ctrl+C`。

## 4. 启动视觉程序

```bash {"name":"start-ultrasound-inference","excludeFromRunAll":"true","interactive":"true","background":"false","closeTerminalOnSuccess":"false"}
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

```bash {"name":"start-sensor-monitor","excludeFromRunAll":"true","interactive":"true","background":"false","closeTerminalOnSuccess":"false"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh && conda activate carotid && LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 python /home/cair-jacen/uspilot_ctrl-main/infer/SensorMonitor/main.py
```

直连模式独占串口。如果之后要启动 `main_rm75`，先退出监视器，启动 `main_rm75`，再重新
运行上述同一条监视命令。

## 6. 停止顺序

> 视觉窗口按 `t`（结束本轮）或 `q`（terminate 并退出），再在 `main_rm75` 终端按
> `Ctrl+C`。
# 腕部投影 observe 扩展（2026-09-15）

新增 `--wrist-projection-calibration FILE --wrist-global-calibration FILE`，
仅允许与 `--observe`、启用 Redis、`--publish-every 1` 或 `2` 一起使用。
启动前要求 `robot:wrist:seed:v1` 有本轮成功粗定位产生的有效种子且标定摘要匹配。
此选项只发布相机位姿用于投影，不启用腕部跟踪或 ServoJ。

完整先后顺序、硬件运行前置条件与相机入口见
[腕部投影操作说明](camera-wrist-operations.md)。

## 腕部点击跟随模式（2026-09-16）

最小联动入口改为 Gemini 画面左键点击，不需要全局相机、数字人或60秒seed。
完整独立观察、Redis观察、dry-run和有条件执行命令见
[腕部点击启动说明](camera-wrist-operations.md)，
[接口](../../infer/Camera_wrist/ARCHITECTURE.md)。
Robot 显式参数为 `--wrist-follow FILE --publish-every 1`，结合 `--observe` 或
`--dry-run-control`；纯离线测试使用 `--simulate --redis-enabled`。
真实执行另需 `--execute-wrist-follow --confirm-wrist-follow` 和现场确认、独立标定验收。
原显式模式5mm/5°总运动包络与全部安全门保留；不要同时启动其他机器人运动控制者。
