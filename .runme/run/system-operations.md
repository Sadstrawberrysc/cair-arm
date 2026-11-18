# 全局定位 → 腕部跟随 → 超声扫描

两种一键入口都会驱动真机，任选一个。先退出其他机械臂控制程序和直连力传感器串口的程序。分步单元与一键流程互斥，禁止 Run All。

## 一键运行

__默认 MoveJ_P__　在全局相机窗口按 `s` 保存本轮采样后按 `q` 退出，或按 Esc 保存并退出。退出后自动移动至沿首点外法向后退 100 mm 的初始点。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-scan-workflow","promptEnv":"never"}
python3 /home/cair-jacen/uspilot_ctrl-main/scripts/run_scan_workflow.py
```

**Collision 关节路径**　核对颈动脉图像首点与点云红点，在点云窗口按 P 规划、E 执行。执行前按固定快照复查路径，运动中新出现的障碍物不会触发相机避障停止。到位验收并确认静止后自动关闭双窗口，进入腕部阶段。真机验收待完成。见[执行说明](../../infer/Collision/EXECUTION.md)。

```bash {"excludeFromRunAll":"true","interactive":"true","name":"start-scan-workflow-collision"}
python3 /home/cair-jacen/uspilot_ctrl-main/scripts/run_scan_workflow.py --collision
```

1. 腕部 Robot 就绪后自动选点并跟随。在腕部窗口按 `p` 暂停、`b` 重选并开始。确认定位完成且探头悬空后按 `q`，脚本等待腕部 Robot 正常退出，再启动超声 Robot 和推理。
2. 保持探头悬空静止，等待 3 秒 tare 和预检通过。在超声窗口按 `b` 接近并居中，再按 `m` 开始扫描。再次按 `m` 关闭扫描，再次按 `b` 停止推理和接近控制。
3. 扫描完成后在超声窗口按 `q`，脚本停止超声 Robot。中途中止整条流程可在启动终端按 `Ctrl+C`。若终端报告进程未退出，先检查该进程和机械臂状态。

## 分步运行

1. __全局定位。__ 执行后按上述默认 MoveJ_P 方式采点，等待移动命令结束。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"camera-rt-move-to-initial-pose"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main
python infer/hand_eye_calibration-main/run_probe_target.py
```

2. **启动 Redis。** 一键流程会自行检查 7777 端口；分步运行时执行此单元。

```bash {"excludeFromRunAll":"true","name":"start-redis-7777"}
if [[ "$(redis-cli -h 127.0.0.1 -p 7777 ping 2>/dev/null)" != PONG ]]; then
  redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
fi
```

3. **启动腕部跟随。** 此单元分别打开腕部相机和 Robot 终端。

```bash {"excludeFromRunAll":"true","interactive":"true","name":"start-wrist-system","promptEnv":"never"}
bash -c 'launcher=/home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh; bash "$launcher" camera && bash "$launcher" robot'
```

4. **切换控制器。** 定位完成且探头悬空后，在腕部窗口按 `q`，再到腕部 Robot 终端按 `Ctrl+C`。等待进程正常退出，保持探头悬空静止，再启动超声 Robot。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-main-rm75"}
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./main_rm75
```

5. **启动超声推理。** 在另一终端运行，预检通过后按上述 `b`、`m` 顺序扫描。按 `q` 退出后，在超声 Robot 终端按 `Ctrl+C` 停止控制器。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-ultrasound-inference"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
cd /home/cair-jacen/uspilot_ctrl-main/intergrate_infer
python main_redis_seg_newphase_recovery_mode.py
```

## 居中与录像

改变设备成像深度时，同步更新推理入口的 `image_total_height_mm`（默认 37 mm）。按 `m` 后，普通 X 扫描与 RZ 对齐还需有效图像中的横向误差 ≤0.8 mm；图像超过 300 ms 未更新时，横向运动暂停，Z 力控继续。

模型加载和 idle 握手完成后即开始录像。终端 `[Recording]` 给出本轮输出目录；用 `frames.csv` 核对超声与力的采样时刻，两段视频按本机时钟配对。详见[配对录像](../../intergrate_infer/ARCHITECTURE.md#配对录像)。
