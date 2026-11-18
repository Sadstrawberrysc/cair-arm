# 全局相机 → 机械臂初始点（Runme notebook）

本入口会执行真实单次 MoveJ_P，默认速度 6。先确认外参、Tool 标定和运动净空。
需要只读预览时，在命令末尾加 `--dry-run`；该模式检查几何臂展上界，不验证完整控制器 IK 或可达性。

## 采样与定位

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"camera-rt-move-to-initial-pose"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main
python infer/hand_eye_calibration-main/run_probe_target.py
```

相机窗口中按 `s` 保存，再按 `q` 退出；或按 Esc 保存并退出，随后自动执行一次真实 MoveJ_P。
目标取画面最上方采样点，转换到 Base 后生成短轴姿态，并沿 Tool -Z 后退 100 mm，与 Collision 目标距离一致。
默认机器人地址为 `192.168.50.254:8080`。本流程无需 Redis，也无需另开 Robot 控制进程；
启动前关闭占用同一相机或控制同一机械臂的其他程序。

## 可选：使用 Collision 规划并执行

此入口同时显示颈动脉图像和点云，由 C++ 独占 D455 并共享图像给识别程序。
颈动脉首点匹配到当前过滤后点云的最近点，显示为红点；按 P 使用红点规划，E 执行，空格停止。
不需要先保存 TXT；到位验收并确认静止后自动关闭双窗口，执行失败不回退 MoveJ_P，也不交接下一阶段。
单独运行本入口只完成粗定位；一键流程 scripts/run_scan_workflow.py --collision 会在清理完成后自动启动腕部阶段。
E 使用原始路点 MoveJ，关闭交融；monitor.example.json 的 movej_velocity_percent 默认 6%。
E 执行前用固定快照复查完整路径；执行中不进行相机场景监督或续期，新增／移动障碍物不会触发相机避障停止。
若在途 MoveJ 中断或协议失败，退出并重启本命令，重新按 P／E；不能沿用中断会话继续发送。
需已构建 Collision，运动约束与窗口用法见 [执行说明](../../infer/Collision/EXECUTION.md)。

```bash {"excludeFromRunAll":"true","interactive":"true","name":"camera-rt-collision-coarse"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main
python infer/hand_eye_calibration-main/run_probe_target.py --collision
```

加 `--dry-run` 仅规划，不启用运动。
更短的同源双窗口命令是 `./infer/Collision/start_monitor.sh --carotid --execute`，去掉 --execute 为只读。

## 构建定位工具

以下构建用于原 MoveJ_P 工具及共享运动锁。

首次运行或更新 Robot 源码后构建：

```bash {"excludeFromRunAll":"true","name":"camera-rt-build-arm-probe-pose"}
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target arm_probe_pose -j2
```

## 预检与离线验证

静态预检和离线测试不启动设备；宿主运行预检会检查 GPU、RealSense 内核及 SDK 设备访问。

```bash {"excludeFromRunAll":"true","name":"camera-rt-preflight","ignore":"true"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/Camera_RT
python tools/preflight.py --static
python -m unittest tools/test_path_format.py tools/test_sample_order.py tools/test_depth_projection.py
python tools/preflight.py --runtime
```
