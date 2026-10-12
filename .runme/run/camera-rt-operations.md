# 全局相机 → 机械臂初始点（Runme notebook）

直接运行下面的命令：打开全局相机，保存采样点并退出后，机械臂自动运动到初始位姿，默认速度 3。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"camera-rt-move-to-initial-pose"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main
python infer/hand_eye_calibration-main/run_probe_target.py
```

相机窗口中按 `s` 保存，再按 `q` 退出；或按 Esc 保存并退出，随后自动执行一次真实 MoveJ_P。
目标取画面最下方采样点，转换到 Base 后生成短轴姿态，并沿 Tool -Z 后退 50 mm。
默认机器人地址为 `192.168.50.254:8080`。本流程无需 Redis，也无需另开 Robot 控制进程；
启动前关闭占用同一相机或控制同一机械臂的其他程序。

仅在首次运行或更新 Robot 源码后需要构建：

```bash {"excludeFromRunAll":"true","name":"camera-rt-build-arm-probe-pose"}
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target arm_probe_pose -j2
```
