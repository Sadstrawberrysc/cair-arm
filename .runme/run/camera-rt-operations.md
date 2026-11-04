# 全局相机 → 机械臂初始点（Runme notebook）

本入口会执行真实单次 MoveJ_P，默认速度 3。先确认外参、Tool 标定和运动净空。
需要只读预览时，在命令末尾加 `--dry-run`；该模式不检查控制器 IK 或可达性。

## 采样与定位

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"camera-rt-move-to-initial-pose"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main
python infer/hand_eye_calibration-main/run_probe_target.py
```

相机窗口中按 `s` 保存，再按 `q` 退出；或按 Esc 保存并退出，随后自动执行一次真实 MoveJ_P。
目标取画面最上方采样点，转换到 Base 后生成短轴姿态，并沿 Tool -Z 后退 50 mm。
默认机器人地址为 `192.168.50.254:8080`。本流程无需 Redis，也无需另开 Robot 控制进程；
启动前关闭占用同一相机或控制同一机械臂的其他程序。

## 构建定位工具

首次运行或更新 Robot 源码后构建：

```bash {"excludeFromRunAll":"true","name":"camera-rt-build-arm-probe-pose"}
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target arm_probe_pose -j2
```
