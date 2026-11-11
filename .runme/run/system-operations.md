# 全局定位 → 腕部跟随 → 超声扫描

## 一键串联运行

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-scan-workflow","promptEnv":"never"}
python3 /home/cair-jacen/uspilot_ctrl-main/scripts/run_scan_workflow.py
```

执行一次后自动衔接各阶段，在同一终端显示进度；按键仍在对应图像窗口操作：

1. 全局相机：按 `s` 保存、`q` 退出，或 Esc 保存并退出，自动移动到初始点。
2. 腕部相机：Robot 就绪后自动选点并跟随；定位完成且探头仍悬空时按 `q`。
   脚本停止腕部 Robot，等待正常退出，再启动超声 Robot 与推理程序。
3. 超声窗口：等待 Robot 的 3 秒悬空 tare 与预检通过，再按 `b` 接近、居中，按 `m` 扫描。
   完成后按 `q`，脚本自动停止超声 Robot 并结束。

超声 Robot 的 3 秒悬空启动预检要求：应用本次 tare 前，最大补偿合力 ≤4 N、
最大补偿合力矩 ≤0.1 N·m；同时保留静止、采样健康和力稳定性检查。

启动前停止其他机器人控制程序和串口直连程序。终端按 `Ctrl+C` 中止整条流程；
阶段异常或 Robot 未正常退出时停止后续启动。Redis 会复用或自动启动，结束后保留。
本入口根据窗口退出进行切换，腕部到位及扫描完成仍由操作者判断。

下面保留分步命令，需要单独操作时使用；不要与一键入口同时运行，不要使用 Run All。

## 1. 全局相机定位并移动到初始点

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"camera-rt-move-to-initial-pose"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main
python infer/hand_eye_calibration-main/run_probe_target.py
```

相机窗口按 `s` 保存、`q` 退出，或按 Esc 保存并退出；随后自动执行一次真实 MoveJ_P，
到达距表面 50 mm 的初始点。等待命令结束后继续。

## 2. 启动 Redis

```bash {"excludeFromRunAll":"true","name":"start-redis-7777"}
if [[ "$(redis-cli -h 127.0.0.1 -p 7777 ping 2>/dev/null)" != PONG ]]; then
  redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
fi
```

## 3. 启动腕部相机与跟随

执行一次，依次打开腕部相机和 Robot 的独立终端。

```bash {"excludeFromRunAll":"true","interactive":"true","name":"start-wrist-system","promptEnv":"never"}
bash -c 'launcher=/home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh; bash "$launcher" camera && bash "$launcher" robot'
```

Robot 就绪后自动模型选点并跟随；腕部相机窗口按 `b` 重选并开始，`p` 暂停。

## 4. 结束腕部跟随，切换到超声控制

腕部定位完成后，在相机窗口按 `q` 退出，再在腕部 Robot 终端按 `Ctrl+C`，
等待进程结束。当前腕部入口为无力悬停试验，尚未实现自动交接；两种 Robot 入口不能同时运行。

确认探头悬空，供启动时进行 3 秒 tare，再在独立终端执行：

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-main-rm75"}
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./main_rm75
```

## 5. 启动超声并完成扫描

另开终端执行：

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-ultrasound-inference"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
cd /home/cair-jacen/uspilot_ctrl-main/intergrate_infer
python main_redis_seg_newphase_recovery_mode.py
```

在超声图像窗口中操作：

- `b`：启动推理、接近控制与 Tool-Y 居中；再次按下停止，每次切换都会关闭扫描。
- `m`：接近并居中后开启 Tool-X 扫描与 Tool-RZ 对齐（RZ 仅在 phase=1 时请求）；再次按下关闭。
- `t`：切换终止标志，结束本轮；再次按下取消该标志。
- `s` / `y`：分别切换分割 / YOLO 显示。
- 扫描完成后按 `q` 发送终止命令并退出超声程序，再在 Robot 终端按 `Ctrl+C` 结束。

## Tool-Y 居中状态

按 `b` 后先沿当前可见血管中心做 Y 居中和 Z 力控；按 `m` 后，图像横向误差 ≤0.8 mm
才继续 X 扫描 / RZ 对齐。超声窗口保持原显示：`y:` / `On Track`、方向箭头及既有
phase、RZ、Recovery、分割和 YOLO 叠加；不新增中心线、选中目标标记或等待状态文字。
未识别到血管不会继续用回归模型猜测横移；原丢失恢复仍保留。
图像超过 300 ms 未更新时心跳停止 Y/X/RZ，保留 Z 力控；新鲜目标返回后重新确认。
居中换算沿用成像高度 37 mm，设备成像深度改变时需同步修改超声入口的 `image_total_height_mm`。
无需新增日志文件；已有力保护仍可能在接触力不足或卸载时暂停横移。

## 超声与 tare 后六维力同步录像

进入超声推理循环后自动开始保存，无需另开 SensorMonitor；按 `q` 退出或流程中断时收尾。
终端 `[Recording]` 输出本次目录：
`intergrate_infer/output/scan_YYYYMMDD_HHMMSS_<id>/`。

- `ultrasound.avi`：推理输入的超声裁剪画面，30 FPS；奇数宽度 557 补一列黑边至 558。
- `wrench_tare.avi`：同时间轴的 Fx/Fy/Fz（N）、Tx/Ty/Tz（N·m）曲线，显示本次录制从 0 秒到当前时刻的全部历史，横轴随时长扩展，纵轴自动缩放。
- `wrench_tare.csv`：Redis 接收到的六维力样本及传感器 / 接收时间戳、有效性。
- `frames.csv`：两段视频共用的帧号、视频时间、主机录制时间、最近超声接收 / 力采样时间。
- `recording.json`：帧率、帧数、时间原点、单位及录制错误等元信息。

六维力订阅 `robot:sensor:v1` 的 `compensated_wrench_tool`，使用标定重力/质心补偿、
本次 tare 后的 Tool 坐标值，不在录像端再次清零。Robot 必须用本次更新后的 `main_rm75`。
缺少 tare 标记、无效或超过 500 ms 的力数据不绘为零，显示等待 / 过期并断开曲线。
后台线程独立录制两段视频；推理未交付新图像或显示冻结时保留上一幅超声图并标注 ROI 帧龄。
两段视频帧数 / 帧率一致，但这是基于本机单调时钟的配对记录，不是相机曝光与力传感器的硬件同步。
Robot 原始运行 CSV 仍保留全控制周期数据；录像曲线 CSV 的采样频率取决于 Redis 遥测发布频率。

三类记录都覆盖本次超声录制开始到结束，不受 `b/m` 开关影响；力曲线视频最后一帧包含整次历史。
长时间运行时，曲线按时间分箱保留六轴最小 / 最大值，保留早期峰值与无效区间标记；
CSV 仍逐条保存收到的数值，不因曲线显示压缩而删减。开始指超声模型加载和 idle 握手完成后的录像起点。
