# 力传感器自动标定

先停止其他 Robot 控制进程和串口直连程序，确保探头悬空、运动范围有净空。
命令会驱动真实机械臂，默认在全关节范围选取 1000 个姿态；不默认启用高度或现场避障约束。

```bash {"name":"calibrate-force-static","excludeFromRunAll":"true","interactive":"true"}
python3 /home/cair-jacen/uspilot_ctrl-main/infer/Robot/tools/calibrate_force_static.py
```

- 默认连接 RM75 `192.168.50.254:8080`、KWR57 `/dev/ttyUSB0`（460800 波特率），运动速度为 5%。
- 自动换姿态，每姿态采集至少 20 帧并取平均，采满后拟合；质量检查通过才更新部署标定，沿用现有安装几何。
- 样本保存到 `infer/Robot/build/calibration_samples.csv`，标定写入 `infer/Robot/build/rm75_force_calibration.json`。
- `Ctrl+C` 取消并请求停止；恢复时在同一命令末尾加 `--resume`，沿原路径继续最新非空断点。

已有标定结果与历史任务见 [力传感器进展](../Progress/force-sensor-progress.md)。
