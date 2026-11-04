# 全局相机与腕部相机标定（Runme notebook）

本 notebook 只保留两部相机的标定入口，每次只运行对应单元。
入口默认采集 50 组，采满后自动求解；仅请求机器人状态，不自动移动机械臂，不修改坐标系或部署外参。
先关闭生产控制、跟随程序和占用对应相机的程序，现场确认机器人反馈为 Base 下的 ArmTip（m/rad）。

默认棋盘参数位于 `infer/hand_eye_calibration-main/`：

- 全局 D455：`config.yaml`，内部角点 9×6、格长 0.00746 m。
- 腕部 Gemini 305：`config_gemini305.yaml`，内部角点 9×6、格长 0.025 m。

使用其他棋盘前修改配置，或传入 `--config <路径>`；采集会保存本轮参数快照。

## 全局相机：RealSense / D455，眼在手外

固定全局相机，将棋盘牢固安装在机器人末端，采集期间棋盘不能相对 ArmTip 移动。
在不同位置、距离、倾角下采样，结果为 `T_base_camera`，将彩色相机坐标转换到 Base。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"calibrate-realsense"}
cd /home/cair-jacen/uspilot_ctrl-main
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
python infer/hand_eye_calibration-main/calibrate_realsense.py
```

可在上述 Python 命令末尾添加：

- `--preview-only`：仅预览棋盘，不连接机器人、不保存。
- `--camera-serial <序列号>`：选择全局 RealSense，多台设备时必须指定。
- `--data-dir <本轮目录>`：只对指定完整批次离线求解，不打开相机、不连接机器人。

默认每轮新建 `infer/hand_eye_calibration-main/data/realsense_时间戳/`；
结果写到该目录的 `eye_to_hand_result.json`。

## 腕部相机：Gemini 305，眼在手上

相机牢固安装在腕部，棋盘固定在 Base 环境中；人工移动机械臂，使相机从不同位置和倾角观察固定棋盘。
采集期间相机不能相对 ArmTip 移动，棋盘不能相对 Base 移动。
结果为 `T_armtip_camera`，将腕部彩色相机坐标转换到 ArmTip，后续需乘实时 `T_base_armtip`。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"calibrate-gemini"}
cd /home/cair-jacen/uspilot_ctrl-main
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
python infer/hand_eye_calibration-main/calibrate_gemini.py
```

可在上述 Python 命令末尾添加：

- `--preview-only`：仅预览棋盘，不连接机器人、不保存。
- `--camera-device <路径>`：默认 `/dev/video16`，Gemini 彩色 MJPG 640×480@30；
  节点变化时优先使用 `/dev/v4l/by-id/` 稳定路径，确认对应 Gemini 305 彩色节点。
- `--data-dir <本轮目录>`：只对指定完整批次离线求解，不打开硬件。

默认每轮新建 `infer/hand_eye_calibration-main/data/gemini_时间戳/`；
结果写到该目录的 `eye_in_hand_result.json`。该结果是彩色光学坐标外参，不能直接用于未对齐的深度坐标。

## 共用操作与结果

- 完整棋盘显示绿色 `FOUND` 后，机械臂保持静止，按 `s` 保存一组；需要绕非平行轴的旋转和不同位置，不能仅平移或重复同一姿态。
- 两次状态读取之间采新彩色帧；位移超过 0.2 mm 或转角超过 0.2° 不保存。此检查不是硬件同步。
- 默认采满 50 组自动关闭预览并求解；`q`、Esc 或 Ctrl+C 提前结束，不执行自动求解。
- 两入口均支持 `--robot-ip <IP>`（默认 `192.168.50.254`）、`--robot-port <端口>`（默认 8080）、
  `--samples <数量>`（至少 6，建议 50）、`--output-dir <新目录>`；不覆盖已有采集目录。
- 每轮包含编号 JPG、严格一图一行的 `poses.txt`、`config.yaml` 快照和 `session.json`。
  新入口要求完整会话且相机 backend 匹配；离线不自动选择最新或旧批次。
- 两部相机共用相机拟合和手眼求解检查；当前 OpenCV 缺少 `calibrateHandEye` 时默认 `linear`，
  有该接口时默认 `park`，可用 `--method linear` 显式指定。结果包含所用方法、内参、重投影误差、
  同批次棋盘位置/姿态一致性、有效编号及输入摘要。
- 同批次拟合残差不是独立定位误差。结果的独立验证状态为 false，应使用未参与标定的姿态或已知点验证后再部署；
  入口不覆盖全局或腕部部署外参，不叠加旧经验补偿。重新离线求解会更新同一批次的结果 JSON。
