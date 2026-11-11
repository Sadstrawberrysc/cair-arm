# 相机手眼标定

在仓库根目录激活 `camera` 环境，按相机运行：

```bash
python infer/hand_eye_calibration-main/calibrate_realsense.py
python infer/hand_eye_calibration-main/calibrate_gemini.py
```

D455 固定、棋盘随末端移动；Gemini 随末端移动、棋盘固定。棋盘均为 9×6 内角点，
格长分别为 7.46 mm 和 25 mm，由 `config.yaml`、`config_gemini305.yaml` 配置。
显示 `FOUND` 后按 `s` 保存，默认 50 组后自动求解并部署；`q` 退出。

`calibration.py` 共用采集和 linear 求解。D455 使用 RealSense SDK，Gemini 使用 Orbbec SDK 同开 RGB-D，
标定使用彩色原图。机器人反馈为 Base 下 ArmTip 位姿，位置单位 m、角度单位 rad。
每轮图像、位姿和结果保存在 `data/<camera>_<时间戳>/`。

部署文件：D455 为 `d455_to_rm75_base.json`（Camera → Base），Gemini 为
`../Camera_wrist/gemini305_to_rm75_armtip.json`（Camera → ArmTip）。
`run_probe_target.py` 和 `transform_point.py` 继续供全局定位使用。

支持 `--samples`、`--config`、`--robot-ip`、`--robot-port`、`--camera-serial`；
`--data-dir <目录>` 对已有采集离线重算并部署。完整启动命令见
[标定操作](../../.runme/run/calibration_camera.md)。
