# 相机手眼标定

## 标定入口

在仓库根目录激活 `camera` 环境后运行：

```bash
# RealSense / D455，全局相机，眼在手外
python infer/hand_eye_calibration-main/calibrate_realsense.py

# Gemini 305，腕部相机，眼在手上
python infer/hand_eye_calibration-main/calibrate_gemini.py
```

默认人工采集 50 组后自动求解。程序只读取机器人状态，不自动移动机械臂，不覆盖部署外参。
保持静止且完整检测棋盘后按 `s` 保存；采满后自动求解，`q` / Esc / Ctrl+C 提前结束不求解。

| 相机 | 默认棋盘配置 | 内部角点 | 格长 | 固定关系 | 输出 |
| --- | --- | --- | --- | --- | --- |
| RealSense | `config.yaml` | 9×6 | 0.00746 m | 相机固定、棋盘随 ArmTip 运动 | `T_base_camera` |
| Gemini 305 | `config_gemini305.yaml` | 9×6 | 0.025 m | 相机随 ArmTip 运动、棋盘固定在 Base | `T_armtip_camera` |

两个入口均支持 `--preview-only`（只预览）、`--data-dir <完整批次目录>`（只离线求解）、
`--config <路径>`、`--samples <数量>`、`--robot-ip <IP>`、`--robot-port <端口>` 和
`--output-dir <新目录>`。RealSense 用 `--camera-serial` 选设备，Gemini 用 `--camera-device`
选彩色节点（默认 `/dev/video16`）。全部参数见各入口的 `--help`。

## 数据与结果

采集保留原始 JPG、严格配对的 `poses.txt`（Base 下 ArmTip，m/rad，Rz*Ry*Rx）、
本轮 `config.yaml` 快照和 `session.json`。离线模式默认使用批次快照，不自动选择历史批次。
RealSense 结果保存为 `eye_to_hand_result.json`，Gemini 保存为 `eye_in_hand_result.json`。
结果包含重投影误差、拟合一致性和输入摘要；这些指标不代替独立验证。

操作步骤见 [标定 notebook](../../.runme/run/calibration-operations.md)。

## 模块与离线检查

内部模块：`calibration_entry.py` 负责流程，`collect_data.py` 负责采集，
`calibration_samples.py` 负责输入校验和位姿转换，`calibration_solver.py` 负责两种手眼求解。
旧的独立求解及 CSV 中间转换程序已移除。`run_probe_target.py` 和 `transform_point.py`
用于定位与坐标转换，保留供现有定位链路使用。

离线测试（不打开硬件）：

```bash
python -B -m unittest discover -s infer/hand_eye_calibration-main/tests -v
```
