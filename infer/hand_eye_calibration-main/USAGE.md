# 数字人单点到 RM75 Probe TCP

## 一次启动完整流程

完成本轮现场检查后，在 `camera` 环境中从仓库根目录运行（默认真实运动，速度1）：

```bash
python infer/hand_eye_calibration-main/run_probe_target.py
```

先打开数字人窗口；按 `s` 保存，按 `q` 退出后，入口检查本次文件已更新，跳过第一行法向量，
读取第一个采样点，即画面最下面的红点（像素 y 最大；由相机保存前排序）。用新外参计算 `p_base = R_base_camera @ p_camera + t_base_camera`，
然后调用 `arm_probe_pose --target-position-base-m`，自动传入速度1和双执行参数，通过规划门后执行单次 MoveJ。
Robot 自动保持启动时探头姿态、打印完整六维目标，不添加50 mm偏置。

若只需读取状态并检查规划，运行：

```bash
python infer/hand_eye_calibration-main/run_probe_target.py --dry-run
```

无参数命令在本轮保存并退出数字人后会进入真实 MoveJ 流程，Robot 的全部规划门仍生效。
`Ctrl+C` 中断流程；数字人异常退出、未保存或文件非法时不会调用 Robot。不自动重试运动。
数字人目前也支持 Esc 保存退出，此入口会把它视为有效的新快照。

需要构建 Robot 时：

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target arm_probe_pose -j2
```

当前外参保存在同目录 `d455_to_rm75_base.json`，定义 `T_base_camera`：把 D455 彩色光学
坐标（米）映射到 RM75 Base 坐标（米）。

先转换数字人保存的单点：

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/hand_eye_calibration-main
python transform_point.py --camera-point-m X Y Z
```

复制输出的 Base 位置，回到仓库根目录做 Robot dry-run：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
infer/Robot/build/arm_probe_pose \
  --target-position-base-m "BASE_X,BASE_Y,BASE_Z" \
  --tool-calibration infer/Robot/build/rm75_force_calibration.json \
  --velocity 1
```

Robot 会读取启动时的 ArmTip 状态，通过 `CalibratedFrameChain` 取得当前 Probe TCP 姿态，
打印完整 `Base→Probe_TCP [x,y,z,rx,ry,rz]` 并运行 IK。单点模式不添加50 mm偏置。

当前新外参由用户确认，但尚未记录独立验证结果；转换输出不是运动授权。真机执行仍需
dry-run 通过、已知点和 Base 方向现场核对、整段净空确认及明确授权。
