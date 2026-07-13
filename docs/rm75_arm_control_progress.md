# Realman RM75 机械臂控制进度

## 当前状态

RM75 七轴控制主线已经打通，当前统一使用：

- 主程序：`infer/Robot/build/main_rm75`
- 主程序源码：`infer/Robot/src/arm_servoj_line_test.cpp`
- 通信封装：`infer/Robot/src/realman_command.cpp`
- 运动学模型：`infer/Robot/src/realman_kinematics.cpp`
- 控制方式：固定周期 streaming ServoJ
- 控制器地址：`192.168.50.254:8080`

辅助程序：

- `arm_read_state`：只读七轴关节状态和末端 6D 位姿。
- `arm_preset_pose`：通过 MoveJ 移动到指定七轴关节姿态。
- `test_rm75_precision.sh`：依次测试 X/Y/Z 负向平移、组合平移和纯姿态旋转，并汇总精度。

## 技术路径

```text
目标位置/姿态
    -> 位姿插值
    -> RM75 FK 与 6x7 Jacobian
    -> 阻尼伪逆数值 IK
    -> 关节步长、限位与奇异风险检查
    -> 固定周期发送 ServoJ
    -> 周期读取真机状态
    -> 终点保持与误差验证
    -> 自动保存 CSV 和 SVG
```

已完成的关键步骤：

1. 通过 TCP JSON 协议连接控制器，并完成七轴状态读取。
2. 将原六轴关节接口改为 RM75 七轴接口，验证七轴 MoveJ。
3. 使用 RM75 MDH 参数建立 FK，真机多姿态测试的最大位置误差约为 `1.13e-5 m`。
4. 建立 `6x7` Jacobian，并用真机中心差分结果验证模型。
5. 使用阻尼伪逆求解每周期关节增量，实现位置和姿态的数值 IK。
6. 完成相对位置、相对姿态和绝对 6D 位姿控制。
7. 将分段运动升级为固定周期 streaming ServoJ。
8. 加入关节限位、RM75 四类奇异位形检查、速度限制和 dry-run。
9. 加入终点保持、轨迹记录和 C++ SVG 离线绘图。

## 主程序用法

先进入构建目录：

```bash
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
```

相对位置运动，单位为厘米：

```bash
./main_rm75 --delta-cm "-5,0,0"
./main_rm75 --delta-cm "-5,0,0" --execute
```

相对姿态运动，单位为度：

```bash
./main_rm75 --delta-rotation-deg "0,0,-5"
./main_rm75 --delta-rotation-deg "0,0,-5" --execute
```

绝对 6D 位姿控制：

```bash
./main_rm75 \
  --target-position-cm "40,20,45" \
  --target-rotation-deg "96,71,97"

./main_rm75 \
  --target-position-cm "40,20,45" \
  --target-rotation-deg "96,71,97" \
  --execute
```

未提供的位置或姿态字段会保持当前值。所有新目标都应先 dry-run，确认没有限位、奇异或路径风险后再添加 `--execute`。

## 当前默认参数

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `period_ms` | `20 ms` | ServoJ 发送周期 |
| `feedback_every` | `2` | 每两个周期读取一次真机状态 |
| `final_hold_ms` | `2000 ms` | 终点继续发送目标，降低跟踪滞后 |
| `max_tcp_speed_cm_s` | `3 cm/s` | 末端平移速度上限 |
| `max_total_delta_cm` | `100 cm` | 单次位置变化上限 |
| `max_rotation_delta_deg` | `180 deg` | 单次姿态变化上限 |
| `damping` | `0.001` | Jacobian 阻尼伪逆系数 |
| `joint_limit_warning_deg` | `10 deg` | 距关节限位的预警距离 |
| `joint_limit_stop_deg` | `3 deg` | 距关节限位的停止距离 |
| `singularity_warning_deg` | `5 deg` | RM75 奇异位形预警阈值 |

关节速度默认采用 RM75 官方上限：J1/J2 为 `180 deg/s`，J3-J7 为 `225 deg/s`。默认使用线性位姿插值；`--interpolation s-curve` 和 `--max-joint-accel-deg-s2` 仅保留作调参实验，当前稳定基线不启用。

## 编译与状态读取

重新编译：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
infer/Robot/tools/build_main_rm75.sh
```

读取一次机械臂状态：

```bash
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./arm_read_state --repeat 1
```

移动到已检查的关节姿态：

```bash
./arm_preset_pose \
  --target-deg "40,55,0,60,0,60,70" \
  --allow-multistep \
  --max-joint-delta-deg 15 \
  --execute
```

## 轨迹记录与精度测试

执行 `main_rm75 --execute` 后，程序自动生成：

```text
logs/main_rm75_时间戳/trajectory.csv
logs/main_rm75_时间戳/trajectory.svg
```

SVG 左侧为三维目标、模型和真机轨迹；右侧为横向偏差随时间变化，单位为 `mm`。横向偏差适合衡量轨迹抖动，不包含沿运动方向的跟踪滞后。

指定记录路径：

```bash
./main_rm75 \
  --delta-cm "-5,0,0" \
  --trajectory-log logs/x_neg_5cm.csv \
  --execute
```

运行完整精度测试：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
infer/Robot/tools/test_rm75_precision.sh
```

测试结果保存在 `infer/Robot/build/logs/precision_时间戳/`，包括每项测试的 CSV、SVG、终端输出和 `summary.csv`。

## 当前结论与下一步

- 七轴通信、FK、Jacobian、数值 IK 和 streaming ServoJ 已完成真机验证。
- 小范围位置闭环的终点误差已达到毫米以下，周期丢失通常较少；实际结果仍受姿态、方向和奇异程度影响。
- 线性插值是目前更稳定的默认方案。此前五次 S 曲线在现有外层 IK 与 ServoJ 组合下出现更明显抖动，因此暂不作为默认值。
- 轨迹图应以真机反馈红线和横向偏差为主要判断依据；模型蓝线仅表示内部运动学预测。
- 下一步使用同一安全初始姿态完成 X/Y/Z、组合平移和纯姿态对比测试，再根据 `summary.csv` 定位方向相关抖动。
- 后续优化重点是降低 Jacobian 条件数较差区域的关节增量波动，并评估更稳定的阻尼、自适应速度和冗余自由度零空间策略。

## 安全注意事项

- `--execute` 会真实驱动机械臂；dry-run 不会发送运动命令。
- 出现 `Target joint Jx exceeds RM75 joint limits` 时，应调整目标或初始姿态，不应直接放宽限位。
- 出现 `rm75_singularity_warning` 时，应先移动到远离奇异点的关节姿态。
- 大位移和大角度测试应逐级增加，并保持急停可用。
- 不要同时使用示教器、其他 JSON 客户端和本程序发送控制命令。
