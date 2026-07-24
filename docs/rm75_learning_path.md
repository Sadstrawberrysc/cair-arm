# RM75 学习路径：从架构理解到可修改控制

本手册用于从零理解当前 RM75 七轴、Haptron 六维力传感器和超声视觉闭环。学习顺序固定为：**源码阅读 → 日志观察 → 真机验证**。完成一个模块的检查项后，再进入下一模块。

生产机械臂入口：

```bash
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./main_rm75 --execute
```

视觉入口：

```bash
cd /home/cair-jacen/uspilot_ctrl-main/intergrate_infer
/home/cair-jacen/anaconda3/envs/carotid/bin/python \
  main_redis_seg_newphase_recovery_mode.py
```

真实运动前必须确认探头悬空、运动方向、现场急停和可用标定。本文说明软件学习与验证路径，不替代现场安全流程。

## 0. 闭环总图

```text
超声图像
  → main_redis_seg_newphase_recovery_mode.py
  → Redis command JSON（Y、RZ、phase、action）
  → main_rm75（20 ms 控制循环）
  → rm75_control（状态机、力控、Tool-X/Y/Z/RZ 增量）
  → 七轴 IK / ServoJ
  → RM75

Haptron Modbus → 原始 wrench → 标定补偿 → rm75_control
RM75 状态反馈 ───────────────────────────┘
```

阅读时始终区分两类数据：视觉命令是“希望向哪个 Tool 方向修正”；机器人与传感器反馈是“当前真实发生了什么”。控制器使用真实反馈限制参考轨迹，不能只累计理想目标。

## 1. 工程结构与启动链路

先阅读：

1. [RM75 迁移进度](rm75_progress.md)：当前硬件、标定、运行边界和已知问题。
2. [`infer/Robot/CMakeLists.txt`](../infer/Robot/CMakeLists.txt)：构建目标与库的依赖。
3. [`infer/Robot/src/main_rm75.cpp`](../infer/Robot/src/main_rm75.cpp)：只看 `Options`、`ApplyImplicitCommissioningProfile`、启动检查、20 ms 主循环和运行摘要。

模块职责：

| 模块 | 主要职责 |
| --- | --- |
| `realman_transport` | RM75 TCP 状态与 ServoJ/Stop 通信 |
| `robot_sensor` | Haptron Modbus、标定补偿、接触点 |
| `rm75_motion` | 力控状态机、IK、ServoJ 规划 |
| `main_rm75` | 固定生产 profile、启动门、I/O 组合、日志 |

检查项：

- [ ] 能说明裸 `./main_rm75 --execute` 为什么能得到机器人 IP、串口、标定文件、Redis、目标力与速度。
- [ ] 能说明 `rm75_control.hpp` 是算法默认值，而 `main_rm75.cpp` 的生产 profile 会覆盖实际运行值。
- [ ] 能在终端输出中找到 `configuration_profile`、`desired_force_n`、`approach_speed_cm_s` 和 `runtime_log`。

## 2. RM75 通信、状态与七轴运动学

阅读顺序：

1. `infer/Robot/include/realman_command.hpp` 与 `src/realman_command.cpp`：TCP 建连、状态读取、ServoJ 和 Stop。
2. `include/realman_kinematics.hpp` 与 `src/realman_kinematics.cpp`：七轴 FK、6×7 Jacobian、数值 IK。
3. `src/arm_read_state.cpp` 与 `src/arm_preset_pose.cpp`：维护工具示例。

维护工具不在默认构建中。需要时先配置并编译：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target arm_read_state
```

只读验证：

```bash
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./arm_read_state --ip 192.168.50.254 --port 8080 --repeat 3 --interval-ms 500
```

应掌握：

- `joints7_rad` 是七关节角；Arm_Tip 位姿是机械臂末端安装面位姿；Probe TCP 位姿是安装偏移后的实际操作点位姿。
- IK 将 Arm_Tip 的期望位姿转为七关节目标；ServoJ 下发的是关节目标，不是直接下发 XYZ。
- `robot receive timed out` 属通信问题；`servo_plan_singularity` 是运动学问题；`robot_position_tracking_error_exceeded` 是实际机械臂未跟上参考轨迹。

检查项：

- [ ] 能从状态输出读出七关节、位置和欧拉角。
- [ ] 能解释为什么控制器每周期使用实际关节反馈更新目标。
- [ ] 能区分通信超时、关节限位、奇异与跟踪误差。

## 3. 六维力传感器、标定与 wrench

阅读顺序：

1. `include/force_sensor.hpp`、`src/force_sensor.cpp`、`include/haptron_modbus.hpp`：Modbus RTU 和线程安全采样快照。
2. `include/force_calibration.hpp`、`src/force_sensor_calibrate.cpp`：偏置、质量、质心和重力补偿。
3. `include/contact_sensing.hpp`、`src/contact_sensing.cpp`：从 wrench 与 STL 表面估计接触点。

数据链：

```text
原始 wrench [Fx,Fy,Fz,Tx,Ty,Tz]
  → 零点 / 重力 / 工具质量与质心补偿
  → Tool/Sensor 坐标 wrench
  → 接触点估计与接触姿态控制
```

其中 force 的单位为 N，torque 的单位为 N·m。`tare` 是探头完全悬空、静止时采集的现场零点；它只能消除当前姿态和安装条件下的静态偏置，不能替代正式多姿态重力标定。

真机观察：生产入口启动时会执行 3 秒悬空 tare。终端应出现 `runtime_tare_sensor`、`runtime_tare_samples` 与最大偏差。若被残差门拒绝，先检查悬空、姿态、线缆受力和标定，而不是直接开始闭环。

检查项：

- [ ] 能解释 Fx/Fy/Fz、Tx/Ty/Tz、tare、重力补偿、工具质量和质心。
- [ ] 能区分残差门、串口/Modbus 异常、数据陈旧和标定文件不匹配。
- [ ] 能复述“原始 wrench 不等于探头接触力”。

## 4. 坐标链与 Tool-X/Y/Z

阅读顺序：

1. `main_rm75.cpp` 中 `rotation_pose_from_tool`、`probe_tcp_tool_m` 和 Arm_Tip/Tool 配置。
2. `rm75_control.cpp` 中 `translation_delta` 与最终 `desired_pose` 合成位置。
3. 当前标定 JSON 中的 `rotation_tool_from_sensor_row_major`、`probe_tcp_sensor_m`。

坐标链：

```text
Base → Arm_Tip → Tool/Sensor → Probe TCP
```

当前控制请求在 Tool/Sensor 坐标系表达；当前安装中 Tool/Sensor 相对 Arm_Tip 约有绕 Z 轴 30° 固定旋转；最终 IK 使用 Arm_Tip 目标。因此 `-Tool-X` 在 Base 视角通常是斜向，并不代表扫描方向错误。

真机观察：分别观察 Tool-Z 接近、Tool-Y 居中和 Tool-X 扫描，并同时记录 Base 视图和探头自身坐标轴方向。

检查项：

- [ ] 能解释 `-Tool-X` 不等于 `-Base-X`。
- [ ] 能判断方向错误应优先检查视觉符号、Tool/Arm_Tip 固定旋转，还是增量合成代码。
- [ ] 能解释 TCP 0.188 m 偏移为何会影响位置与力矩。

## 5. Z 向力控、超力卸载与恢复

阅读顺序：

1. `rm75_control.hpp`：目标力、接触阈值、虚拟质量/阻尼、卸载和恢复参数。
2. `rm75_control.cpp`：接触判定、Z 导纳、`target_force_unloading`、制动、重新接触和恢复分支。
3. `main_rm75.cpp`：写入 `control_config` 的生产参数。

当前 production profile 的关键值：

```text
目标力：-3 N
接触门：|Fz| ≥ 0.99 N
虚拟质量 M：3
虚拟阻尼 D：20
普通 Tool-Z 力控上限：0.05 cm/s
超力开始卸载：Fz ≤ -3.5 N
```

控制含义：未接触时沿 `+Tool-Z` 接近；接触后导纳使 Fz 向 -3 N 收敛；压得更深、Fz 更负时请求 `-Tool-Z` 卸载。卸载与恢复期间 X/Y/RZ 会被抑制，避免横向或旋转与卸载相互干扰。

真机练习顺序：按 `b` 后观察未接触接近；轻接触后观察向 -3 N 收敛；施加更大压缩力，确认仅轴向卸载；释放后观察制动、重新接触与普通导纳恢复。

检查项：

- [ ] 能从 CSV/summary 判断普通导纳、卸载、恢复哪个分支正在运行。
- [ ] 能解释 Tool-X 卡顿常来自 `force_settle`、卸载或恢复，而不一定是 20 ms 调度问题。
- [ ] 能说明增大 `target_force_unload_margin_n` 会让卸载更晚发生。

## 6. Tool-Y 居中、Tool-X 扫描与 RZ

阅读顺序：

1. `rm75_control.cpp` 中 `visual_y_enabled_`、`model_y_velocity_m_s` 和 `model_y_step`。
2. 同文件的 `force_settle`、`scan`、`trigger_align` 和 `rotate_align` 状态转换。
3. `main_rm75.cpp` 中 Y 增益、扫描稳定力带、X 扫描速度和 RZ 参数。

当前键控行为：

```text
按 b：开始 moving；Tool-Z 接近/力控，Tool-Y 视觉居中。
按 m：保持 Tool-Y 居中；在稳定接触后开始 -Tool-X 连续扫描；允许视觉 phase 驱动 RZ。
按 t：请求结束本轮扫描并回到 idle。
按 q：发布 terminate 并退出视觉端。
```

Tool-Y 当前为比例速度控制：

```text
v_y = model_y_direction × model_y_velocity_gain_per_s × visual_y_error
Δy = v_y × 0.02 s
```

当前生产 profile 中符号为 `-1`，增益为 `1.0 s⁻¹`。Y 只有在传感器已接触、视觉命令 fresh、处于 moving、且不在卸载/恢复/跟踪暂停时才会真正形成 Tool-Y 请求。

检查项：

- [ ] 按 `b` 后可由视觉画面确认颈动脉向图像中心移动。
- [ ] 按 `m` 后确认 Tool-Y 保持工作且 Tool-X 连续扫描。
- [ ] 能从 CSV 对比 `command_model_y_m`、`requested_dy_tool_m` 与实际 TCP。
- [ ] 能定位 Y 不动是视觉输出为零、Redis Hold、未接触、卸载/恢复、实际跟踪暂停还是符号错误。

## 7. Redis 协议与视觉状态机

阅读顺序：

1. `include/redis_bridge.hpp` 与 `src/redis_bridge.cpp`：命令校验、session、sequence、超时 Hold 与状态发布。
2. `intergrate_infer/main_redis_seg_newphase_recovery_mode.py`：`VisionCommandPublisher`、200 ms 心跳、`b/m/t/q`、phase 推理、防抖和命令门控。
3. `main_rm75.cpp` 中 Redis snapshot 到 `ControlIntent` 的映射。

命令核心字段：

```json
{
  "version": 1,
  "session_id": "UUID",
  "sequence": 123,
  "parameters": {"y": 0.0012, "rz": -8.5},
  "phase_idx": 0,
  "action_state": true,
  "terminate": false
}
```

协议原则：新视觉 session 先发送 idle；同一 session 的 sequence 递增；视觉端每 200 ms 心跳刷新命令。命令超过 500 ms、Redis 断线或 JSON 无效时机器人进入 Hold，不继续累计视觉目标。

phase 含义：`0=Scan`、`1=Trigger`、`2=Action`。按 `b` 的 phase 为 -1，只开始接近与 Tool-Y；按 `m` 将 phase 置为 0，使扫描状态机可进入 Tool-X；视觉稳定发布 phase 1 后才可能进入 Trigger/RZ 对齐。

检查项：

- [ ] 启动视觉端后确认先出现新 session 的 idle 握手。
- [ ] 按 `b` 后观察 `action_state=true`、`phase=-1`。
- [ ] 按 `m` 后观察 `phase=0`，命令年龄持续低于 500 ms。
- [ ] 能判断“不旋转”是视觉未发 phase 1、Trigger 的 Y 条件未满足，还是未进入稳定接触。

## 8. 日志驱动调试与修改规范

每次真机运行结束后，优先检查最新的同名 CSV 和 `.summary.json`。不要在 `main_rm75` 仍运行时清理 `infer/Robot/build/logs/`。

重点字段：

| 文件 | 优先检查项 |
| --- | --- |
| `summary.json` | `result`、`fault_code`、`planned_scan_distance_mm`、卸载/恢复周期、Y 跟踪暂停周期、`missed_periods` |
| CSV | `state`、`command_model_y_m`、`requested_dx/dy/dz_tool_m`、`control_fz_n`、`command_fresh`、卸载/恢复标记、实际 TCP 位姿 |

修改顺序：

1. 只调生产参数时，改 `main_rm75.cpp` 的 implicit profile 或 `control_config` 赋值。
2. 要改变公式、状态转换或门控时，才改 `rm75_control.cpp/.hpp`。
3. 每次只改一个控制维度或一个条件；构建后保存对应日志；确认结果再进行下一项。
4. 修改后构建唯一生产目标：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
cmake --build infer/Robot/build --target main_rm75
```

最终能力检查：

- [ ] 能从日志确认 X/Y/Z/RZ 是否既产生了请求，也被实际 TCP 跟踪。
- [ ] 能说明一个参数影响的状态、坐标系和故障门。
- [ ] 能将问题定位到视觉、Redis、传感器、控制状态机或 IK/ServoJ 五类边界之一。

## 推荐学习节奏

不要按日期赶进度。每个模块的“完成标准”全部满足时才继续：先完成 1–4，获得稳定的坐标和数据链认识；再完成 5–6，理解真实运动和力控；最后完成 7–8，能够在视觉闭环中定位并修改问题。真机每次只验证当前模块的一项行为，避免视觉、力控和坐标问题同时叠加。
