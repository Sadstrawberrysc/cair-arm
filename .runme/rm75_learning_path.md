# RM75 源码与调试学习路径

先读源码，再观察日志，最后做现场验证。本文只列阅读顺序和关键检查点；启动步骤见
[Robot 操作](run/robot-operations.md)，接口和参数细节见 [Robot 架构](../infer/Robot/ARCHITECTURE.md)。
无参数 `main_rm75` 会进入真实 Execute，学习源码和离线检查不需要启动它。

## 1. 入口与配置

| 阅读入口（相对 `infer/Robot/`） | 要理解的内容 |
| --- | --- |
| [CMakeLists.txt](../infer/Robot/CMakeLists.txt) | 生产库、维护工具与离线测试的依赖 |
| [src/main_rm75.cpp](../infer/Robot/src/main_rm75.cpp) | 生命周期、10 ms 快照循环、安全监督和停止路径 |
| [include/rm75_runtime_config.hpp](../infer/Robot/include/rm75_runtime_config.hpp)、[实现](../infer/Robot/src/rm75_runtime_config.cpp) | `RobotRuntimeConfig`、`MakeImplicitRm75ProductionConfig` 和有效配置 |
| [include/rm75_control.hpp](../infer/Robot/include/rm75_control.hpp) | 控制、planner 与安全参数结构体 |

检查：能区分结构体默认 Observe 与无参数 Execute；能从启动输出 / summary 找到 profile、目标力、速度和标定路径。
默认 KWR57 为 `/dev/ttyUSB0`、460800/8N1，标定为可执行文件旁 `rm75_force_calibration.json`，要求身份 `KWR57`。

## 2. 通信与七轴规划

阅读 `realman_command.hpp/.cpp`、`realman_kinematics.hpp/.cpp` 和 `rm75_control.hpp/.cpp`：

```text
实测七轴 / ArmTip → 控制参考 → Probe TCP 目标 → ArmTip 目标 → 七轴 IK / planner
→ 有效计划 → ServoJ 单槽提交 → 后台发送与结果检查
```

- `joints7_rad` 为七关节弧度，ArmTip 是反馈安装参考点，Probe TCP 是标定后的操作点。
- `TryServoJ` 成功只表示提交成功；Stop 回执也不等于实体静止。
- 能区分通信超时、关节限位、奇异和实际跟踪误差；维护定位的控制器 MoveJ_P 与本地 planner 检查范围不同。

需要只读硬件状态时，构建并运行维护工具（仍会连接 Robot）：

```bash
cmake -S infer/Robot -B infer/Robot/build -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target arm_read_state
infer/Robot/build/arm_read_state --ip 192.168.50.254 --port 8080 --repeat 3 --interval-ms 500
```

## 3. 力觉与坐标链

阅读 `force_sensor.hpp/.cpp`、`force_calibration.hpp/.cpp`、`contact_sensing.hpp/.cpp` 和
`calibrated_frame_chain.hpp/.cpp`。KWR57 协议与无 CRC 语义见 [接入说明](../docs/kwr57_sensor_integration.md)。

```text
原始 Sensor wrench（N / N·m）→ 原始量程门 → 重力 / 零偏 / tare 补偿
→ Sensor 下接触估计 → Tool 工作轴表达 → 控制
Base → ArmTip → Sensor；Tool 工作轴由独立探头旋转确定；TCP 由几何字段确定
```

`T_A_B` 将 B 映射到 A。schema v2 分开保存 `R_A_S` 与 `R_S_T`，控制工作轴为
`R_A_T=R_A_S×R_S_T`；不能把探头转角叠加到 Sensor 重力补偿旋转。
当前装配配置是 Sensor 安装 +30°、探头相对 0°、Sensor→TCP 188 mm、
Sensor→ArmTip 平移 `[0,0,-0.0125]` m，因此 ArmTip→TCP 为 200.5 mm；实物误差仍需独立核验。

检查：能解释原始 wrench 与接触力的区别、Tool-X 与 Base-X 的区别、TCP 偏移对位置 / 力矩的影响。
tare 只消除当前姿态的固定偏置，不代替多姿态重力和几何标定。
项目通过 Redis 传递遥测。SensorMonitor 是可选独立工具：有项目遥测时自动只读订阅 Redis，否则直连串口；监控清零不改变项目输入。

## 4. 超声控制状态

阅读 `Rm75ControlLaw::Step()`，先掌握优先级，再看参数：

| 分支 | 当前行为 / 默认值 |
| --- | --- |
| idle / terminate | 结束本轮，通常回 Armed；有效超力的卸载 / 制动优先完成 |
| 接近 | +Tool-Z 20 mm/s；`abs(Fz)≥0.99 N` 锁存接触 |
| 力稳定 | 目标 -2 N，M=3、D=20，普通 Z 速度上限 2 mm/s |
| 卸载 / 恢复 | 比目标更负 1 N 持续 0.5 s 后 -Z 卸载，最高 10 mm/s；重接触最高 5 mm/s |
| Y 居中 | `v_y=-1×1.0×y`，单周期最多 0.5 mm；力门 `abs(Fz)>1.2 N` |
| X 扫描 | 稳定力带和 Y 居中合格后 -Tool-X 10 mm/s；Scan 的 Y 缩放 0.8 |
| Trigger / RZ | phase=1，Y 误差 <8 mm 持续 0.4 s 后锁存；RZ 5°/s、累计额度 120°；Y 缩放 0.3 |
| 丢失搜索 | 旋转锁存后 phase=2 且 recovery；左 -Y、右 +Y，沿旋转前锁存轴以 0.2 mm/s 搜索，最多 20 mm；保留 Z 力控 |

Y 跟踪暂停 / 恢复为 20 / 10 mm；Y 主导误差达到 5 mm 且占比 ≥80% 可重建参考，腕部模式不使用该重置。
检查 CSV 中请求、实际 TCP 与 Hold 原因，不能把“未产生 X 请求”直接归因于调度卡顿。

## 5. 视觉与 Redis

阅读 `redis_bridge.hpp/.cpp` 及
[超声主入口](../intergrate_infer/main_redis_seg_newphase_recovery_mode.py) 的
`VisionCommandPublisher`、`VisualYConditioner` 和按键 / phase 状态。

| 项目 | 约定 |
| --- | --- |
| 按键 | `b` 接近 / Y；`m` 扫描；`t` 本轮结束；`q` terminate 后退出视觉 |
| phase | -1 未启用扫描；0 Scan、1 Trigger、2 Action |
| v1 意图 | session、递增 sequence、整数 `timestamp_unix_ms`、y（m）、rz（deg）、action、terminate |
| 时效 | 视觉心跳 200 ms，Robot 按本机接收时间执行 500 ms fresh 门 |
| Y 调理 | 死区 0.01 mm、EMA 当前帧权重 0.7、换向确认 1 帧 |

视觉端先发 idle 是生产者约定，Robot 当前没有独立的“新 session 必须先 idle”接收门。
发布成功只说明 Redis 收到消息，执行确认应看同 session 的 status `producer_sequence`。

## 6. 日志与修改验证

| 证据 | 先看什么 |
| --- | --- |
| summary | result、fault、completion、control 有效配置、ServoJ / Stop 结果、丢行数 |
| CSV | state、命令年龄 / 序号、请求增量、控制 Fz、卸载 / recovery、实际与模型误差、周期超期 |
| 腕部日志 | 相机 JSONL、tracking 图片、Robot wrist CSV；按同机单调时间关联 |

控制参数改对应配置结构体，公式 / 状态改控制模块，入口负责装配；跨模块接口决定记录到
[DECISIONS.md](../DECISIONS.md)。验证入口见 [AGENTS.md](../AGENTS.md#环境与验证入口)。
离线通过不能替代空间、力准确度、实体净空与停止验收；真机前按操作手册核对装配与现场条件。
运行期间不要清理正在写入的日志。
