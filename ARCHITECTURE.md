# USPilot Control 系统架构

超声扫描机器人的视觉—力觉闭环系统。同机进程通过 Redis、设备 TCP 和串口通信；训练实验独立于在线控制。

## 在线链路

| 链路 | 入口与数据流 |
| --- | --- |
| 串联运行 | `scripts/run_scan_workflow.py` → 全局采样/MoveJ_P → 腕部跟随 → 超声扫描；保留窗口按键，以相机退出和 Robot 正常停止衔接阶段，操作见 [系统运行](.runme/run/system-operations.md) |
| 超声闭环 | `intergrate_infer/main_redis_seg_newphase_recovery_mode.py` → Redis → `infer/Robot/src/main_rm75.cpp` → RM75 |
| 全局初始定位 | `Camera_RT/cliff_demo.py` 保存相机快照 → `hand_eye_calibration-main/run_probe_target.py` 转换顶部首点并构造短轴姿态 → `arm_probe_pose` 执行单次 MoveJ_P |
| 腕部跟随 | `Camera_wrist/click_follow.py` 发布 RGB-D 观测与请求 → Robot 按采集时刻变换坐标 → `WristFollowController` → 七轴 planner / ServoJ |
| 超声设备与 UI | `py-xiaokai` → Redis → `SonoScape_api/redis_service.py` → 超声主机 |

Robot 在 10 ms 周期内融合机械臂状态和最新 SI 力快照，执行补偿、接触估计、控制与规划；
设备 I/O、Redis 发布和日志由后台处理。默认力传感器为 KWR57 RS422，保留 Haptron / AA55 协议。

## 项目通信与独立监控

| 范围 | 数据流与职责 |
| --- | --- |
| 项目力觉与遥测 | KWR57 → Robot `ForceSensorReader` → 补偿 / 控制；Robot 通过 Redis `robot:sensor:v1` 发布遥测 |
| SensorMonitor 独立检查 | 未检测到项目遥测时，KWR57 → SensorMonitor → 本地曲线 / CSV / 图片 |
| 可选遥测查看 | 检测到项目遥测时，Robot → Redis → SensorMonitor，仅作只读显示 / 本地记录，无需指定参数 |

SensorMonitor 默认自动识别输入来源；项目不依赖它启动或转发数据。监控窗口的清零、记录与退出只影响监控程序；
项目的 tare、补偿、控制和日志由 Robot 管理。串口直连监控与 Robot 使用同一设备时互斥。

## 全局相机碰撞检查

[Collision 架构](infer/Collision/ARCHITECTURE.md)描述独立的 C++17 检查链路：
展开后的 URDF／STL → 逐连杆 FK 与 FCL 实体几何；D455 点云与独立 RM75 七轴反馈 →
Color→Base 变换、自身及半径邻域稀疏点过滤 → 单帧 OctoMap → 当前状态的环境／自碰撞、距离及未知空间检查 → JSON／3D 显示。
人工静态场景已支持 OMPL RRTConnect 关节路径搜索、FCL 路径段约束、全路径复查、保存／校验重放和动画预览。
监测窗口通过 `--goal` 按 P 或 `--plan-once` 已将同帧关节反馈和 OctoMap 快照接入该规划器，结果只读。
`start_monitor.sh --coarse` 从新保存的颈动脉点文件生成后退 50 mm 的探头 TCP 目标，读取 Robot 工具标定，
用同一 URDF 求多候选 IK，再经当前场景碰撞检查与规划；不调用上表既有 MoveJ_P 运动入口。
旧 TXT 仅有保存时间而无曝光／受试者身份，当前以文件时效、相机标定身份和当前点云支持约束目标，仍需新鲜采集和实体核对。

该链路当前不消费 Robot 生产状态缓存或 Redis 七轴反馈，也不向 Robot 发送运动或停止指令。
它把所有非机器人观测作为障碍物，没有人体分类；遮挡、无效深度和 ROI 外保持未知，不能按“未见障碍”视为自由。
相机和机械臂为独立诊断接入，与其他采集／控制进程的并行资源使用尚需验证。
后续规划在控制周期外运行，Robot 保持唯一运动发送者；轨迹执行、停止确认及重规划尚未实现。
当前编译与离线验证不代表真实设备联机、空间精度或运动保护已验收。

## 公共接口与坐标

| 接口 | 用途 |
| --- | --- |
| Redis `127.0.0.1:7777`，`robot:command:channel` | 超声意图 JSON v1；session、递增 sequence 和接收时效校验；y 为 m，rz 为度 |
| 同实例，`robot:status:channel` / `robot:sensor:v1` | 状态确认、结构化力与接触遥测；`sensor_data` 兼容旧显示器 |
| 同实例，`robot:wrist:*:v1` | 腕部 seed、state、observation、command；详见腕部协议 |
| Redis `127.0.0.1:6379` | SonoScape 命令 List 与结果键；`device:arm:*` 无当前 RM75 消费者 |

内部单位为 m、rad、N、N·m；`T_A_B` 将 B 坐标映射到 A。
`RobotRuntimeConfig` 装配有效配置，`CalibratedFrameChain` 统一坐标链。
全局外参为 Camera→Base，腕部外参为 Camera→ArmTip；Tool 工作轴与 Sensor 安装旋转分别保存。
相机标定统一从 `calibrate_realsense.py` / `calibrate_gemini.py` 进入，默认采集 50 组后求解，不自动部署。

## 运行边界

- 无参数 `main_rm75` 是真实 Execute 入口；默认读取可执行文件旁 `rm75_force_calibration.json`，身份要求 `KWR57`。
- 全局定位使用本轮新快照的画面顶部首点，Tool +X 沿血管切向、+Z 朝内，沿 Tool -Z 后退 50 mm；默认速度 3。
- 腕部相机默认模型模式：新鲜 Robot Hold 和完整 RGB-D → 自动选点 → 有效观测及目标回传 → 请求跟随；`--manual` 使用人工选点与开始。
- 腕部 `start_robot.sh` 是持续运行的真实无力候选试验入口，关闭累计位移 / 转角及 TCP 位置跟踪误差门；保留关节、规划、通信与停止保护。
- 相机 `q` 退出本轮，Robot 保持 Hold；Robot `Ctrl+C` 进入停止与静止确认。候选试验、用户确认或离线通过均不代表独立空间验收。

## 文档入口

| 内容 | 文档 |
| --- | --- |
| 本机操作与进展 | [Runme 索引](.runme/README.md)、[全系统入口](.runme/run/system-operations.md)、[项目进展](.runme/Progress/project-progress.md) |
| Robot 控制、协议与构建 | [Robot 架构](infer/Robot/ARCHITECTURE.md)、[操作](.runme/run/robot-operations.md)、[学习路径](.runme/rm75_learning_path.md) |
| 超声推理 | [推理架构](intergrate_infer/ARCHITECTURE.md) |
| 全局相机与初始定位 | [Camera_RT 架构](infer/Camera_RT/ARCHITECTURE.md)、[操作](.runme/run/camera-rt-operations.md) |
| 腕部跟随与模型 | [Camera_wrist 架构](infer/Camera_wrist/ARCHITECTURE.md)、[detector 架构](infer/Camera_wrist/detector/ARCHITECTURE.md)、[操作](.runme/run/camera-wrist-operations.md) |
| 相机标定 | [标定说明](infer/hand_eye_calibration-main/README.md)、[操作](.runme/run/calibration_camera.md) |
| 力传感器与末端替换 | [KWR57 接入](docs/kwr57_sensor_integration.md)、[采集分析](docs/sensor_drift_recording.md)、[SensorMonitor 架构](infer/SensorMonitor/ARCHITECTURE.md)、[替换计划](.runme/Progress/project-progress.md#末端替换计划与验收记录历史) |
| 长期决策 | [DECISIONS.md](DECISIONS.md) |
| 碰撞检测与避障规划 | [Collision 架构](infer/Collision/ARCHITECTURE.md)、[进展](infer/Collision/PROGRESS.md)、[监测操作](infer/Collision/MONITOR.md)、[规划操作](infer/Collision/PLANNING.md)；当前支持人工／相机场景 OMPL 规划与颈动脉目标 IK，生产反馈桥接和执行待完成 |

`.runme/` Markdown 随仓库维护；路径和设备配置按本机调整。设备、常驻进程及运动单元逐个执行，禁止 `runme run --all`。
