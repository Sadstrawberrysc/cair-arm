# 系统架构

同机多进程，通过 Redis、设备 TCP 和串口通信；训练与实验目录独立于在线链路。

## 主要链路

- **超声闭环**：`intergrate_infer/main_redis_seg_newphase_recovery_mode.py` → Redis →
  `infer/Robot/src/main_rm75.cpp` → RM75。Robot 融合机械臂状态与 Haptron 力数据，
  在 10 ms 周期内执行接触估计、控制计算、七轴 IK 与 ServoJ。
- **全局初始定位**：`Camera_RT/cliff_demo.py` 保存左颈动脉相机坐标快照 →
  `hand_eye_calibration-main/run_probe_target.py` 用 D455 外参转换首点、构造短轴姿态，
  沿目标 Tool -Z 后退 50 mm → `arm_probe_pose` 换算 Probe TCP 并调用控制器 MoveJ_P。
- **腕部跟随**：`Camera_wrist/click_follow.py` 发布 RGB-D 追踪观测和请求 → Redis →
  Robot 按采集时刻转换坐标 → `WristFollowController` → 原 planner 与运动发送链。
  Robot 回传相机位姿和请求确认；`projection_preview.py` 另支持全局种子投影与追踪预览。
- **超声设备/UI**：`py-xiaokai` → Redis → `SonoScape_api/redis_service.py` → 超声主机。

## 接口与计算

| 接口 | 用途 |
| --- | --- |
| Redis `127.0.0.1:7777`，`robot:command:channel` | 超声命令 JSON v1；会话、递增序号与时效校验；y 为 m，rz 为度 |
| 同实例，`robot:status:channel`、`robot:sensor:v1` | 状态和结构化力/接触遥测；`sensor_data` 兼容旧显示器 |
| 同实例，`robot:wrist:*:v1` | seed、state、observation、command；详见腕部协议 |
| Redis `127.0.0.1:6379` | SonoScape 命令 List 与带 TTL 的结果键；`device:arm:*` 无 RM75 消费者 |

Robot 的控制律和 planner 消费内存快照，设备通信、Redis 与日志由后台 I/O 处理。
`RobotRuntimeConfig` 装配有效配置，`CalibratedFrameChain` 提供标定坐标变换。
内部使用 m、rad、N、N·m；全局外参为 Camera→Base，腕部外参为 Camera→ArmTip。
`--wrist-no-force` 在腕部模式禁用力采集及力控，遥测明确报告力数据无效。

## 详细文档

- [Robot](infer/Robot/ARCHITECTURE.md)、[超声推理](intergrate_infer/ARCHITECTURE.md)、
  [全局相机](infer/Camera_RT/ARCHITECTURE.md)、[传感器监视](infer/SensorMonitor/ARCHITECTURE.md)
- [腕部架构与协议](infer/Camera_wrist/ARCHITECTURE.md)、[腕部使用说明](infer/Camera_wrist/USAGE.md)
- [当前进度](PROGRESS.md)、[启动说明](docs/rm75_build_and_start_commands.md)、[决策摘要](DECISIONS.md)

2026-09-17 用户授权候选外参执行：--wrist-candidate-trial允许无力、限时30秒以内的候选实验；
额外--wrist-unlimited-excursion取消实测/规划相对起点的累计位移和累计转角门，仅此模式可用。
速度、关节/IK、通信、周期、Hold和急停不变，外参文件不改验证状态。summary control记录两个开关。

2026-09-17：Camera_wrist/start_robot.sh为已授权腕部实验的专用无参数常驻入口。
候选执行可设duration=0，由Ctrl+C/故障结束；相机q只结束本轮并保持Hold。普通main_rm75默认模式不变。
