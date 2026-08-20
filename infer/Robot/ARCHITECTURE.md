# RM75 控制服务架构

## 职责

该服务是 RM75 七轴生产控制入口，负责装配机器人状态、Haptron wrench、标定、接触点、
Redis 视觉意图、确定性控制律、七轴规划、安全监督和运行日志。唯一生产可执行文件为
`main_rm75`；维护工具不属于生产入口。

## 模块与依赖

| 模块 | 公共 API | 职责 | 直接依赖 |
| --- | --- | --- | --- |
| `realman_transport` | `RMCommand`、`RMStateReader`、`RMResult` | RM75 JSON-over-TCP、状态快照、ServoJ mailbox、Stop 优先级 | Threads、nlohmann/json |
| `robot_sensor` | `ForceSensorReader`、`ForceCalibration`、`ContactLocation` | Modbus/legacy 帧解析、SI 样本、重力与 tare 补偿、STL 接触点 | Threads、OpenSSL、Eigen |
| `rm75_motion` | `Rm75ControlLaw`、`Rm75ServoPlanner`、`RMKinematics` | 状态机、Tool 增量、七轴 FK/Jacobian/IK、安全规划 | Eigen |
| `RedisBridge` | `LatestCommand`、`PublishSensor`、`PublishStatus` | Redis v1/legacy 边界、重连、回放保护和异步发布 | hiredis、nlohmann/json |
| `main_rm75` | 进程入口 | 配置校验、线程生命周期、10 ms 编排、安全门、CSV/summary | 上述全部 |

CMake 依赖方向为 `main_rm75 → realman_transport + robot_sensor + rm75_motion`。`redis_bridge.cpp`
直接编入入口，避免控制/传感器库反向依赖 Redis。

## 核心 API 契约

- `RMCommand::Try*` 返回 `RMResult`，新代码必须检查结果；无返回值方法仅作 legacy 兼容。
- `RMStateReader::Latest()` 返回完整 `RobotStateSnapshot`，单位为关节 rad、位置 m、姿态 rad。
- `ForceSensorReader::LatestSample()` 返回 `[Fx,Fy,Fz,Tx,Ty,Tz]` SI 快照，陈旧判定使用
  `steady_clock`。
- `ForceCalibration::Compensate()` 输出 sensor/tool 两种表达；调用前必须先过原始量程门。
- `ContactLocation::estimateContactPoint()` 不退出进程，使用 `ContactEstimate.valid/error` 表示结果。
- `Rm75ControlLaw::Step(input, intent, motion_armed)` 只产生笛卡尔目标，不发送硬件命令。
- `Rm75ServoPlanner::Plan(...)` 只有返回 `valid=true` 才可提交 ServoJ。
- `RedisBridge::EvaluateCommandForControl()` 是 Redis 意图进入控制层的 fail-closed 门。

## 线程与时序

```text
RMStateReader/I/O owner ─┐
ForceSensorReader ───────┼→ immutable/latest snapshots → 10 ms control loop
Redis subscriber ────────┘                                │
Redis publisher ← bounded/coalesced queue ────────────────┤
AsyncRuntimeLogger ← fixed cycle record ──────────────────┘
```

控制周期固定顺序为：读取快照 → 校验安全门 → wrench 补偿 → 接触估计 → Redis 意图判定 →
控制律 → IK/ServoJ 规划 → 提交/Hold/Stop → 发布与日志。控制线程不得直接做阻塞 I/O。

ServoJ 使用单个待处理 mailbox，防止 10 ms 目标在 socket 队列中无限堆积；Stop 拥有更高
优先级。反馈恢复后必须按实际状态重建参考，并等待更新的生产者命令，不能继续旧命令。

## Redis 接口

- 输入：`robot:command:channel`，JSON v1，详见根级 [ARCHITECTURE.md](../../ARCHITECTURE.md)。
- 输出：`robot:status:channel`，包含 state、status、error code、session 与确认序号。
- 输出：`robot:sensor:v1`，包含原始/补偿 wrench、传感器状态、Probe-TCP 接触点和 fault。
- 兼容输出：`sensor_data`，四元素数组，仅供旧接触点显示器使用。

## 架构决策与边界

- 控制律与规划器保持无硬件 I/O，便于未来离线回放和确定性测试。
- 原始量程、补偿量程、数据陈旧、关节限位、奇异、跟踪误差和调度异常分层检查。
- 参数唯一来源是 `Rm75ControlConfig`、`Rm75ServoPlannerConfig`、
  `Rm75RuntimeSafetyConfig`；入口只注入标定坐标链。
- 生产使用七轴反馈和 6×7 Jacobian；不得接入 `tests/legacy/six_axis`。
- `build/` 中标定、日志和二进制属于运行产物，不是模块 API。

## 构建与验证

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release
cmake --build infer/Robot/build --target main_rm75
```

当前没有注册 CTest。编译只验证接口和链接，不代表状态机、标定或真机运动验收通过。
任何运行命令必须遵守根目录 `AGENTS.md` 的真机硬约束。
