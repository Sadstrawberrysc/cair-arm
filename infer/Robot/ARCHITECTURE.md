# RM75 控制服务架构

## 职责

该服务是 RM75 七轴生产控制入口，负责装配机器人状态、Haptron wrench、标定、接触点、
Redis 视觉意图、确定性控制律、七轴规划、安全监督和运行日志。唯一生产可执行文件为
`main_rm75`；维护工具不属于生产入口。
旧六轴源码仅作历史参考，CMake 不提供 legacy 构建开关或 target。

## 模块与依赖

| 模块 | 公共 API | 职责 | 直接依赖 |
| --- | --- | --- | --- |
| `realman_transport` | `RMConnectionConfig`、`RMCommand`、`RMStateReader`、`RMResult`、`BestEffortStopGuard` | RM75 JSON-over-TCP、状态快照、ServoJ mailbox、Stop 优先级与确认重试 | Threads、nlohmann/json |
| `robot_sensor` | `ForceSensorReader`、`ForceCalibration`、`ContactLocation` | Modbus/legacy 帧解析、SI 样本、重力与 tare 补偿、STL 接触点 | Threads、OpenSSL、Eigen |
| `rm75_motion` | `Rm75ControlLaw`、`Rm75ServoPlanner`、`RMKinematics` | 状态机、Tool 增量；`RMKinematics` 提供七轴 FK/Jacobian，planner 唯一负责 IK 与安全规划 | Eigen |
| `rm75_runtime_config` | `RobotRuntimeConfig`、profile 工厂、配置校验 | 入口参数与 Control/Planner/Safety 有效配置的单一装配路径 | `rm75_motion` |
| `rm75_frame_chain` | `CalibratedFrameChain`、`StopAndConfirmStationary()` | 不可变标定变换及依赖 Probe TCP 的物理静止确认 | transport、sensor、Eigen |
| `rm75_runtime_logging` | `AsyncRuntimeLogger`、`RuntimeSummaryData`、`BuildRuntimeSummary()` | 有界异步 CSV、summary v2 构造/落盘及终端完成报告 | transport、sensor、motion |
| `RedisBridge` | `LatestCommand`、`PublishSensor`、`PublishStatus` | Redis v1/legacy 边界、重连、回放保护和异步发布 | hiredis、nlohmann/json |
| `main_rm75` | 进程入口 | 配置校验、线程生命周期、10 ms 编排、安全门、周期/最终快照提交 | 上述全部 |
| `tests/offline` | CTest 可执行文件 | 有效配置、parser、freshness、控制/planner 拒绝边界和 schema characterization | 生产静态库，不访问设备 |

CMake 依赖方向为 `main_rm75 → rm75_runtime_config + rm75_frame_chain + rm75_runtime_logging + realman_transport + robot_sensor + rm75_motion`。
`redis_bridge.cpp` 直接编入入口，避免控制/传感器库反向依赖 Redis。

## 核心 API 契约

- `RMCommand` 的连接、读取、运动、ServoJ、Hold 和 Stop 操作只提供返回 `RMResult` 的 `Try*`
  API，调用者必须显式检查结果；生产 transport 不再提供旧六轴无返回值 wrapper。
- `RMConnectionConfig` 在构造 `RMCommand` 时单次注入控制器 IPv4 地址和端口，连接生命周期内
  不可改写；生产入口和 maintenance tools 不得再修改 transport 内部连接字段。
- `RMCommand` 的 socket、`RMJsonLineFramer`、mailbox 和错误缓存均为 transport 私有状态；
  JSON 请求直接从局部 payload 发送，接收字节只通过单一 byte-span 入口进入有界 framer，
  不保留无人消费的缓冲/丢弃计数 getter、旧 public 字符数组或重复 JSON 镜像。调用者只通过
  `Try*`、快照查询和 `SetQuiet()` 交互。
- `RMStateReader::Latest()` 返回完整 `RobotStateSnapshot`，单位为关节 rad、位置 m、姿态 rad。
- `ForceSensorReader::LatestSample()` 返回 `[Fx,Fy,Fz,Tx,Ty,Tz]` SI 快照；陈旧判定只通过
  `WrenchSample::IsStale()` 使用当前 `steady_clock`，不公开无人调用的显式时刻包装。
- `ForceSensorReader` 是生产和 maintenance 传感器采集的唯一串口 owner；旧同步
  `CLinuxSerial` 已删除。`ForceSensorFrameParser` 继续作为 legacy AA55 协议的离线测试 seam，
  但不提供另一条生产串口读取路径。reader 不再复制一份无人消费的聚合统计；两种 parser
  保留各自的离线统计接口，运行健康状态只由 `Start()` 结果、`WrenchSample` 和 `LastError()`
  表达，不公开无人调用的内部运行、端口、配置或缓冲区 getter。
- `ForceCalibration::Compensate()` 输出 sensor/tool 两种表达；调用前必须先过原始量程门。
- `ContactLocation::estimateContactPoint()` 不退出进程，使用 `ContactEstimate.valid/error` 表示结果。
- `ContactEstimate` 只公开控制、Redis 和日志实际消费的点、残差、等效点误差、有效位和错误；
  不公开无人读取的 STL 面片索引。退化面片在 `LoadSTL()` 边界直接拒绝，不进入周期错误集合。
- 接触估计不再提供把失败压缩为 `false + 零点` 的旧 `calContactPoint()` wrapper；调用者必须保留
  `ContactEstimate` 的无接触、模型、数值条件和残差错误语义。
- `ContactLocation` 的 STL 法向与三角面片只由模型对象内部持有；旧 `faceN/facePara` 重复状态
  已删除。法向在 STL 事务加载时校验并归一化一次，周期估计直接复用；平面参数仍从同一面片
  和法向计算，调用者不能改写半加载模型。
- `Rm75ControlLaw::Step(input, intent, motion_armed)` 只产生笛卡尔目标，不发送硬件命令。
- `Rm75ServoPlanner::Plan(...)` 只有返回 `valid=true` 才可提交 ServoJ。
- `RMKinematics` 只提供 RM75 七轴 FK 和 6×7 Jacobian；旧 `GetNextJoints()` 迭代接口已删除，
  IK、阻尼、限位和奇异拒绝不得绕过 `Rm75ServoPlanner`。非零 MDH 参数和关节限位由运动学
  对象私有持有；RM75 表中的恒零 a、关节 offset 和 tool offset 不保存为重复运行状态。
  planner、入口诊断和 maintenance tool 只能通过 `JointMinimums()`/`JointMaximums()` 读取限位。
- `RedisBridge::EvaluateCommandForControl()` 是 Redis 意图进入控制层的 fail-closed 门。
- `MakeImplicitRm75ProductionConfig()` 生成无参数生产 profile；启动校验、模块构造、终端输出和
  summary 必须读取其中同一组 `control/planner/safety` 配置。
- `CalibratedFrameChain` 从已校验标定单次构造，统一提供 Probe TCP Base 位置、Tool-Y Base
  方向、Base←Tool/Sensor 旋转和 Arm_Tip 姿态差；入口不得手工重组这些矩阵。
- `RequestConfirmedStop()` 保持 Stop mailbox 优先和有界确认重试；`StopAndConfirmStationary()`
  只有连续 5 帧同时满足关节、Probe TCP 和姿态静止门才返回成功。提前退出由
  `BestEffortStopGuard` 补发一次有界 Stop。
- `AsyncRuntimeLogger::PushAndMeasure()` 只向 8192 行有界队列提交周期快照并记录完成时间；后台
  writer 是唯一 CSV 文件 owner。`BuildRuntimeSummary()` 只处理最终快照，不参与控制循环。

## 线程与时序

```text
RMStateReader/I/O owner ─┐
ForceSensorReader ───────┼→ immutable/latest snapshots → 10 ms control loop
Redis subscriber ────────┘                                │
Redis publisher ← bounded/coalesced queue ────────────────┤
rm75_runtime_logging writer ← fixed cycle record ─────────┘
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
  `Rm75RuntimeSafetyConfig`；`RobotRuntimeConfig` 只统一装配这些配置及入口参数，入口随后只
  派生控制周期，并从 `CalibratedFrameChain` 注入控制律所需的不可变变换。控制和安全参数不再
  提供 CLI 覆盖。
- 生产使用七轴反馈和 6×7 Jacobian；不得接入 `tests/legacy/six_axis`。
- `tests/legacy/six_axis` 仅保存历史源码，不保证继续针对当前生产头文件编译；maintenance tools
  必须使用 `RMConnectionConfig`、`RMResult` 和 `Try*` API。
- CSV 列顺序和单位由 `runtime_schema.hpp` 固化，summary 由 `RuntimeSummaryData` 单次构造；入口
  不得自行追加平行的日志序列化路径。
- `build/` 中标定、日志和二进制属于运行产物，不是模块 API。

## 构建与验证

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release
cmake --build infer/Robot/build --target main_rm75 robot_offline_tests
ctest --test-dir infer/Robot/build --output-on-failure
```

当前注册 6 个纯离线 CTest，覆盖有效配置唯一来源、标定坐标链等价性、Redis
parser/freshness 与输出 schema、
AA55/Haptron frame parser、基础控制状态、planner 非法输入/限位拒绝，以及 CSV/summary schema
契约。测试不访问 Redis 服务、串口或机器人；通过仍不代表标定或真机运动验收完成。任何运行
命令必须遵守根目录 `AGENTS.md` 的真机硬约束。
