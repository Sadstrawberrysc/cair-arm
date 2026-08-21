# 决策日志

本文件只记录影响多个模块、公共接口、运行安全或长期维护方式的关键决策。详细设计放在
`ARCHITECTURE.md` 和各模块文档中。

后续记录规则：

- 新决策按时间倒序追加，格式保持为“日期 / 决策 / 原因”。
- 已落地决策发生变化时新增一条“取代某决策”的记录，不改写历史。
- 参数微调、普通修复、一次性实验和实现细节不记入本文件。
- 尚未确定的方案写入 `PROGRESS.md` 待办，达成结论后再转为决策。

## 2026-08-21：WrenchSample 只公开生产使用的 stale 判定

**决策：** `WrenchSample` 只保留基于当前 `steady_clock` 的 `IsStale(maximum_age)`；删除无人
调用的显式时刻转发方法。有效位、I/O 错误、streaming 状态、时间戳缺失/倒退、负最大年龄和
超龄条件保持原顺序与语义。

**原因：** 全仓库只有当前时刻入口调用转发方法，额外公开另一套入口扩大 API，但没有测试或
运行调用者，也不改变 fail-closed stale 判定所需的信息。

## 2026-08-21：传感器运行健康只通过结果和快照公开

**决策：** 删除 force sensor reader 无调用的内部运行、端口和配置 getter，以及两种 parser
无调用的缓冲区大小 getter。reader 的启动结果、最新 `WrenchSample`、`LastError()` 与 parser
离线统计接口继续保留；内部原子状态、配置和有界缓冲区不变。

**原因：** 全仓库没有这些 getter 的调用者；并行暴露内部布尔状态会与携带 stale 和 I/O
错误的快照形成第二套健康判断来源，缓冲区大小也已包含在 parser 统计快照中。

## 2026-08-21：传感器 reader 不再维护无人消费的统计镜像

**决策：** 删除 `ForceSensorReader` 的聚合统计查询、镜像状态和 parser-to-reader 复制路径；
legacy 与 Haptron parser 继续维护各自的离线统计接口。生产健康状态仍通过最新
`WrenchSample` 的 sequence、checksum、stale、I/O 状态/错误以及 `LastError()` 表达。

**原因：** 全仓库没有 reader 聚合统计调用者，镜像只在串口热路径增加复制、互斥锁和公共
API 面积，却不参与任何控制、安全、日志或故障判断。保留 parser 统计即可维持协议测试 seam。

## 2026-08-21：ContactEstimate 只保留被运行链消费的结果

**决策：** `ContactEstimate` 不再公开无人读取的 STL 面片索引；退化面片只在事务式
`LoadSTL()` 边界拒绝，不再保留无法从周期估计产生的退化模型错误项。其后错误码显式保持
原数值，点、残差、等效点误差、有效位以及 Redis/CSV 错误字符串契约不变。

**原因：** 全仓库审计确认面片索引只有写入没有读取，成功加载的私有模型也不存在退化面片，
保留这两项只会扩大公共 API 并暗示不存在的运行分支。

## 2026-08-21：生产 transport 不再兼容旧六轴无返回值 API

**决策：** 删除 `RMCommand` 的 `ConnectTCPSocket()`、`Read*()`、`Move*()`、`ServoJ()`、
`HoldMotion()` 和 `StopMotion()` 无返回值 wrapper；生产与 maintenance 调用统一使用返回
`RMResult` 的 `Try*` API。`tests/legacy/six_axis` 仅保留历史源码，不要求针对生产头文件编译。

**原因：** 全仓库审计确认生产入口和 maintenance tools 均已迁移，wrapper 只会吞掉结构化
错误并保留旧六轴源码兼容负担。统一显式结果传播可以避免运动或 Stop 失败被调用者忽略。

## 2026-08-21：RMCommand 不再公开 transport 实现状态

**决策：** `RMCommand` 的 socket、收发缓冲、JSON/命令缓存及内部错误缓存全部私有化；生产
入口和 maintenance tools 只使用 `Try*`、状态快照、连接配置及 `SetQuiet()` 公共接口。

**原因：** 这些字段没有非 legacy 外部读取者，公开后却允许调用者绕过 mailbox、锁和结果传播
直接改变通信状态。收窄接口可以固定单一 I/O owner 边界，同时不改变报文、时序或运动语义。

## 2026-08-21：RM75 连接端点改为构造时单次注入

**决策：** `RMConnectionConfig` 唯一表达 RM75 控制器 IPv4 地址和端口，并在构造
`RMCommand` 时单次注入；生产入口和 maintenance tools 不再直接改写 transport 的公开字符
缓冲区或端口字段。默认端点仍为 `192.168.50.254:8080`。

**原因：** 构造后逐字段复制导致生产和维护入口重复维护缓冲区长度与终止符，并允许连接生命
周期内意外改变目标控制器。不可变配置收缩了公共 API，同时不改变连接时序、socket 协议或运动
命令路径。

## 2026-08-20：Stop 命令确认与物理静止确认分层

**决策：** `realman_transport` 唯一提供 Stop mailbox 优先级、`RequestConfirmedStop()` 有界确认
重试和 `BestEffortStopGuard` 提前退出兜底；依赖坐标的 `StopAndConfirmStationary()` 使用
`CalibratedFrameChain` 检查连续 5 帧关节、Probe TCP 和姿态静止。入口只处理返回的 `RMResult`。

**原因：** `arm_stop=true` 只证明控制器确认命令，不能证明机械臂物理静止。分层后所有启动、
故障、deadline 和正常退出路径复用同一门，同时保持原超时、重试、阈值和 Stop 高于 ServoJ 的
优先级不变。

## 2026-08-20：标定坐标链统一为不可变对象

**决策：** `CalibratedFrameChain` 从单份已校验 `ForceCalibration` 构造，唯一提供
Base→Arm_Tip→Tool/Sensor→Probe TCP 的运行时变换，包括 Probe TCP Base 位置、Tool-Y Base
方向、Base←Tool/Sensor 旋转和 Arm_Tip 姿态差。入口与安全检查不再手工拼接这些矩阵。

**原因：** 同一旋转和 TCP 偏移此前分散用于 tare、wrench 补偿、tracking、行程门和日志，重复
表达容易产生乘法顺序或坐标语义分叉。不可变对象确保各消费者使用同一标定快照，同时保持原有
Euler 约定、乘法顺序和 SI 单位。

## 2026-08-20：运行日志由独立模块单线程持有文件

**决策：** `rm75_runtime_logging` 唯一拥有异步 CSV writer、固定周期行序列化和 summary v2
构造/落盘。控制循环只提交有界周期快照，入口在停止所有 I/O owner 后只提交一次最终运行结果；
CSV header、字段顺序、单位及 summary schema v2 继续作为兼容契约。

**原因：** 日志格式化、JSON 构造和磁盘 I/O 不应进入 10 ms 控制编排，也不应长期占据生产
入口。集中所有日志序列化可使 schema characterization 测试直接覆盖真实 builder，同时保持
8192 行有界队列、丢行计数和后台单 writer 行为不变。

## 2026-08-20：运行配置统一为强类型单一来源

**决策：** `RobotRuntimeConfig` 统一装配入口参数及 `Rm75ControlConfig`、
`Rm75ServoPlannerConfig`、`Rm75RuntimeSafetyConfig`。删除控制/安全 CLI 覆盖，无参数 profile
只设置设备、生命周期和生产接近包络；启动校验、控制对象、终端输出和 summary 使用同一有效
配置。Redis `desired_force_n` 继续作为协议明确提供的单次命令覆盖。profile 名称不再编码目标力。

**原因：** 原 profile 用 `-3 N / 1 cm/s` 通过启动门，但实际控制对象长期执行
`-2 N / 2 cm/s`，导致校验、命名与真机行为分叉。用户明确选择保持当前实际行为，并授权将
provisional 准入校验对齐到 `-2 N / 2 cm/s`；raw wrench、tracking、IK、tare 和 deadline
等其他安全门保持不变。

## 2026-08-20：标定文件收敛为无版本唯一名称

**决策：** 当前v10来源的标定结果作为唯一部署文件，统一命名为
`rm75_force_calibration.json`，其采集溯源文件命名为 `calibration_samples.csv`。删除工作区内
旧v6/v9标定结果和v8/v9采集文件，无参数 `main_rm75` 只引用该无版本文件。

**原因：** 用户决定不再保留多个工程标定版本，避免默认路径与现场所称“当前标定”
不一致。该命名决策不改写质量事实：文件仍为 `residuals_verified=false`、
`tool_chain_verified=false`，Force max `1.2488 N` 仍未通过 `0.6 N` 门。

## 2026-08-20：默认临时力标定切换为v10

**决策：** 无参数 `main_rm75` 默认加载
`rm75_force_calibration_v10_provisional.json`，取代v9作为当前工程调试标定。v10仍保持
`residuals_verified=false` 和 `tool_chain_verified=false`，并继续受现有provisional启动tare、
wrench硬门和其他真机安全门约束。

**原因：** v10的10姿态采集通过多样性预检，Force max从v9的 `1.7448 N`改善为
`1.2488 N`，但仍高于正式 `0.6 N` 接受门；因此只能按用户要求作为临时调试标定，
不得表述为已验收标定。

## 2026-08-20：SensorMonitor 自动选择六维力数据源

**决策：** SensorMonitor 自动选择数据源：`main_rm75` 已发布遥测或已独占串口时，
只读订阅 Redis，以实线显示 Sensor-frame 原始 wrench、以虚线显示 Tool-frame 补偿 wrench；
否则通过独占、只读的 Haptron Modbus RTU 查询只显示原始 wrench。不提供数据源手动切换参数。

**原因：** 启动 tare 失败时需要在控制进程未运行的情况下独立观察传感器数据；明确区分原始
直连数据和生产控制器补偿数据，可以避免为诊断而启动机器人控制，也避免伪造补偿语义。

## 2026-08-20：建立仓库级协作、架构、进度和决策文档

**决策：** 根目录使用 `AGENTS.md` 约束协作和真机安全，使用 `ARCHITECTURE.md` 描述当前
服务/API 边界，使用 `PROGRESS.md` 记录实时状态和待验收项，使用本文件保存长期关键决策。

**原因：** 运行说明、架构现状、项目进度和历史决策具有不同更新节奏；分开维护可以避免
临时参数、设计原因和操作规则互相覆盖，也便于新成员找到当前事实来源。

## 2026-08-05：视觉闭环沿用三阶段协议并在视觉端映射五分类模型

**决策：** ConvNeXt 五分类 `pre/in/after/brench/rota` 在视觉服务边界映射为 RM75 的
`Scan/Trigger/Action` 三阶段；机器人控制状态机不直接依赖模型的五个训练标签。

**原因：** 保持已部署 Redis/控制状态机协议稳定，同时允许视觉模型独立迭代；模型标签变化
不会直接扩大机器人端状态空间。

## 2026-07-31：生产入口唯一化，legacy 与维护工具隔离

**决策：** RM75 生产构建只保留 `main_rm75` 入口；旧六轴源码移动到
`infer/Robot/tests/legacy/six_axis/`，诊断和标定程序移动到 `tests/tools/`，并通过默认关闭的
`BUILD_MAINTENANCE_TOOLS` 单独构建。

**原因：** 避免旧六轴入口或会运动的诊断程序被误当作七轴生产控制器，同时保留历史参考和
现场维护能力。

## 2026-07-24：Redis 视觉命令采用会话、递增序号和失效关闭

**决策：** 生产命令使用 Redis v1 payload；每个视觉进程创建新的 `session_id`，`sequence`
在会话内严格递增并以心跳刷新。断线、过期、重放、无效 JSON 或反馈恢复后的旧命令均进入
Hold，只有更新命令才能重新释放运动。

**原因：** Pub/Sub 不提供持久执行确认，不能让重连前命令、重复角度增量或陈旧视觉结果继续
驱动机械臂；失效关闭比静默沿用上一条运动命令更安全。

## 2026-07-21：控制增量统一在 Tool/Sensor 坐标表达

**决策：** 力、接触点和 Tool-X/Y/Z/RZ 控制请求在 Tool/Sensor 坐标系表达；通过标定固定
变换映射到 Arm_Tip，再由七轴 IK 生成关节目标。公共数值使用 SI 单位，视觉协议中的 RZ 度数
只在接口边界转换。

**原因：** 扫描方向和力反馈必须跟随探头物理轴，而不是 Base 轴；明确坐标链和单位可避免
安装旋转、TCP 偏移和隐式单位换算造成方向错误。

## 2026-07-21：确定性控制、关节规划和硬件 I/O 分层

**决策：** `Rm75ControlLaw` 只根据快照产生状态与笛卡尔目标，`Rm75ServoPlanner` 负责
6×7 IK、限位、速度/加速度和奇异性检查，`RMCommand`/`RMStateReader` 才拥有硬件通信。
10 ms 控制循环只消费内存快照，Redis、串口、socket 和日志写盘放在独立 I/O 线程。

**原因：** 将控制计算与非确定时延隔离，防止阻塞 I/O 破坏控制周期；同时为离线回放、单元
测试和安全规划提供清晰边界。

## 2026-07-06：机械臂主链路迁移到 RM75 七轴模型

**决策：** 生产运动学、状态和命令接口以 RM75 七关节、6×7 Jacobian 和七关节 ServoJ 为准，
不在生产链中兼容旧六轴数组或运动学假设。

**原因：** RM75 的冗余自由度、关节限位和奇异性行为与旧六轴系统不同；在生产边界保留六轴
假设会产生错误索引、错误 IK 和不可验证的运动目标。
