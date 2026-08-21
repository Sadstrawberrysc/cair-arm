# 决策日志

本文件只记录影响多个模块、公共接口、运行安全或长期维护方式的关键决策。详细设计放在
`ARCHITECTURE.md` 和各模块文档中。

后续记录规则：

- 新决策按时间倒序追加，格式保持为“日期 / 决策 / 原因”。
- 已落地决策发生变化时新增一条“取代某决策”的记录，不改写历史。
- 参数微调、普通修复、一次性实验和实现细节不记入本文件。
- 尚未确定的方案写入 `PROGRESS.md` 待办，达成结论后再转为决策。

## 2026-08-20：runtime tare 使用显式配置与只读停止依赖

**决策：** `CollectRuntimeTare()`、`RuntimeTareResult` 和 `ApplyRuntimeTare()` 与依赖 Probe TCP
的静止逻辑共同归入现有 frame-chain 模块。采样率、关节/TCP/姿态跨度及 wrench 稳定门由
`RuntimeTareConfig` 唯一持有；进程停止标志通过 `const std::atomic<bool>&` 单次注入。

**原因：** tare 同时依赖标定坐标、机器人反馈和传感器快照，留在入口会重复坐标和阈值逻辑，
并隐式读取进程全局状态。显式依赖使入口只编排采集结果，同时保持采集顺序、SI 单位和原安全门
不变。

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
