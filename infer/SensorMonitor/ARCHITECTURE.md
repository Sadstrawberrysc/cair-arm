# Sensor Monitor 服务架构

## 职责

该服务是 RM75 结构化遥测的只读监视器，显示补偿力/力矩曲线、接触点、I/O 状态、控制状态
和 fault，用于现场诊断与日志对照，不生成控制命令。

## 接口

- 默认输入：Redis `127.0.0.1:7777` channel `robot:sensor:v1`。
- CLI：`--host`、`--port`、`--channel`、时间窗口和刷新率参数。
- `parse_sensor_payload()` 是纯解析边界，仅接受 `version=1`，校验结构、数值有限性和 sequence。
- `SensorSample` 是 GUI 使用的不可变内部 DTO；无网络输出。

传感器 `valid`、checksum、stale 与 io_error 联合决定 wrench 是否可显示；无效值用 NaN
形成图线断点，不能用零伪装成有效测量。接触点从 m 转为 mm 仅用于 UI。

## 依赖与线程

依赖 Python、redis-py、NumPy、PyQt5 和 pyqtgraph。`RedisSubscriber` 在 QThread 中负责
连接、重连和解析，通过 Qt signal 向 GUI 线程传递完整 `SensorSample`；`SampleBuffer` 只由
GUI 线程拥有。sequence 或时间倒退会清空窗口，表示生产者重启或时间线改变。

## 架构决策

- 严格只读，不得加入控制发布功能。
- 协议变更通过新增 version 和解析分支完成，不静默接受含义不同的字段。
- GUI 卡顿不能反压机器人服务；Pub/Sub 允许监视器丢失中间样本并显示最新状态。
