# Sensor Monitor 服务架构

## 职责

该服务是六维力传感器与 RM75 结构化遥测的只读监视器，不生成控制命令。启动时自动选择
数据源：`main_rm75` 已发布遥测或已占用串口时，读取 Redis 并同图显示 Sensor-frame
原始 wrench（实线）与 Tool-frame 补偿 wrench（虚线）；否则独占直连 Haptron，仅显示原始值。

## 接口

- 自动选择：先探测 `127.0.0.1:7777` channel `robot:sensor:v1`，再检查串口独占状态。
- 直连输入：Haptron Modbus RTU `/dev/ttyUSB0`、115200 baud、slave 1。
- CLI：直连设备/Modbus、Redis、时间窗口和刷新率参数；不对外暴露数据源手动切换。
- `parse_sensor_payload()` 是纯解析边界，仅接受 `version=1`，校验结构、数值有限性和 sequence。
- `SensorSample` 是 GUI 使用的不可变内部 DTO；无网络输出。

传感器 `valid`、checksum、stale 与 io_error 联合决定 wrench 是否可显示；无效值用 NaN
形成图线断点，不能用零伪装成有效测量。接触点从 m 转为 mm 仅用于 UI。

## 依赖与线程

依赖 Python、redis-py、NumPy、PyQt5 和 pyqtgraph，不额外依赖 pyserial。`DirectSensorReader`
使用 Python 标准库在 QThread 中发送只读 Modbus function-04 查询；`TIOCEXCL` 防止与
`main_rm75` 同时占用串口。`RedisSubscriber` 保留原结构化遥测路径。二者均通过 Qt signal
向 GUI 线程传递完整 `SensorSample`；`SampleBuffer` 只由 GUI 线程拥有。

## 架构决策

- 严格只读，不得加入控制发布功能。
- 直连模式只显示原始 Sensor-frame wrench；没有机器人姿态时不得伪造重力补偿或接触点。
- 直连模式与 `main_rm75` 互斥；要启动主程序时仍需先退出已在直连的监视器，
  然后启动 `main_rm75` 并重新打开监视器，使其自动选择 Redis。
- 协议变更通过新增 version 和解析分支完成，不静默接受含义不同的字段。
- GUI 卡顿不能反压机器人服务；Pub/Sub 允许监视器丢失中间样本并显示最新状态。
