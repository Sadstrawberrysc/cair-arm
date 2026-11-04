# SensorMonitor 架构

独立六轴力监控工具，负责曲线、本地记录、软件清零和出图。
项目由 Robot 采集传感器、执行控制并发布 Redis 遥测，不依赖 SensorMonitor。
监控清零仅影响本地显示与 CSV，不修改 Robot tare、标定或控制输入。

## 默认入口与来源

在仓库根目录运行 `python infer/SensorMonitor/main.py`，无需传入 `--source redis`。

| 输入 | 启动行为 | 设备边界 |
| --- | --- | --- |
| 默认自动识别 | 检查 `127.0.0.1:7777/robot:sensor:v1`，1 秒内收到消息则订阅 Redis；无服务或无消息则直连串口 | 识别一次，选定后不自动切换 |
| Redis | 使用 Robot 已发布的 Sensor-frame SI 六轴原始值 | 不打开串口，不发布控制命令 |
| KWR57 串口 | 默认 `/dev/ttyUSB0`、460800/8N1 | 独占设备，与 Robot / CuteCom 等采集者互斥 |

`--source serial/redis` 仅供强制指定来源；地址可用 `--redis-host` / `--redis-port` 修改。
无效 / no-force 遥测也会选中 Redis，随后报告数据不可用，不能因此改连串口。
超过 1 秒没有有效新样本、断连或写盘错误时停止并显示原因；退出只关闭监控会话。

## 数据与时效

- Redis 校验 v1、SI 单位、发布时间、valid / stale、I/O 状态、checksum 可用性、六轴有限值与采样身份；不再次乘 9.80665。
- KWR57 无 checksum 时接受有效完整帧；旧消息缺少 availability 字段时仍要求 checksum。
- 重复 / 旧采样时间不写盘，采样序号重置后重新清零。Pub/Sub 快照不代表完整 500 Hz 采样。
- 串口按 28 字节固定帧解析，发送 `48 AA 0D 0A` 启流，退出有界尝试 `43 AA 0D 0A` 停流；协议无 CRC。
- 样本先写盘，最多 50 Hz 投递显示，GUI 每 100 ms 重绘最近 20 秒。记录为主机接收时间，同批帧可相同；同步写盘不保证无丢帧。

## 模块

| 文件 | 职责 |
| --- | --- |
| [main.py](main.py) | 输入识别、监控线程、窗口与 `--headless` 入口 |
| [redis_source.py](redis_source.py) | 只读订阅、v1 校验和 SI 原始值解析 |
| [kwr57.py](kwr57.py) | RS422 帧解析，kgf / kgf·m 转 N / N·m |
| [zeroing.py](zeroing.py) | 连续稳定 3 秒后的固定零偏、手动重置 |
| [recording.py](recording.py) | 每会话一个 CSV，每秒刷新 |
| [record.py](record.py) | 无界面监控，Ctrl+C 停止后出图，同样自动识别来源 |
| [analyze.py](analyze.py) | 流式统计均值与样本标准差，生成 PNG |

## 清零与输出

保持姿态固定、无外部接触。3 秒窗口内六轴样本标准差上限为
`[0.5 N × 3, 0.01 N·m × 3]`，前后半窗均值差上限为 `[0.1 N × 3, 0.005 N·m × 3]`。
固定零偏不跟随后续载荷 / 漂移；重新清零生成新 `zero_id`，统计和图片分段处理。
清零前只保留原始值，不参与清零后统计。命令、CSV 和图片含义见[使用说明](../../docs/sensor_drift_recording.md)。
