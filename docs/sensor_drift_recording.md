# SensorMonitor 监控、记录与出图

SensorMonitor 是独立监控工具。项目由 Robot 采集和控制，通过 Redis 发布遥测；
监控程序只查看数据，清零不改变项目 tare、标定或控制输入。

## 启动

仓库根目录运行，无需指定来源：

```bash
conda activate carotid
python infer/SensorMonitor/main.py
```

启动检查 `127.0.0.1:7777/robot:sensor:v1`，1 秒内有消息则只读订阅 Redis；
无服务或无消息则直连 KWR57 `/dev/ttyUSB0`、460800 baud。窗口与终端显示来源，选定后不自动切换。
无效 / no-force 遥测仍使用 Redis 并报告数据不可用；Redis 数据已是 SI，不重复换算。
串口直连与 Robot / CuteCom 互斥，启动 Robot 前先退出直连会话。

| 可选参数 | 用途 |
| --- | --- |
| `--source serial` / `--source redis` | 排查时强制指定输入，日常无需传入 |
| `--device <路径>` / `--baud <波特率>` | 指定串口设备与速率 |
| `--redis-host <主机>` / `--redis-port <端口>` | 修改 Redis 地址 |

依赖 NumPy、Matplotlib、redis-py；GUI 另需 pyqtgraph 和 Qt，串口不需要 pyserial。
Conda Qt 出现 libstdc++ 冲突时，可在命令前加 `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6`。

## 清零与停止

保持姿态固定、无外部接触，至少连续稳定 3 秒后自动减去固定零偏，显示六轴曲线。
“重新清零”重新采集稳定窗口；“结束采集并出图”停止记录、关闭 CSV 并生成图片。
直接关闭窗口只保存数据；清零不做线性去趋势、温度补偿或自动排除前 30 分钟。

无界面入口同样自动识别来源，Ctrl+C 停止后自动出图：

```bash
python infer/SensorMonitor/record.py
```

重新分析最近记录，或指定会话目录：

```bash
python infer/SensorMonitor/analyze.py
python infer/SensorMonitor/analyze.py infer/SensorMonitor/records/记录目录 --bin-sec 1
```

## 文件与统计

每次会话保存到 `infer/SensorMonitor/records/日期时间/`，不按小时轮转：

| 文件 | 内容 |
| --- | --- |
| `samples.csv` | 接收相对时间 `elapsed_s`、`zero_id`、六轴原始值与 `zero_` 前缀的清零后值 |
| `analysis/mean_std.png` | 六个子图：每个时间箱的均值与 ±1 样本标准差 |

力为 N，力矩为 N·m；默认箱宽 1 秒，标准差分母为 `n−1`，不是标准误或置信区间。
少于两个样本只画均值，缺失时间箱及不同 `zero_id` 之间不连线。
清零前保留原始值，清零后字段留空；重新清零期间不参与统计，无有效清零数据时明确拒绝出图。
旧 `samples_*.csv` 需有 `events.jsonl` 清零事件才能生成清零后图片。

## 记录边界

CSV 每秒刷新，正常停止时关闭；不生成额外 JSON、Markdown、趋势表或拟合报告。
断流、持续无效数据或写盘错误即停止，不自动重连或切源。
时间戳是主机接收时间，同批帧可相同；串口无 CRC，Redis 只记录收到的有效新快照，均不保证完整 500 Hz 或无丢帧。
实现与质量门限见 [SensorMonitor 架构](../infer/SensorMonitor/ARCHITECTURE.md)。
