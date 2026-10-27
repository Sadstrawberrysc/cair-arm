# KWR57 采集、清零与均值 ± 标准差图片

仓库根目录运行：

```bash
conda activate carotid
python infer/SensorMonitor/main.py
```

默认直连 KWR57 `/dev/ttyUSB0`、460800 baud，无需启动 Robot、Redis 或 ROS。
同一串口只能由一个程序使用。可用 `--device /dev/ttyUSB1` 修改串口。
如当前 Conda Qt 环境出现 libstdc++ 冲突，可在命令前加
`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6`。
依赖 NumPy、Matplotlib；实时界面另需 pyqtgraph 和 Qt 绑定，无需 redis-py 或 pyserial。

保持姿态固定、无外部接触，等待至少连续 3 秒稳定数据，软件自动计算固定零偏并减去。
界面显示清零后的六轴曲线；“重新清零”重新采集稳定窗口。“结束采集并出图”停止采集、
关闭 CSV，然后生成图片并显示保存路径。直接关闭窗口只保存数据，不出图。
固定零偏处理消除初始偏置，后续残余漂移仍体现在曲线上；不做线性去趋势或温度补偿。
不预设每次采集均已预热，也不自动删除前 30 分钟数据。

无界面采集，Ctrl+C 停止后自动出图：

```bash
python infer/SensorMonitor/record.py
```

重新分析最近一次记录，或指定记录目录：

```bash
python infer/SensorMonitor/analyze.py
python infer/SensorMonitor/analyze.py infer/SensorMonitor/records/记录目录 --bin-sec 1
```

每次记录保存到 `infer/SensorMonitor/records/日期时间/`：

| 文件 | 内容 |
| --- | --- |
| `samples.csv` | 接收相对时间 `elapsed_s`、清零编号 `zero_id`、六轴原始值和 `zero_` 前缀的清零后值 |
| `analysis/mean_std.png` | 六个子图，每个时间箱的均值曲线和 ±1 样本标准差阴影 |

力单位 N，力矩单位 N·m。默认统计窗口 1 秒，标准差分母为 `n−1`，不是标准误或置信区间。
少于两个样本的时间箱只画均值点；缺失时间箱和不同清零编号之间不连线。
清零前 `zero_id` 与清零后数值留空；原始值始终保存。重新清零期间不参与图片统计。
没有稳定清零后的数据时，明确提示无法出图，不使用原始值冒充清零结果。
旧版 `samples_*.csv` 记录如有 `events.jsonl` 中的清零事件也可分析；没有清零事件则不能出清零后图片。

CSV 每秒刷新、正常停止时关闭；不生成额外 JSON、Markdown、趋势表或拟合报告。
采集遇到断流或写盘错误即停止并显示原因，不自动重连。时间戳为主机接收时间，
同一读取批次的多帧可以相同；串口协议无 CRC，不保证严格 500 Hz 或无丢帧。
