# KWR57 RS422 接入

更新：2026-10-26。依据用户提供的 KWR57 系列 RS422 手册第 6～8 页。
用户已确认单次读取和 500 Hz 连续输出正常；本次软件适配未连接真实传感器或机械臂。

## 当前软件行为

- Robot、SensorMonitor 和标定采集工具默认使用 `kwr57-rs422`、460800/8N1，无流控。
- 开始采集发送 `48 AA 0D 0A`；正常退出及 I/O 退出时有界尝试发送 `43 AA 0D 0A`。
- 数据帧固定为 28 字节：`48/49 AA`、六个小端 float32、固定位置 `0D 0A`。
  数据内部的 CRLF 或帧头字节不作为分隔符；支持任意拆包、粘包和噪声恢复。
- 手册的 kg/kg·m 按 kgf/kgf·m 解释，六个分量均乘 9.80665，向上层提供 N、N·m。
  实际出厂单位及受力方向仍待已知载荷确认。
- 协议无 CRC。有效完整帧的 `checksum_available=false`、`checksum_valid=false`；
  `IntegrityValid()` 仍要求样本 valid，旧协议仍要求校验和通过。位错误若保持合法帧和有限值，
  无法仅靠此协议检出；控制层继续执行新鲜度、原始量程及原有力控保护。
- Robot 接收 500 Hz 数据，只向控制循环提供最新快照，控制周期仍为 10 ms。
- SensorMonitor 消费全部数据，按 `--query-period-ms`（默认 20 ms）投递最新帧，
  序号可跳跃；GUI 默认 15 Hz 重绘。
- 未确定 A/B/C/D 型号前，KWR57 有效原始力矩门最多 2 N·m，力门仍为原有 50 N。

## 先单独查看六轴数据

关闭 CuteCom 和其他串口采集进程，再在桌面终端运行：

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 python infer/SensorMonitor/main.py \
  --sensor-protocol kwr57-rs422 --device /dev/ttyUSB0 --baud 460800
```

此命令按现有机制自动选源：Robot 正在发布遥测时读 Redis，否则直连串口。
直连独占串口，退出监视器后才能启动 Robot；Robot 运行后再启动监视器即可读 Redis。
工具使用 Python 标准库串口接口，无需 pyserial。

`/dev/ttyUSB0` 是尚未获知新转换器唯一 ID 时的默认路径；多 USB 串口时需确认实际设备，
部署时建议将已确认的 `/dev/serial/by-id/...` 路径填入配置。
对提供 `latency_timer` 的转换器，可按手册确认其为 1 ms；程序不自动修改系统权限或驱动参数。

## Robot 与标定

无参数 `main_rm75` 仍是既有真机执行入口。新配置见
[rm75_runtime_config.cpp](../infer/Robot/src/rm75_runtime_config.cpp)：

| 项目 | 新默认值 |
| --- | --- |
| 协议 / 波特率 | `kwr57-rs422` / 460800 |
| 串口 | `/dev/ttyUSB0` |
| 标定文件 | 可执行文件旁的 `kwr57_force_calibration.json` |
| 标定身份 | `KWR57`，人工赋予的配置 ID，不是设备读回的序列号 |

本次没有生成或复制标定文件。缺失新标定时，Robot 在连接硬件前拒绝启动。
更换装配后需要重新确认零偏、重力、重心、传感器安装旋转和探头 TCP，不能复制旧文件来绕过。

维护工具 `force_sensor_calibration_capture` 新增 `--sensor-protocol`，默认 KWR57/460800；
采集 CSV 已包含 SI 单位，`force_sensor_calibrate` 直接拟合，不再次乘 9.80665。
构建维护工具时配置 `-DBUILD_MAINTENANCE_TOOLS=ON`。

旧 Haptron 驱动保留：主程序和采集工具支持 `--sensor-protocol haptron-modbus`，
未显式给波特率时使用 115200；旧 AA55 流使用 `legacy-aa55`。
旧协议使用各自设备和标定配置。SensorMonitor 支持 KWR57 与 Haptron 两种直连协议。

## 遥测与构建

Redis `robot:sensor:v1` 新增 `sensor.checksum_available`，缺失时按 true 兼容旧生产者。
旧监视器会因 checksum_valid=false 拒绝 KWR57 样本，需要使用本次更新后的版本。
CSV v2 列不变，KWR57 的 checksum 列为 false；summary.control 中新增
`sensor_protocol`、`sensor_baud`、`sensor_checksum_available` 供诊断。

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DBUILD_MAINTENANCE_TOOLS=OFF
cmake --build infer/Robot/build --target main_rm75 robot_offline_tests
ctest --test-dir infer/Robot/build --output-on-failure
```

按用户要求，已删除本次 KWR57 适配新增的测试代码，保留仓库原有离线测试。
软件构建与离线检查不能代替真实装配标定和机器人运行验收。

## 固定姿态长期漂移记录

SensorMonitor 已支持无参数自动记录、按小时分文件及离线漂移报告；无需更改传感器协议或
Robot 标定。直接运行 main.py 可边看边记录，record.py 可无界面记录，analyze.py 默认分析
最新会话。具体方式见[长期记录说明](sensor_drift_recording.md)。
