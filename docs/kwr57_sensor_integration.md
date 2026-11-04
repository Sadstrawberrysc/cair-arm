# KWR57 RS422 接入

核对日期：2026-10-30。协议依据用户提供的 KWR57 系列 RS422 手册第 6～8 页。
用户已确认单次读取和 500 Hz 连续输出正常；下方软件验证记录不包含真实传感器或机械臂验收。

## 项目与监控职责

| 范围 | 职责 |
| --- | --- |
| Robot 项目链路 | 串口采集 → SI 补偿 / 接触估计 / 控制，并向 Redis 发布遥测 |
| SensorMonitor | 可选独立监控；启动自动识别项目 Redis 遥测或串口直连，只做本地显示 / 记录 / 出图 |
| 标定维护工具 | 采样和拟合新装配参数，独立于监控清零 |

## 协议与控制边界

- 项目由 Robot 的 `ForceSensorReader` 采集 KWR57，默认 `kwr57-rs422`、460800/8N1，无流控；遥测通过 Redis 发布。
- 开始采集发送 `48 AA 0D 0A`；正常退出及 I/O 退出时有界尝试发送 `43 AA 0D 0A`。
- 数据帧固定为 28 字节：`48/49 AA`、六个小端 float32、固定位置 `0D 0A`。
  数据内部的 CRLF 或帧头字节不作为分隔符；支持任意拆包、粘包和噪声恢复。
- 手册的 kg/kg·m 按 kgf/kgf·m 解释，六个分量均乘 9.80665，向上层提供 N、N·m。
  实际出厂单位及受力方向仍待已知载荷确认。
- 协议无 CRC。有效完整帧的 `checksum_available=false`、`checksum_valid=false`；
  `IntegrityValid()` 仍要求样本 valid，旧协议仍要求校验和通过。位错误若保持合法帧和有限值，
  无法仅靠此协议检出；控制层继续执行新鲜度、原始量程及原有力控保护。
- Robot 接收 500 Hz 数据，只向控制循环提供最新快照，控制周期仍为 10 ms。
- SensorMonitor 是可选独立监控工具，默认自动识别：有项目遥测时只读订阅 Redis，否则串口直连，无需指定来源。
  本地记录 / 清零与 Robot 控制互不替代；最多 50 Hz 投递显示，GUI 每 100 ms 重绘。
- 未确定 A/B/C/D 型号前，KWR57 有效原始力矩门最多 2 N·m，力门仍为原有 50 N。

## 独立监控：SensorMonitor

日常直接运行 `python infer/SensorMonitor/main.py`，无需传入 `--source redis`。
需要强制独立检查原始串口时，先关闭 Robot、CuteCom 等占用者，再运行：

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 python infer/SensorMonitor/main.py \
  --source serial --device /dev/ttyUSB0 --baud 460800
```

此直连命令独占串口，启动 Robot 前先退出。SensorMonitor 不向项目转发采样或控制命令。
需要额外查看项目运行数据时，直接运行 `python infer/SensorMonitor/main.py` 即可自动识别并只读订阅；
项目自身不依赖监控窗口。
工具使用 Python 标准库串口接口，无需 pyserial。

`/dev/ttyUSB0` 是尚未获知新转换器唯一 ID 时的默认路径；多 USB 串口时需确认实际设备，
部署时建议将已确认的 `/dev/serial/by-id/...` 路径填入配置。
对提供 `latency_timer` 的转换器，可按手册确认其为 1 ms；程序不自动修改系统权限或驱动参数。

## 项目采集、Redis 与标定

无参数 `main_rm75` 是真机执行入口。有效默认配置见
[rm75_runtime_config.cpp](../infer/Robot/src/rm75_runtime_config.cpp)：

| 项目 | 默认值 |
| --- | --- |
| 协议 / 波特率 | `kwr57-rs422` / 460800 |
| 串口 | `/dev/ttyUSB0` |
| 标定文件 | 可执行文件旁的 `rm75_force_calibration.json` |
| 标定身份 | `KWR57`，人工赋予的配置 ID，不是设备读回的序列号 |

当前无参数入口复用部署文件名 `rm75_force_calibration.json`，并要求 `sensor_id=KWR57`。
缺失文件或身份不匹配时，Robot 在连接硬件前拒绝启动；文件名本身不代表已完成新传感器标定。
更换装配后需要重新确认零偏、重力、重心、传感器安装旋转和探头 TCP，不能复制旧文件来绕过。

维护工具 `force_sensor_calibration_capture` 支持 `--sensor-protocol`，默认 KWR57/460800；
采集 CSV 已包含 SI 单位，`force_sensor_calibrate` 直接拟合，不再次乘 9.80665。
构建维护工具时配置 `-DBUILD_MAINTENANCE_TOOLS=ON`。

旧 Haptron 驱动保留：主程序和采集工具支持 `--sensor-protocol haptron-modbus`，
未显式给波特率时使用 115200；旧 AA55 流使用 `legacy-aa55`。
旧协议使用各自设备和标定配置。SensorMonitor 直连仅支持 KWR57，Redis 模式可监看 Robot 发布的其他协议遥测。

## 遥测与构建

Redis `robot:sensor:v1` 新增 `sensor.checksum_available`，缺失时按 true 兼容旧生产者。
遥测消费者不能单独用 `checksum_valid` 判断 KWR57 是否有效，应同时检查
`valid`、`sensor.stale`、`io_status`、`io_error` 和 `checksum_available`；
仅在校验和可用时要求 `checksum_valid=true`。`raw_wrench_sensor` 已是 SI，不能再次乘 9.80665。
既有 `sensor_data` 四元素接口继续由 Robot 发布，超声推理端无需更改传感器协议。
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

## 历史验证（2026-10-30）

监控记录命令和输出见[使用说明](sensor_drift_recording.md)。以下为该日已有记录，不代表文档整理时重新执行：

main_rm75、robot_offline_tests、force_sensor_calibration_capture 和
force_sensor_calibrate 构建成功，CTest 7/7 通过。临时离线检查覆盖 C++/Python 任意拆包、
单位一致性、噪声恢复、非有限值拒绝，以及 C++ 伪终端启流/停流命令、最新快照和过期判定。
临时检查当时保存于 `/tmp`，未加入仓库；未连接真实传感器或启动机器人。
