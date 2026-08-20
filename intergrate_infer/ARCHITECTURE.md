# 超声视觉推理服务架构

## 职责

该服务从超声视频流提取位姿/横向偏差、旋转误差、扫描 phase、分割结果和血管丢失状态，
将其调理为 RM75 Redis v1 控制意图。当前生产入口是
`main_redis_seg_newphase_recovery_mode.py`，其他 `main_*` 文件为历史或实验入口。

## 模块

| 模块 | 职责 |
| --- | --- |
| `main_redis_seg_newphase_recovery_mode.py` | 相机生命周期、推理编排、按键状态、恢复判定、Redis 发布/状态确认 |
| `VisionCommandPublisher` | v1 session、递增 sequence、200 ms 心跳与命令序列化 |
| `VisualYConditioner` | Tool-Y 死区、低通和换向确认 |
| `infer_realtime.py` | 主干网络与姿态/角度推理 |
| `seg_infer.py`、`detect_infer.py` | 分割与目标检测 |
| `models/`、`utils/` | 模型定义、归一化和通用推理工具 |
| `weights/` | 运行时模型权重，不是代码 API |

## 接口

输入包括相机帧、键盘 `b/m/t/q`、本地权重和 Redis status。输出固定发布到
`127.0.0.1:7777/robot:command:channel`，并订阅 `robot:status:channel` 获取 session/sequence
确认。payload schema 见根级 [ARCHITECTURE.md](../ARCHITECTURE.md)。

按键语义：`b` 切换 moving/idle，`m` 请求扫描阶段，`t` 结束本轮，`q` 发布 terminate
后退出。Redis publish 成功不是机器人执行确认，必须以同 session 的 status
`producer_sequence` 判断确认进度。

## 依赖

Python `carotid` 环境；主要依赖 PyTorch、torchvision、OpenCV、NumPy、redis-py、模型权重、
摄像头驱动和可用 CUDA（可用时）。RM75 控制器与本服务仅通过 Redis 协议耦合。

## 架构决策与边界

- 每次进程启动创建新 UUID session；sequence 从 1 严格递增，心跳不可复用旧序号。
- 推理慢、画面冻结或按键等待期间仍每 200 ms 发布最后状态，满足控制端 500 ms fresh 门。
- phase debounce、Y 调理和 recovery 判定归视觉端；力安全、状态机、限位和 ServoJ 归机器人端。
- 不得从 Python 直接连接 RM75 或绕过 `robot:command:channel` 发送运动。
- 路径和模型参数目前部分硬编码；更改入口或权重时须同步更新本文件和运行文档。
