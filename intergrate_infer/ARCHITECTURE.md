# 超声视觉推理服务架构

## 职责

该服务从超声视频流提取位姿/横向偏差、旋转误差、扫描 phase、分割结果和血管丢失状态，
将其调理为 RM75 Redis v1 控制意图。当前生产入口是
`main_redis_seg_newphase_recovery_mode.py`，其他 `main_*` 文件为历史或实验入口。

## 模块

| 模块 | 职责 |
| --- | --- |
| `main_redis_seg_newphase_recovery_mode.py` | 相机生命周期、推理编排、按键状态、恢复判定、Redis 发布/状态确认 |
| `scan_recording.py` | 独立线程保存配对超声 / tare 后六维力 AVI、时间戳 CSV 与录制元数据 |
| `VisionCommandPublisher` | v1 session、递增 sequence、200 ms 心跳与命令序列化 |
| `visual_y_alignment.py` | 当前血管图像中心的误差、目标关联、死区 / 滞回 / 换向确认、视觉指令时效门 |
| `infer_realtime.py` | 主干网络与姿态/角度推理 |
| `seg_infer.py`、`detect_infer.py` | 分割与目标检测 |
| `models/`、`utils/` | 模型定义、归一化和通用推理工具 |
| `weights/` | 运行时模型权重，不是代码 API |

## 接口

输入包括相机帧、键盘 `b/m/t/q`、本地权重和 Redis status。输出固定发布到
`127.0.0.1:7777/robot:command:channel`，并订阅 `robot:status:channel` 获取 session/sequence
确认。payload schema 见 [Robot Redis 接口](../infer/Robot/ARCHITECTURE.md#redis-接口)。

按键语义：`b` 切换 moving/idle，`m` 请求扫描阶段，`t` 结束本轮，`q` 发布 terminate
后退出。Redis publish 成功不是机器人执行确认，必须以同 session 的 status
`producer_sequence` 判断确认进度。

运行步骤见 [Robot 操作](../.runme/run/robot-operations.md#4-启动视觉程序)。

默认视频输入为采集卡固定路径
`/dev/v4l/by-id/usb-MACROSILICON_USB3.0_UHD_20210623-video-index0`（1920×1080、30 FPS），
避免 `/dev/videoN` 编号变化后误选 RealSense。更换采集卡时，可用环境变量
`USPILOT_ULTRASOUND_CAMERA=/dev/video8` 或对应的 `/dev/v4l/by-id/…` 路径覆盖。

## 依赖

Python `carotid` 环境；主要依赖 PyTorch、torchvision、OpenCV、NumPy、redis-py、模型权重、
摄像头驱动和可用 CUDA（可用时）。RM75 控制器与本服务仅通过 Redis 协议耦合。

## 架构决策与边界

- 每次进程启动创建新 UUID session；sequence 从 1 严格递增，心跳不可复用旧序号。
- 心跳每 200 ms 维持通信；本次视觉采集超过 300 ms 未更新时清零 Y/RZ、phase=-1、停止 recovery 搜索，保留 Z 力控使能。已锁存 terminate 仍继续重发，不撤销终止。
- phase debounce、Y 调理和 recovery 判定归视觉端；力安全、状态机、限位和 ServoJ 归机器人端。
- 不得从 Python 直接连接 RM75 或绕过 `robot:command:channel` 发送运动。
- 路径和模型参数目前部分硬编码；更改入口或权重时须同步更新本文件和运行文档。

## 配对录像

生产入口成功完成模型加载和 idle 握手后自动创建 `output/scan_<时间>_<id>/`，
后台线程以 30 FPS 写出 `ultrasound.avi` 与 `wrench_tare.avi`，并保存六维数值和帧时间戳 CSV。
超声来源是推理输入的 557×640 ROI（编码补一列黑边），独立于分割 / YOLO 开关；
固定帧率下重复最新图像，叠加实际接收帧龄，避免慢推理使视频快放。
力源是 `robot:sensor:v1.compensated_wrench_tool`，必须带 `runtime_tare_applied=true` 且有效 / 新鲜，
缺失时标记等待 / 无效 / 过期，曲线断开而非补零。录像不发控制命令，不重复 tare。
两段视频使用相同录制时刻，`frames.csv` 保留超声接收和力采样单调时钟时间用于核对，
不宣称曝光级硬件同步。异常或 SIGINT 时 finally 关闭编码器；硬杀进程不能保证 AVI 收尾。

六维曲线横轴从本次录制 0 秒扩展到当前时刻，直至退出，最后一帧覆盖整次录制。
显示采用最多 512 个自适应时间箱保存 min/max 包络及缺口标记，避免长时间历史令绘图成本持续增长；
完整收到的逐样本数据保存在 CSV，不受图表分箱影响。

## Tool-Y 图像居中

Tool-Y 使用当前有效血管中心，不再以 MAE pose head 的回归位移直接驱动；该模型输出仅保留终端诊断。
YOLO 与分割使用相同的深度范围 20～500 px，不再单独把 YOLO 限在 200～350 px。
优先选择落在 YOLO 框内的分割区域中心，否则用有效 YOLO 中心；无 YOLO 时使用单一分割中心，
多个分割目标仅在可关联到上一目标时接管。60 px 以上的突然跳变拒绝，失去目标 0.5 s 后可重新获取。
分割权重缺失时禁用分割，不能把随机初始化模型用于控制。

误差为 `(ROI中心x - 血管中心x) × image_total_height_mm / ROI高度 / 1000`，
默认成像高度 37 mm，单位 m，假设像素等比例；需与设备当前成像深度保持一致。
沿用 Robot `model_y_direction=-1`：图像右偏产生负误差并要求 +Tool-Y，左偏相反，
与既有血管丢失恢复侧别一致。门控和限速后的指令为比例速度输入，不是逐帧累加完整误差。
0.5 mm 死区、0.8 mm 重新启动阈值、连续两帧方向确认、EMA alpha=0.35，输出绝对值不超过 4 mm；
按当前 Robot 增益 1/s 对应未缩放 4 mm/s，旋转阶段仍遵守 Robot 原 0.3 缩放。

只有当前有效且未过期的图像误差 ≤0.8 mm 时才放行普通 X/RZ 和自动完成判定；偏心时先继续 Y/Z。
没有目标、目标有歧义或图像超过 300 ms 时普通横向指令归零，不把零指令当作已居中；
已有明确的血管丢失 recovery 搜索不受普通目标可见性门阻塞，但仍受心跳视觉时效门约束。
Robot 接触力门、卸载、跟踪误差暂停、限位和 planner 保护保持原样。
按用户要求，超声窗口沿用原显示布局、`y:` / `On Track`、箭头及既有叠加，不增加中心线、目标标记或新状态文字；居中诊断仅输出终端。
按用户要求不新增 `tool_y_alignment.csv`，只保留原有视频、力数值和帧时间戳记录。
