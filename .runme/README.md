# USPilot Runme 索引

本目录保存本机操作、进展与验证历史，Markdown 随 Git 维护。当前接口以
[系统架构](../ARCHITECTURE.md)及模块架构为准；跨机器使用时调整设备、环境和绝对路径。

2026-11-11 已按用户要求清理历史运行日志、录像和非标定采样，各类保留最近一轮；
最新超声录像尚无收尾元数据，同时保留上一轮完整录像。当前标定、原始标定数据和模型权重保留。
进展文档中的旧输出路径保留作历史出处，部分原始记录已删除。

## 操作手册

| Notebook | 内容 |
| --- | --- |
| [全系统入口](run/system-operations.md) | 常用入口与状态检查 |
| [Robot](run/robot-operations.md) | 构建、Redis、超声闭环与停止 |
| [全局相机](run/camera-rt-operations.md) | 相机快照到初始位姿 |
| [腕部跟随](run/camera-wrist-operations.md) | Redis、相机、机械臂最简启动 |
| [相机标定](run/calibration_camera.md) | RealSense / Gemini 采集到求解 |
| [力传感器标定](run/calibration_force.md) | 1000 姿态自动静态采集、拟合与部署 |
| [RM75 学习路径](rm75_learning_path.md) | 源码、协议、控制和日志阅读顺序 |

腕部日常启动使用独立终端，依次执行 `start-redis-7777` → `start-wrist-camera` →
`start-wrist-robot`。模型模式在 Robot 就绪后自动选点并请求跟随。
原 `Carotid_begin` 已并入 `Camera_wrist/detector/`。

## 独立工具与规划

| 文档 | 范围 |
| --- | --- |
| [SensorMonitor 使用说明](../docs/sensor_drift_recording.md) | 直接启动，自动识别 Redis / 串口，只读监控；不参与项目控制 |
| [KWR57 接入](../docs/kwr57_sensor_integration.md) | 协议、SI 单位、项目采集和标定 |
| [Collision 架构](../infer/Collision/ARCHITECTURE.md)、[实现路线](../infer/Collision/README.md)、[只读监测](../infer/Collision/MONITOR.md) | C++17 几何／FCL 检查、单帧占据、SDK 链接、3D 回放；显式 live 可读 D455 和机械臂状态，不发送运动或停止指令 |
| [Collision 架构](../infer/Collision/ARCHITECTURE.md)、[规划操作](../infer/Collision/PLANNING.md)、[进展](../infer/Collision/PROGRESS.md) | 人工场景 OMPL 关节规划、全路径复查、保存／校验重放和动画；`start_plan.sh` 默认合成示例，无设备连接 |

## 进展与历史

| Notebook | 内容 |
| --- | --- |
| [项目进展](Progress/project-progress.md) | 当前配置、待办、跨模块末端替换计划与验收记录 |
| [力传感器进展](Progress/force-sensor-progress.md) | 力标定、启动预检及末端工具几何变更记录 |
| [全局相机进展](Progress/camera-rt-progress.md) | 设备、定位、外参与历史证据 |
| [腕部进展](Progress/camera-wrist-progress.md) | 跟随、启动诊断、时序与验证 |
| [模型进展](Progress/carotid-begin-progress.md) | detector 及原 Carotid_begin 试验历史 |
| [早期实施计划](Progress/yolo-roboflow-visual-servo-plan.md) | 数据、模型与视觉伺服方案沿革 |

## Runme CLI

从普通终端列出或执行指定单元：

```bash {"ignore":"true"}
cd /home/cair-jacen/uspilot_ctrl-main
runme list --filename .runme/run/robot-operations.md --chdir /home/cair-jacen/uspilot_ctrl-main
runme run git-status --filename .runme/run/system-operations.md --chdir /home/cair-jacen/uspilot_ctrl-main
```

所有 notebook 按单元执行；含相机、串口、常驻进程和真实运动入口，禁止 `runme run --all`。
