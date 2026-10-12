# USPilot 本地 Runme notebooks

本目录保存启动操作、项目进展、验证证据和实验历史，Markdown notebooks 纳入 Git。当前系统事实和接口以
根级及模块级 `ARCHITECTURE.md` 为准。

| Notebook | 内容 |
| --- | --- |
| [run/system-operations.md](run/system-operations.md) | 全系统常用入口和 Runme CLI 用法 |
| [run/robot-operations.md](run/robot-operations.md) | RM75、Redis、视觉与传感器监视启动 |
| [run/camera-rt-operations.md](run/camera-rt-operations.md) | 全局相机预检、功能入口与快照操作 |
| [run/camera-wrist-operations.md](run/camera-wrist-operations.md) | 腕部相机默认模型 / 手动选点、配置检查与离线测试 |
| [run/carotid-begin-operations.md](run/carotid-begin-operations.md) | 腕部模型模式的 Redis、相机和 Robot 最简启动（保留原 notebook 名） |
| [run/calibration-operations.md](run/calibration-operations.md) | Gemini/D455 标定与单点 MoveJ 编排 |
| [Progress/project-progress.md](Progress/project-progress.md) | 根级项目进展与运行边界历史 |
| [Progress/camera-rt-progress.md](Progress/camera-rt-progress.md) | Camera_RT 进展、证据与待办 |
| [Progress/camera-wrist-progress.md](Progress/camera-wrist-progress.md) | 腕部模块当前状态、进展、证据与待办 |
| [Progress/carotid-begin-progress.md](Progress/carotid-begin-progress.md) | 腕部 detector 模型、跟随、恢复与验证历史（原 Carotid_begin） |
| [Progress/yolo-roboflow-visual-servo-plan.md](Progress/yolo-roboflow-visual-servo-plan.md) | YOLO + Roboflow 早期实施计划及历史记录 |

原 `infer/Carotid_begin/` 模型服务已并入 `infer/Camera_wrist/detector/`。
无参数启动默认使用模型，手动模式传入 `--manual`；当前接口见
[腕部架构](../infer/Camera_wrist/ARCHITECTURE.md)与[模型服务架构](../infer/Camera_wrist/detector/ARCHITECTURE.md)。

腕部日常操作统一从 [camera-wrist-operations.md](run/camera-wrist-operations.md) 进入，
依次执行 `start-redis-7777` → `start-wrist-camera` → `start-wrist-robot`。
相机与 Robot 通过各自独立终端运行；Robot 就绪后模型模式自动选取初始目标，观测及回传确认有效后请求跟随，无需按 `b`。
手动模式使用同一 notebook 的 `start-wrist-camera-manual` 单元替代模型相机单元。
原 carotid 命名的启动和进展 notebook 保留文件名，当前入口与历史记录均链接到合并后的模块。

从仓库根目录的普通终端列出一个 notebook 的单元：

```bash {"ignore":"true"}
cd /home/cair-jacen/uspilot_ctrl-main
runme list --filename .runme/run/robot-operations.md --chdir /home/cair-jacen/uspilot_ctrl-main
```

从普通终端执行安全的命名单元（示例）：

```bash {"ignore":"true"}
cd /home/cair-jacen/uspilot_ctrl-main
runme run git-status --filename .runme/run/system-operations.md --chdir /home/cair-jacen/uspilot_ctrl-main
```

所有 notebook 均按单元执行。不要在本项目使用 `runme run --all`，因为操作 notebook
包含相机、串口、常驻进程和真实机器人运动入口。
