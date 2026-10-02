# USPilot 本地 Runme notebooks

本目录保存启动操作、项目进展、验证证据和实验历史，Markdown notebooks 纳入 Git。当前系统事实和接口以
根级及模块级 `ARCHITECTURE.md` 为准。

| Notebook | 内容 |
| --- | --- |
| `run/system-operations.md` | 全系统常用入口和 Runme CLI 用法 |
| `run/robot-operations.md` | RM75、Redis、视觉与传感器监视启动 |
| `run/camera-rt-operations.md` | 全局相机预检、功能入口与快照操作 |
| `run/camera-wrist-operations.md` | 腕部相机、投影与真实跟随入口 |
| `run/carotid-begin-operations.md` | Carotid_begin 的 Redis、模型相机和 Robot 最简启动 |
| `run/calibration-operations.md` | Gemini/D455 标定与单点 MoveJ 编排 |
| `Progress/project-progress.md` | 根级项目进展与运行边界历史 |
| `Progress/camera-rt-progress.md` | Camera_RT 进展、证据与待办 |
| `Progress/camera-wrist-progress.md` | Camera_wrist 进展、证据与待办 |
| `Progress/carotid-begin-progress.md` | Carotid_begin 模型、跟随、恢复与验证历史 |
| `Progress/yolo-roboflow-visual-servo-plan.md` | YOLO + Roboflow 自动目标检测到腕部视觉伺服的实施计划 |

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
