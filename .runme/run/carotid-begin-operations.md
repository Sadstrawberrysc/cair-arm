# 腕部模型模式最简启动（Runme notebook）

[模型进展及历史](../Progress/carotid-begin-progress.md) · [模型服务架构](../../infer/Camera_wrist/detector/ARCHITECTURE.md)

原 `Carotid_begin` 已并入 `Camera_wrist/detector/`，本 notebook 保留原文件名。

按 **Redis → 相机 → Robot** 逐个执行下面三个单元，不使用 Run All。
相机与 Robot 各打开独立 Terminator 窗口；相机脚本自动激活环境、加载当前默认模型。
相机先显示 `waiting for Robot state` 属于预期；Robot 就绪后自动选取初始目标，
有效追踪及目标回传确认后自动请求跟随，无需按 `b`。

当前 Robot 入口是真实无力悬停候选试验，关闭累计位移 / 转角及 TCP 位置误差门；
启动前确认探头不接触人体、运动范围有净空，急停可用。

## 1. Redis

已有 `PONG` 时沿用服务，否则在本机后台启动 Redis；新建服务关闭持久化，仅作消息中转。

```bash {"excludeFromRunAll":"true","name":"carotid-start-redis"}
if [[ "$(redis-cli -h 127.0.0.1 -p 7777 ping 2>/dev/null)" != PONG ]]; then
  redis-server --bind 127.0.0.1 --port 7777 --save "" --appendonly no --daemonize yes
fi
redis-cli -h 127.0.0.1 -p 7777 ping
```

## 2. 相机

若启动提示 SDK 枚举到 0 台设备，先检查 Gemini 305 的供电与数据线、重新插拔或直连
电脑 USB 口；相机未识别时重复启动不会恢复连接。序列号不匹配时错误会列出实际设备。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"carotid-start-camera","promptEnv":"never"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh camera
```

## 3. Robot

先停止其他 Robot 控制进程，再启动：

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"carotid-start-robot"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh robot
```

初始选点失败时每隔至少 0.1 秒重试，等待唯一目标、有效深度和新鲜追踪结果。
腕部速度为 TCP 参考 4 mm/s、姿态参考及每关节命令上限 4°/s。
窗口按 `b` 可重新选点并请求跟随；`d` 只选点，`p` 暂停并取消自动开始和恢复。
`q` 退出相机；在 Robot 独立终端按 `Ctrl+C` 停止机械臂控制。
默认模型由 [detector/config.py](../../infer/Camera_wrist/detector/config.py) 的
`DEFAULT_MODEL_ID` 定义，无需传入参数；`CAROTID_MODEL_ID` 环境变量和显式
`--model-id` 可覆盖，显式参数优先。仅模型加载完成不足以开始跟随，仍须通过上述状态和观测检查。
手动选点使用 `bash infer/Camera_wrist/start_camera.sh --manual`，操作说明见
[腕部启动与运行](camera-wrist-operations.md#4-开始与停止)。
模型更换与验证记录见 [进展 notebook](../Progress/carotid-begin-progress.md#更换模型)。
