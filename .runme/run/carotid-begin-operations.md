# Carotid_begin 最简启动（Runme notebook）

[项目进展](../Progress/carotid-begin-progress.md) · [架构](../../infer/Carotid_begin/ARCHITECTURE.md)

按 **Redis → 相机 → Robot** 逐个执行下面三个单元，不使用 Run All。
相机与 Robot 各打开独立 Terminator 窗口；相机脚本自动激活环境、加载当前默认模型。
相机先显示 `waiting for Robot state` 属于预期，Robot 就绪后再按 `b`。

当前 Robot 入口是真实无力悬停候选试验，关闭累计位移 / 转角及 TCP 位置误差门；
启动前确认探头不接触人体、运动范围有净空，急停可用。

## 1. Redis

已有 `PONG` 时沿用服务，否则在本机后台启动 Redis；新建服务关闭持久化，仅作消息中转。

```bash {"name":"carotid-start-redis","excludeFromRunAll":"true"}
if [[ "$(redis-cli -h 127.0.0.1 -p 7777 ping 2>/dev/null)" != PONG ]]; then
  redis-server --bind 127.0.0.1 --port 7777 --save "" --appendonly no --daemonize yes
fi
redis-cli -h 127.0.0.1 -p 7777 ping
```

## 2. 相机

```bash {"name":"carotid-start-camera","background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","promptEnv":"never"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh camera
```

## 3. Robot

先停止其他 Robot 控制进程，再启动：

```bash {"name":"carotid-start-robot","background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh robot
```

窗口按 `b` 自动新选点并请求开始；`d` 只选点，`p` 暂停并取消自动恢复。
`q` 退出相机；在 Robot 独立终端按 `Ctrl+C` 停止机械臂控制。
当前模型以 [start_camera.sh](../../infer/Camera_wrist/start_camera.sh) 为准，
模型更换与验证记录见 [进展 notebook](../Progress/carotid-begin-progress.md#更换模型)。
