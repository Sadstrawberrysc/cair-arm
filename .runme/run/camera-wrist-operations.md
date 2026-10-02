# Camera_wrist 启动与运行（Runme notebook）

> 本文件包含真实机器人跟随入口。运动、相机交互和常驻进程代码块必须逐个执行，
> 不得对本 notebook 使用 `runme run --all`。

当前入口：无力传感器、候选外参、取消累计位移/转角上限，持续运行直到手动退出或故障停止。

快速启动：在 Runme 中依次执行 `start-redis-7777`、`start-wrist-robot`、
`start-wrist-camera` 三个单元；Robot 与相机单元各只有一行命令。
也可在仓库根目录的普通终端分别运行
`bash infer/Camera_wrist/launch_wrist_terminal.sh robot` 和
`bash infer/Camera_wrist/launch_wrist_terminal.sh camera`。

## 1. Redis

```bash {"excludeFromRunAll":"true","name":"wrist-redis-ping"}
redis-cli -h 127.0.0.1 -p 7777 ping
```

没有返回 `PONG` 时才启动：

```bash {"excludeFromRunAll":"true","name":"start-redis-7777"}
if [[ "$(redis-cli -h 127.0.0.1 -p 7777 ping 2>/dev/null)" != PONG ]]; then
  redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
fi
redis-cli -h 127.0.0.1 -p 7777 ping
```

## 2. 机械臂终端

先停止其他机器人控制程序，再运行：

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-wrist-robot"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh robot
```

## 3. 相机终端

先确认上面的机械臂终端持续运行，再启动相机。只启动相机时窗口显示
`Robot: offline: waiting for Robot state`，属于预期；Robot 发布有效状态后应显示
`hold`。首次打开 Gemini 时，SDK 流启动后等待 10 秒取得完整 RGB-D；
若始终无帧，启动脚本只重新初始化一次相机（最多两轮），仍失败则保留
`NO FRAMES`/对齐错误和退出码。不要在等待时重复启动相机。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-wrist-camera","promptEnv":"never"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh camera
```

## 4. 开始与停止

- 等待窗口出现 Robot 状态，左键选点，观测有效后按 `b` 开始真实跟随。
- `p` 暂停；重新选点后按 `r` 恢复。
- `q` 退出相机，机械臂端保持 Hold；不会关闭机械臂进程。
- 在机械臂终端按 `Ctrl+C` 结束；紧急情况使用物理急停。

机械臂和相机各自打开独立 Terminator 窗口，与 Runme 执行终端分离。
相机按 `q` 或 `Ctrl+C` 后，窗口显示退出码和日志路径；机械臂在其窗口按 `Ctrl+C`
会走 StopMotion 与静止确认。退出后按 Enter 关闭对应窗口。
启动时若出现 `QObject::moveToThread` 或 OpenCV Qt 字体目录警告，先观察窗口与帧日志；
这两类警告本身不代表进程已退出。不要重复启动相机或机械臂单元。
