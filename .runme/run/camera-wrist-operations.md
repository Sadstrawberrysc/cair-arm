# Camera_wrist 启动与运行（Runme notebook）

> 本文件包含真实机器人跟随入口。运动、相机交互和常驻进程代码块必须逐个执行，
> 不得对本 notebook 使用 `runme run --all`。

当前入口：无力传感器、候选外参、取消累计位移/转角上限，持续运行直到手动退出或故障停止。
相机无参数默认模型模式，Robot 就绪后自动选取初始目标并在观测有效、目标回传确认后请求跟随。
腕部 TCP 参考限速 4 mm/s，姿态参考及每关节命令限速 4°/s；`--manual` 切换鼠标选点。
模型配置见 [detector/config.py](../../infer/Camera_wrist/detector/config.py)，服务边界见
[模型服务架构](../../infer/Camera_wrist/detector/ARCHITECTURE.md)。

快速启动：在 Runme 中依次执行 `start-redis-7777`、`start-wrist-camera`、
`start-wrist-robot` 三个单元；Robot 与相机单元各只有一行命令。
也可在仓库根目录的普通终端分别运行
`bash infer/Camera_wrist/launch_wrist_terminal.sh camera` 和
`bash infer/Camera_wrist/launch_wrist_terminal.sh robot`。

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

## 2. 相机终端

该单元无需模型参数，脚本自动激活 `camera` 环境并加载默认模型；相机后台启动
`ultralytics` 环境中的模型服务。只启动相机时窗口显示
`Robot: offline: waiting for Robot state`，属于预期；Robot 发布有效状态后应显示
`hold`，随后自动选取初始目标并请求跟随。首次打开 Gemini 时，SDK 流启动后等待 10 秒取得完整 RGB-D；
若始终无帧，启动脚本只重新初始化一次相机（最多两轮），仍失败则保留
`NO FRAMES`/对齐错误和退出码。不要在等待时重复启动相机。

若启动报告“Orbbec SDK 枚举到 0 台设备”，说明 SDK 未识别腕部相机，需检查相机端供电
与数据线，重新插拔或直连电脑 USB 口后再启动。若枚举到其他序列号，错误会列出设备名
和实际序列号，应核对设备身份及对应标定后再配置 `--serial`。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-wrist-camera","promptEnv":"never"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh camera
```

## 3. 机械臂终端

先停止其他机器人控制程序，再运行；Robot 就绪后，默认模型相机将自动选点并请求跟随。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-wrist-robot"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh robot
```

## 4. 开始与停止

- 默认模型模式：等待新鲜 Robot Hold 和完整 RGB-D，自动从新帧选取初始目标，目标回传确认后请求真实跟随，无需按 `b`。选点失败时每隔至少 0.1 秒重试。
- `d` 只选点，不请求开始；已有开始历史时，`r` 可对有效新鲜目标请求恢复。
- `p` 暂停、清除目标并取消自动开始和恢复；重新按 `b` 可完成新选点和恢复。
- `q` 退出相机，机械臂端保持 Hold；不会关闭机械臂进程。
- 在机械臂终端按 `Ctrl+C` 结束；紧急情况使用物理急停。

手动选点时，用下面单元替代默认相机单元；若相机已在运行，先退出已有相机。
该单元在当前执行终端运行，显示相机窗口并保存日志，
不加载模型。左键选点后使用同一光流和深度跟踪，观测有效时按 `b` 请求开始；
暂停后重新左键选点，再按 `r` 请求恢复。

```bash {"background":"false","closeTerminalOnSuccess":"false","excludeFromRunAll":"true","interactive":"true","name":"start-wrist-camera-manual","promptEnv":"never"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/start_camera.sh --manual
```

默认模型入口的机械臂和相机各自打开独立 Terminator 窗口，与 Runme 执行终端分离。
相机按 `q` 或 `Ctrl+C` 后，窗口显示退出码和日志路径；机械臂在其窗口按 `Ctrl+C`
会走 StopMotion 与静止确认。退出后按 Enter 关闭对应窗口。
启动时若出现 `QObject::moveToThread` 或 OpenCV Qt 字体目录警告，先观察窗口与帧日志；
这两类警告本身不代表进程已退出。不要重复启动相机或机械臂单元。

## 5. 模型配置与凭据检查

默认模型 ID 由 [detector/config.py](../../infer/Camera_wrist/detector/config.py) 的
`DEFAULT_MODEL_ID` 定义；显式 `--model-id` 优先于 `CAROTID_MODEL_ID` 环境变量，
环境变量优先于内置默认值。日常默认启动无需设置两者。
缓存位于 `Camera_wrist/data/model_cache/`，可由 `MODEL_CACHE_DIR` 覆盖。
凭据继续读取 `ROBOFLOW_API_KEY` 或 `~/.config/carotid_begin/roboflow_api_key`。

下面单元只检查凭据是否可用，不显示密钥，也不加载模型或连接硬件：

```bash {"excludeFromRunAll":"true","name":"check-wrist-model-key"}
cd /home/cair-jacen/uspilot_ctrl-main
/home/cair-jacen/anaconda3/envs/ultralytics/bin/python -B -m infer.Camera_wrist.detector.credentials check
```

## 6. 离线验证

统一测试入口包含手动和默认模型模式、跟随状态机、检测管线及私有 socket 协议；
使用假模型和合成数据，不启动相机、Redis 或机械臂。最近一次实际结果与真机待验收项见
[腕部进展](../Progress/camera-wrist-progress.md)。

```bash {"excludeFromRunAll":"true","name":"test-wrist-offline"}
cd /home/cair-jacen/uspilot_ctrl-main
/home/cair-jacen/anaconda3/envs/camera/bin/python -B -m unittest discover -s infer/Camera_wrist/tests -v
```
