# 腕部相机真实运动启动

当前入口：无力传感器、候选外参、取消累计位移/转角上限，持续运行直到手动退出或故障停止。

## 1. Redis

```bash
redis-cli -h 127.0.0.1 -p 7777 ping
```

没有返回 `PONG` 时才启动：

```bash
redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
```

## 2. 机械臂终端

先停止其他机器人控制程序，再运行：

```bash
cd ~/uspilot_ctrl-main
bash infer/Camera_wrist/start_robot.sh
```

## 3. 相机终端

```bash
cd ~/uspilot_ctrl-main
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
python infer/Camera_wrist/click_follow.py
```

## 4. 开始与停止

- 等待窗口出现 Robot 状态，左键选点，观测有效后按 `b` 开始真实跟随。
- `p` 暂停；重新选点后按 `r` 恢复。
- `q` 退出相机，机械臂端保持 Hold；不会关闭机械臂进程。
- 在机械臂终端按 `Ctrl+C` 结束；紧急情况使用物理急停。
