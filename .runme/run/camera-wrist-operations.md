# 腕部相机运行

## Redis

```bash {"excludeFromRunAll":"true","name":"start-redis-7777"}
if [[ "$(redis-cli -h 127.0.0.1 -p 7777 ping 2>/dev/null)" != PONG ]]; then
  redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
fi
```

## 相机

```bash {"excludeFromRunAll":"true","interactive":"true","name":"start-wrist-camera","promptEnv":"never"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh camera
```

## 机械臂

```bash {"excludeFromRunAll":"true","interactive":"true","name":"start-wrist-robot"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh robot
```
