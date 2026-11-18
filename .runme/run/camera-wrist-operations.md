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

TCP 位置参考限速为 3 mm/s，姿态参考及每关节命令限速为 3°/s。
TCP 位置误差大于 7 mm 才启动平移，收敛到 3 mm 内时停止。

```bash {"excludeFromRunAll":"true","interactive":"true","name":"start-wrist-robot"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/launch_wrist_terminal.sh robot
```

## 相机参数与按键

启动器的 `camera`、`robot` 分别选择相机和机械臂终端。相机可选参数直接传给
`start_camera.sh`；例如，在仓库根目录使用 `bash infer/Camera_wrist/start_camera.sh --manual`
启动手动选点模式。独立终端启动器只接收组件名称，不转发相机参数。

| 相机参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--serial` | `CV2L360000HZ` | Gemini 305 设备序列号 |
| `--redis-port` | `7777` | 本机 Redis 通信端口 |
| `--calibration` | `gemini305_to_rm75_armtip.json` | 腕部 Camera → ArmTip 外参，默认位于 `infer/Camera_wrist/` |
| `--model-id` | [模型配置](../../infer/Camera_wrist/detector/config.py)中的默认模型 | 显式参数优先于 `CAROTID_MODEL_ID` 环境变量，再使用源码默认值 |
| `--model-class` | `carotid` | 检测目标类别 |
| `--model-confidence` | `0.4` | 模型选点置信度阈值，范围 0～1 |
| `--manual` | 不启用 | 改用鼠标选点，不加载检测模型；与 `--model-id` 互斥 |

采集分辨率、帧率和视差统一由 [gemini_config.py](../../infer/Camera_wrist/gemini_config.py)
定义，启动时校验实际可用档位；不提供对应 CLI 参数。

默认模型模式在新鲜 Robot Hold 状态、有效目标和目标回执就绪后自动请求首次跟随。
相机窗口操作如下：

| 操作 | 行为 |
| --- | --- |
| `b` | 模型模式重新选点并请求跟随，需要新鲜 Robot Hold；手动模式在鼠标选点后请求首次开始 |
| `d` | 仅模型模式：选点，不请求开始；跟随中需先暂停 |
| 鼠标左键 | 仅手动模式：选择新目标 |
| `r` | 已有跟随历史时，使用有效的新鲜目标请求恢复 |
| `p` | 请求暂停，取消待开始请求并清除当前跟踪目标 |
| `q` | 请求结束并退出相机 |

暂停后，模型模式可用 `b` 重新选点并跟随；手动模式需重新点击选点。
机械臂终端按 `Ctrl+C` 结束控制进程。

## 更换模型

通过 `CAROTID_MODEL_ID` 覆盖源码默认模型，或向 `start_camera.sh` 显式传入 `--model-id`。
使用上面的独立相机终端单元时，可先在启动它的同一终端导出
`CAROTID_MODEL_ID='新的完整 Model ID'`。类别改变时直接调用相机脚本，并传入 `--model-class`。

以下为填写模型 ID 和类别后使用的模板：

```bash {"name":"start-wrist-custom-model-template","ignore":"true"}
bash /home/cair-jacen/uspilot_ctrl-main/infer/Camera_wrist/start_camera.sh \
  --model-id '新的完整 Model ID' --model-class 'carotid'
```

API Key 保存在本机私有凭据文件或 `ROBOFLOW_API_KEY` 环境变量中，不写入启动命令。
检查凭据是否可用（只显示存在状态）：

```bash {"name":"check-wrist-model-credentials","excludeFromRunAll":"true"}
cd /home/cair-jacen/uspilot_ctrl-main
/home/cair-jacen/anaconda3/envs/ultralytics/bin/python -m infer.Camera_wrist.detector.credentials check
```

## 离线 Redis 回环验证

需要已构建的 `infer/Robot/build/main_rm75`、本机 Redis 和 `camera` 环境的 numpy / redis-py。
脚本使用独立的本地临时 Redis 端口和显式模拟 Robot，不使用 7777，不连接相机或真实机械臂。

```bash {"name":"check-wrist-simulated-loopback","excludeFromRunAll":"true"}
cd /home/cair-jacen/uspilot_ctrl-main
/home/cair-jacen/anaconda3/envs/camera/bin/python \
  infer/Camera_wrist/tests/loopback_check.py --no-force --candidate-trial --unlimited-excursion \
  --unlimited-position-tracking-error --auto-recovery
```
