# USPilot_control

## Run
**image infer**
```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
cd /home/cair-jacen/uspilot_ctrl-main/intergrate_infer
python main_redis_seg_newphase_recovery_mode.py
```

**robot**
```bash
cmake -S infer/Robot -B infer/Robot/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build infer/Robot/build --target main_rm75

# Default build contains one runtime entry.
infer/Robot/build/main_rm75
```

Original diagnostics and calibration utilities remain in the repository but
are excluded from the default runtime build. Build them only when needed:

```bash
cmake -S infer/Robot -B infer/Robot/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build
```

The migrated implementation follows the original source responsibilities:
sensor protocol/calibration lives in `force_sensor.cpp`, force control and
ServoJ planning in `rm75_control.cpp`, and Redis ownership in the final
`main_rm75.cpp` entry. Original project sources are preserved.

The original `main`/`main_nomove` and `robot_control` files are retained under
`infer/Robot/tests/legacy/six_axis/` and are not used as the RM75 production
entry. Diagnostics and calibration CLIs live under `infer/Robot/tests/tools/`.
See
[RM75 七轴系统迁移总进度](PROGRESS.md) and
[RM75 学习路径](docs/rm75_learning_path.md) before any
hardware run; use a current RM75 calibration and keep the physical emergency
stop available for every `main_rm75` production run. The complete startup
sequence is in [RM75 构建与启动命令](docs/rm75_build_and_start_commands.md).

**contact**
```bash
cd infer/ContactPointShow
conda activate ics
python main.py
```

**camera**
```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/Camera_RT
python tools/preflight.py --static
python tools/preflight.py --runtime
# 只有两级预检均无 BLOCKED 后才启动功能入口
python cliff_demo.py
```

Camera_RT 当前只输出 RealSense 相机光学坐标，不提供已标定的机器人轨迹。环境、资产和
新机器验证状态见 [Camera_RT 项目指南](infer/Camera_RT/AGENTS.md)。

颈动脉单点链路采用文件快照：Camera_RT 中按 `s` 保存路径，再用 Calibration 的 `convert`
生成 Base 路径与摘要。Robot 只通过维护工具读取第一点、沿外法向悬停 50 mm 并求解一次
MoveJ 目标。当前 `arm_probe_pose --artery-path-base ...` 隐式使用首点、50 mm 偏置和启动
Probe TCP 姿态；先用 `--inspect-transform-only` 做无连接检查，再在现场运行不带 `--execute` 的 dry-run；
完整命令和当前阻塞见 [Calibration 项目指南](infer/Calibration/AGENTS.md)。

## 运行xiaokai
### redis服务
```
cd /home/cair/Projects/demo/uspilot_ctrl/SonoScape_api
python redis_service.py
```
### ui
```
conda activate py-xiaozhi
cd /home/cair/Projects/demo/uspilot_ctrl/py-xiaokai
python main.py --mode chatgui
```

### 查看服务器log
```
docker logs -f xiaozhi-esp32-server
```
