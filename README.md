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
cd infer/Camera_RT
conda activate dvpath
python cliff_demo.py
```

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
