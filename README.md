# USPilot_control

## Run
**image infer**
```bash
conda activate usrobot
cd scan_pilot/intergrate_infer
python main_redis_seg.py
```

**robot**
```bash
cd infer/Robot/build
sudo ./main
```

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
