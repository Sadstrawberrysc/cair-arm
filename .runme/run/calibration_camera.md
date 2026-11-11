# 相机标定

```bash {"name":"calibration-environment"}
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main
```

全局 D455：相机固定，棋盘随末端移动；9×6 内角点，格长 7.46 mm。

```bash {"name":"calibrate-realsense","interactive":"true","background":"false"}
python infer/hand_eye_calibration-main/calibrate_realsense.py
```

腕部 Gemini 305：相机随末端移动，棋盘固定；9×6 内角点，格长 25 mm。

```bash {"name":"calibrate-gemini","interactive":"true","background":"false"}
python infer/hand_eye_calibration-main/calibrate_gemini.py
```

棋盘显示 `FOUND` 后按 `s` 保存一组，默认采满 50 组自动求解并部署；按 `q` 退出。
