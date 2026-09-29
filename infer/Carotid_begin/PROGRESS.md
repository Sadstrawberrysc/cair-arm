# Carotid_begin 进度与启动命令

## 当前状态（2026-09-29）

- 当前模型 ID：`songjiatong/carotid-detect-c9gi9-8-rfdetr-medium-t1`；
   检测类别为 `carotid`。本机已保存 API Key。
- 已在 `ultralytics` 环境安装 ONNX Runtime GPU 1.20.2。新模型的真实
   ONNX 会话启用 `CUDAExecutionProvider`；对已有 `data/pictures/1.png`
   推理得到一个 `carotid` 框，置信度 0.784。
- 尚未在实时 Gemini 视频流上测量新模型的检测帧率、定位稳定性或连续跟踪。
   最近一次相机启动在 10 秒内未收到 RGB-D 帧，报 `NO FRAMES`；
   当前 Gemini 仍显示为 USB 2.0（480 Mbps）。
- 已加入无 Redis 模式的跟踪丢失自动重选和绿色点显示平滑；
   16 项相关离线测试通过，真实相机上仍需验证恢复成功率和点位抖动。
- 以前约 5–6 FPS 的预览速度和选点记录属于第 7 版 RF-DETR Small 模型，
   不能作为当前第 8 版 RF-DETR Medium 模型的性能数据。
- 直接预览只显示检测框和 P0 候选，不连接 Robot。

## 直接窗口启动

每次用户给出新模型 ID，只替换下面 `MODEL_ID`；类别改变时同步修改
`MODEL_CLASS`。从仓库根目录执行：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate ultralytics
MODEL_ID='songjiatong/carotid-detect-c9gi9-8-rfdetr-medium-t1'
MODEL_CLASS='carotid'
LD_LIBRARY_PATH="$CONDA_PREFIX/lib/python3.11/site-packages/torch/lib:$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
LD_PRELOAD=/home/cair-jacen/anaconda3/pkgs/mkl-2023.1.0-h213fc3f_46344/lib/libmkl_core.so.2 \
python -m infer.Carotid_begin.live_preview \
  --model-id "$MODEL_ID" --class-name "$MODEL_CLASS"
```

启动时会打印 `Roboflow ONNX execution: CUDAExecutionProvider, CPUExecutionProvider`；
若 CUDA 会话不可用则报错退出，不会静默退回 CPU。模型推理的出框置信度
固定为 0.4；宽松模式的选点框阈值仍为 0.25，
因此实际可见框的置信度至少为 0.4。默认使用宽松视觉门槛；
只有需要对照旧门槛时才追加 `--standard-vision`。
具体数值与仍保留的有效性检查见 [ARCHITECTURE.md](ARCHITECTURE.md)。

窗口显示相机原图、所有检测框、类别、置信度及 P0 候选状态；
按 `q`/Esc 退出。运行前关闭其他占用 Gemini 的程序。
逐帧诊断自动写入 `infer/Carotid_begin/reports/live_preview_*.jsonl`。

## 按 d 选点并跟踪（不连接 Robot）

先按 `q` 退出上面的预览并释放 Gemini。下面的命令可以从任意目录复制执行：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
python infer/Camera_wrist/click_follow.py --model-id songjiatong/carotid-detect-c9gi9-8-rfdetr-medium-t1 --model-class carotid --no-redis
```

复制命令时，路径里的 `_` 和 `.` 前不要加反斜杠。

窗口会持续在相机画面上叠加最新检测框（橙色，标注类别和置信度）；
框来自模型处理的最近一帧，超过 500 ms 会隐藏。首次按 `d` 从新相机帧
选 P0；随后局部跟踪器逐帧追踪，绿色圆圈和 `p(m)` 使用时间平滑后的显示值。
短暂缺帧由下一帧继续跟踪；若光流、深度或帧时效检查失败，程序约每 0.75 秒
尝试用新的模型结果重新选点。按 `p` 停止自动重选，再按 `d` 可重新开始。
检测框只是预览，不会在跟踪仍有效时直接拉动跟踪点。自动重选仅在
`--no-redis` 模式启用；连接 Robot 的模式仍需人工重新选点。
原始跟踪结果和安全检查不使用显示平滑值，Robot 控制请求不受此改动影响。
诊断日志会记录 `model_preview` 的检测完成时间和推理耗时，可用于计算实际出框帧率。
这条无 Redis 模型选点命令默认使用宽松视觉门槛，也不会发送 Robot 控制请求。
连接 Robot 的模式仍使用标准视觉门槛。

## 首次使用与故障排查

本机已保存私有 Roboflow API Key；需要检查时执行：

```bash
python -m infer.Carotid_begin.credentials check
```

仅当提示缺失时执行 `python -m infer.Carotid_begin.credentials save`。
重建 `ultralytics` 环境时，先安装 `requirements.txt`，再执行：

```bash
python -m pip uninstall -y onnxruntime
python -m pip install --no-deps onnxruntime-gpu==1.20.2
```

当前 `inference` 包的依赖声明仍要求 CPU 版包名，`pip check` 会提示它缺失，
但实际 CUDA 推理已验证。当前验证版本为 `inference==1.7.2`、
`inference-models==0.39.0`、PyTorch 2.5.1 / CUDA 12.1 / cuDNN 9。
新私有模型首次加载可能需要联网下载。若返回 404，先核对 Deploy 页的
__完整__ Model ID、工作区前缀以及当前 API Key 权限。
若相机无帧，关闭其他 Gemini 进程并重启预览；若窗口显示框但无绿色 P0，
先看类别、置信度、框数量、深度/纹理和帧龄，不把灰色或橙色框当作可用点。

只需无窗口核对模型与相机是否启动，可给主命令追加
`--headless --max-frames 10`；此模式不会显示检测框。
