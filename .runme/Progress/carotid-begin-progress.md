# 腕部 detector 模型进展及 Carotid_begin 历史（Runme notebook）

## 2026-11-10：超声窗口恢复原显示

按用户要求移除 Tool-Y 修复新增的中心参考线、目标圆圈及新状态 / 误差来源文字，
恢复原 `y:`、`On Track`、箭头和字体；其他超声叠加显示保持原样。
后台图像居中、时效门和配对录像继续保留，不新增 Tool-Y CSV。
源码语法及显示区与原版的差异检查通过；未启动真实图像窗口或机械臂。
下方新增标记的记录为本次恢复前的历史。


## 2026-11-10：Tool-Y 居中改用图像闭环并抑制抖动

检查 17:37:30 Robot CSV：436 次视觉误差更新内换向 69 次，其中 44 次两侧均小于 1 mm，
旧 0.01 mm 死区、单帧方向确认会让噪声驱动来回修正；跟踪暂停锁存未触发。
17:37:35 超声录像第 30 秒离线 YOLO / 分割回放：血管中心约 x=426 px，ROI 中心 x=278 px，
按当前 37 mm 成像高度为约 8.5 mm 偏差，而同步旧模型输入仅约 -0.6 mm。
此外 YOLO 内部仍写死深度 200～350 px，与主流程 20～500 px 不一致。

生产入口改为由当前匹配的分割 / YOLO 中心计算横向误差，pose head 仅终端诊断；
0.5 mm 死区、0.8 mm 重启阈值、两帧确认、alpha=0.35，正常 Y 未缩放速度上限 4 mm/s。
当前未限幅图像误差 ≤0.8 mm 才放行 X/RZ / 自动结束；无目标或跳变时保留 Y/Z 中的 Z 力控，
原 recovery 搜索、低力 / 卸载 / planner 门保留。图像超过 300 ms 时心跳不再持续重发旧运动。
界面显示目标标记、中心线和等待原因；按用户要求不新增 tool_y_alignment.csv。
11 项视觉居中测试和原录像测试通过，Robot motion_logic_test 通过；生产 v1 协议自检通过。
验证覆盖带延迟的左右闭环收敛、中心噪声、换向、失效 / 多目标 / 跳变、YOLO 范围及真实心跳循环。
本轮仅离线录像回放、合成闭环和离线 C++ 测试，未运行真实机械臂；物理方向与实际改善仍需现场验证。


## 2026-11-10：配对录像显示完整录制历史

按用户补充要求，力曲线不再限制最近 10 秒，改为 0 秒到当前时刻，最终帧显示整个录制区间。
超声视频和两类 CSV 原本即持续保存到退出。曲线用最多 512 个自适应时间箱保留 min/max、
早期极值与缺口标记，完整数值继续逐条写 CSV。起点为模型加载及 idle 握手后进入超声录像时刻。
7 项录像离线测试通过，含超过 10 秒仍显示初期数据、模拟一小时压缩后极值/缺口保留及 AVI 解码。
本轮未启动真实采集或机械臂。


## 2026-11-10：超声与 tare 后六维力配对录像

超声入口在模型加载及 idle 握手后自动保存 `output/scan_<时间>_<id>/`：
`ultrasound.avi`、`wrench_tare.avi`、六维 CSV、帧时间戳 CSV 和录制元数据。
后台线程以同一 30 FPS 时间轴写两段 AVI，力曲线显示从录制开始到当前时刻的全部历史；推理交付图像较慢时
重复上一帧并显示接收帧龄。只读取现有超声 ROI 和 Robot Redis 遥测，不另开采集卡或发送控制命令。
Robot 新增 `runtime_tare_applied` 标记，录像只将已 tare 且有效、新鲜的 Tool wrench 绘为曲线；
未 tare / 失效 / 过期保留缺口，不补零。Ctrl+C、异常与正常退出均收尾视频文件。
已编译 `main_rm75` / `redis_bridge_test`；Redis 协议测试、5 项录像测试、8 项串联流程测试通过。
录像测试实际编码并逐帧解码两段合成 AVI，验证帧数匹配、六维 CSV、断流标记和编码失败上报。
未启动真实相机或机械臂；实际采集吞吐和设备时间关系仍需现场验证。


> 本文件记录腕部 detector 模型服务及原 Carotid_begin 的进展、验证证据与运行边界，纳入 Git。
> 当前架构见 [detector/ARCHITECTURE.md](../../infer/Camera_wrist/detector/ARCHITECTURE.md)，
> 最简启动命令见 [启动 notebook](../run/camera-wrist-operations.md)。
>
> Notebook 整理及模块合并日期：2026-10-02。下方历史验证结果保留原记录的日期与执行范围，迁移未新增实测。
> 历史验证命令标记为 `ignore=true`，仅供查阅；日常启动在 run notebook 中逐个执行。

## 当前组织与待验收

原 `Carotid_begin` 已并入 `Camera_wrist/detector/`。默认模型 ID 取 `detector/config.py`，显式 CLI 优先于环境变量；
相机默认模型模式，`--manual` 保留手动跟踪，始终连接 Redis。2026-10-12 起模型在 Robot 就绪后自动选点并请求跟随，
TCP / 姿态 / 每关节命令限速为 4 mm/s、4°/s、4°/s；下方 10-02 的人工首次开始和限速值保留历史语义。

模型进程使用 ultralytics，SDK 使用 camera，经私有 socketpair 交互；Robot 负责坐标与运动。
当前仍需真实目标连续融合 / 恢复、独立扫描点真值、外参 / TCP / 时序与净空验收；接触和超声力控交接未实现。
当前验证与启动见 [腕部进展](camera-wrist-progress.md)和 [腕部操作](../run/camera-wrist-operations.md)。

## 合并前证据汇总（2026-10-02）

- 模型：`songjiatong/carotid-detect-c9gi9-9-rfdetr-small-t1`，类别 `carotid`。
   本机 `ultralytics` 环境的 ONNX Runtime 已启用 CUDA provider；API Key
   保存在本机私有凭据文件中。
- 已将相机启动脚本默认值和启动 notebook 中的入口更新为第 9 版。本机第 9 版已加载，
   ONNX Runtime 使用 `CUDAExecutionProvider`；对已保存的 `1.png` 推理得到
   1 个有效 `carotid` 框。尚未验证实时相机画面、连续跟踪或真机运动。
   以下历史模型检测数值均来自第 8 版
   `songjiatong/carotid-detect-c9gi9-8-rfdetr-medium-t1`，不能作为第 9 版结果。
- 当时实现了检测框中心选点、光流逐帧跟踪、源帧对齐后的模型校正、无 Redis 模式自动
   恢复，以及 Redis 模式视觉丢失后的自动重选/恢复。无 Redis 分支后来已移除；当时腕部相机 47 项、模型模块
   5 项离线测试通过；Robot 6 项 CTest 通过。
- 临时端口 Redis + 模拟 Robot 的无力候选试验通过：状态、观测、开始、
   暂停、恢复和超时 Hold 均有记录；新增连续两次视觉丢失后自动恢复及人工
   暂停保持测试通过，模拟 ServoJ 发送数为 0。自动恢复尚待真实相机和运动验证。
- `camera` 环境经私有 socket 调用 `ultralytics` GPU 模型，用已保存的 `1.png`
   得到 1 个有效框，源帧标识一致。
- 旧 session `20260928_142615_003bdcb2` 的 10 张真实 RGB-D 上，第 8 版模型
   有 8 张得到唯一有效框；这 8 张用__标准视觉门槛__均成功从框中心初始化
   三维点和法向。样本间最短间隔约 634 ms，不能用来验证连续光流或 Redis
   实时跟踪，也没有目标点真值可据此评价定位误差。
- 对一张存档真实 RGB-D 图像施加已知的 0–6 px 水平平移后，光流对 6 px
   位移的误差约 0.001 px；模型框中心相对已知位移的误差依次为 0、0.5、
   0.7、23.1、27.6、3.8、4.6 px。该合成平移暴露连续两帧模型跳框，
   **不是**真实人体运动测试。当前版本不再因模型中心与光流点相差超过
   15 px 而 Hold；融合点每次最多校正 2 px。现场仍需检查模型跳框后实际点位
   是否正确；这项变化不影响 Robot 的其他检查。
- 已保存 RGB-D 样本 `93e229f08d3347c6a2960ad6ae503498` 的 1 px
   合成平移单步联调通过：第 8 版 GPU 模型经私有 socket 返回中心
   `(372.0,389.5)`，光流点为 `(375.499,390.000)`，源帧对齐后的融合点为
   `(374.625,389.875)`，`applied=true`、`confirmed=true`。这验证了模型进程
   到融合函数的调用链；未经过 Redis 或真实 Robot，也不代表动态定位精度。
- 在隔离的本地 Redis 端口，用真实 RM75 状态反馈与合成的近 TCP 观测
   完成 `--dry-run-control --wrist-no-force --wrist-candidate-trial` 回环：
   303 条腕部记录中出现有效规划、`following` 与操作员 `Hold`，Robot
   正常退出，力传感器未启用，累计运动门槛仍启用，ServoJ 发送数为 0。
   该试验只验证真实硬件反馈、Redis 协议和规划链路；观测点不是相机模型输出，
   也没有执行机械臂运动。
- 真实 Gemini 检测窗口与真实 RM75 **observe 只读模式**同时运行成功：
   相机记录 271 条 RGB-D 跟踪采样、57 次模型预览，并接收到 Robot 的有效
   Redis 状态；Robot 退出正常，ServoJ 发送数为 0。这段模型预览约 7.6 Hz，
   推理耗时中位数 13 ms；这是无目标画面的短时测量，不能代表有目标跟踪率。
   当前人不在画面中，57 次预览无合格唯一目标，尚未验证真实目标选点、
   融合点稳定性或真实机械臂运动。

真实运行 `wrist_continuous_wLnrVdAX` 曾因规划 TCP 离起点超过 5 mm 停止；
取消位移门槛后的 `wrist_continuous_BnZfbLqc` 因累计转角超过 5° 停止。
当前 `start_robot.sh` 已配置关闭累计位移、累计转角和腕部 50 mm 位置跟踪
误差门槛；Robot 已重新编译；重启当前 Robot 进程后生效。运行摘要的
`wrist_translation_limit_enabled`、`wrist_orientation_limit_enabled` 和
`wrist_position_tracking_error_limit_enabled` 可核对实际启用状态。6 项 Robot
离线测试通过；隔离 Redis 模拟链路完成两次自动恢复，摘要显示三项检查关闭、
ServoJ 发送数为 0。新配置尚未进行真机验收。

## 2026-09-30 模块化验证（历史）

模型服务已拆成配置（`config.py`）、单帧业务（`pipeline.py`）、协议
（`live_protocol.py`）和启动/常驻循环（`model_worker.py`）。原入口、CLI 默认值、
响应字段、框中心规则和 CUDA provider 要求保留；现有相机模块删除状态保留。

在 `camera` 环境运行以下命令，13 项离线测试通过，覆盖凭据、检测、协议、
配置拒绝、同一 socket 多帧处理、无目标后继续处理、源帧标识、异常模型输出和断连：

```bash {"name":"carotid-recorded-module-tests","ignore":"true"}
/home/cair-jacen/anaconda3/envs/camera/bin/python -m unittest discover \
  -s infer/Carotid_begin/tests -v
```

本次未运行 GPU 推理、真实相机、Redis 或 Robot。调用方
`Camera_wrist/tests/test_model_selection.py` 的重构前基线为 9 项中 6 项通过、
3 项错误：测试仍要求 `robot_mode` 参数和融合层的跳框拒绝，但当时工作区
`model_selection.py` 未提供对应接口/行为。这些既有不一致未纳入本次修改，
不能将上文历史通过记录视为当前工作区的完整回归结果。

## 启动方式更新（2026-10-02）

相机每次启动都连接 Redis 并订阅 Robot 状态，已取消 `--no-redis` 和仅用于
独立视觉的 `--standard-vision` 参数。统一使用标准跟踪门槛，默认选点置信度
当时为 0.5（当前为 0.4）；启动顺序见 [最简启动 notebook](../run/camera-wrist-operations.md)：Redis → 相机 → Robot。模型模式首次按 `b` 自动选点并请求跟随。
已运行的旧相机进程需按 `q` 退出后重新启动，修改才会生效。

本次相机离线测试 48 项全部通过，Python 编译及差异检查通过；未重启现场
相机或 Robot，未执行真机运动验证。上文模块化验证时记录的 3 项调用方测试
错误在本次工作区回归中已不再出现。

## 跟随参考限速减半（2026-10-02）

按要求将腕部 TCP 平移参考限速从 5 mm/s 降为 2.5 mm/s，姿态参考限速
从 5°/s 降为 2.5°/s。`main_rm75` 已重新编译，6 项 Robot 离线测试全部通过。
本次未重启 Robot 或执行真机运动验证；正在运行的旧进程仍使用原限速，
需在 Robot 终端用 `Ctrl+C` 正常停止后，重新运行 `start_robot.sh` 才生效。

## 跟随参考限速降至 1（2026-10-02）

按要求将腕部 TCP 位置参考限速从 2.5 mm/s 降至 1 mm/s，将探头姿态参考
角速度从 2.5°/s 降至 1°/s。进一步按“关节角速度”要求，为腕部跟随的
ServoJ 规划增加每个关节 **1°/s** 的命令步长上限，即 10 ms 周期内每关节
最多 0.01°。普通超声模式的关节规划速度配置保持原值；实际关节速度还需
真机日志测量。`main_rm75` 已重新编译，6 项 Robot 离线测试通过。若旧进程
仍在运行，需在 Robot 终端用 `Ctrl+C` 正常停止，再运行 `start_robot.sh`。
隔离 Redis 模拟跟随及两次自动恢复通过：1346 条记录中的最大相邻关节命令
步长约 0.0063°，低于 10 ms 周期的 0.01° 上限；模拟未发送真实 ServoJ。

## 腕部参考与关节命令限速提升至 2（2026-10-02）

按要求将腕部 TCP 位置参考由 1 提升至 2 mm/s，探头姿态参考由 1 提升至
2°/s，同时将每关节 ServoJ 命令上限由 1 提升至 2°/s。10 ms 规划周期内
每关节命令最多变化 0.02°。普通超声模式的关节速度配置不变；新数值需要
重新启动 Robot 进程才会生效，真机实际速度尚待测量。`main_rm75` 重新编译，
6 项 Robot 离线测试通过；隔离 Redis 模拟跟随及两次自动恢复通过，最大相邻
关节命令步长约 0.0125°，低于每 10 ms 的 0.02° 上限，真实 ServoJ 发送数为 0。

## Robot 时间匹配暂停自动恢复（2026-10-02）

已将 Robot 主动触发的 `wrist_pose_time_unmatched` 接入模型自动恢复。
首次人工 `b`/`r` 后，若同一 Robot 会话针对本相机进程当前目标返回此 Hold，
相机会发送自己的暂停请求，等待暂停回执，再按约 0.75 秒重试间隔用新帧重选。
新目标有效且 Robot 状态确认新 target_id 后，自动发送 `resume`。
若恢复请求仍被时间匹配检查拒绝，会再次走上述流程；重复状态不重复发送暂停。

仅这一主动 Hold 原因新增自动入口。人工暂停/结束、故障、重启、状态超时、
其他进程的回执、其他目标或规划拒绝不会通过此入口自动启动；Robot 原有
时间插值和时效检查保留。持续时间匹配失败时仍保持 Hold，不保证恢复成功。

52 项相机离线测试通过，覆盖连续两次主动 Hold 后恢复及身份、回执、时效
与取消条件；Python 编译检查通过。本次未进行 Redis 回环或真机运动验证。
需退出并重启相机进程才能使用新逻辑；模型模式首次跟随按 `b` 自动选点并开始。

## 模型模式单键开始（2026-10-02）

按 `b` 自动在新帧检测、初始化并追踪新目标；观测先发布，等待 Robot 状态
回传该 target_id 后，首次发送 `begin`，已有跟随历史则发送 `resume`。
按键时需要新鲜的 Robot Hold 状态。无有效框、追帧失败或新点过期不会启动；
失败后需再次按 `b`。通信错误、状态超时、Robot 重启、故障及 `p/q` 取消待启动
请求，重复按 `b` 不重复排队。`d` 仍可仅选点，非模型鼠标模式保持点击后按 `b`。

57 项相机离线测试通过；未执行真机验证。需重启相机进程后使用新按键流程。

## 历史启动约定（2026-10-02）

Redis → 相机 → Robot；Redis 单独存在不会消除 `waiting for Robot state`。
当时首次按 b 选点并请求开始，d 只选点，视觉丢失后暂停确认 / 重选 / resume；p 取消、q 退出相机，Robot 用 Ctrl+C 停止。
这段人工首次开始约定已由 2026-10-12 模型自动初始开始替代；当前命令见[启动手册](../run/camera-wrist-operations.md)。

## 运行配置与诊断记录（2026-10-02）

Robot 使用 `--wrist-no-force --wrist-candidate-trial --wrist-unlimited-excursion`
和 `--wrist-unlimited-position-tracking-error --duration-sec 0`，
持续运行直至 `Ctrl+C` 或故障；Hold 只暂停运动，**不读取力传感器**，
已取消实际与规划 TCP 的累计位移、累计转角检查，也取消实测 TCP 与规划
模型之间的 50 mm 位置误差检查；关节、规划、通信和停止检查仍在。Robot 对
融合观测生成的参考轨迹另有限速：TCP 位置 2 mm/s、姿态 2°/s；
腕部 ServoJ 目标每关节命令步长上限为 2°/s。
计算目标为估计表面外法向 50 mm；物理净空必须在现场确认。新设置允许 TCP
离起始位置超过 5 mm，也允许超过 5° 的累计转角；不会自动扩大关节范围或改变跟随参考限速。

相机诊断在 `infer/Camera_wrist/log/wrist_click_*.jsonl`，其中 `model_preview`
记录出框时间，`model_fusion` 记录校正或拒绝原因，`model_recovery_wait` 和
`model_auto_resume` 记录自动恢复转换。Robot 的 `runtime.csv.wrist.csv`
及 `runtime.summary.json` 在 `infer/Camera_wrist/log/wrist_continuous_*/`。
可在无真实运动条件下复测模拟 Redis 链路：

```bash {"name":"carotid-recorded-loopback","ignore":"true"}
/home/cair-jacen/anaconda3/envs/camera/bin/python \
  infer/Camera_wrist/tests/loopback_check.py --no-force --candidate-trial --unlimited-excursion \
  --unlimited-position-tracking-error --auto-recovery
```

## 更换模型

以后拿到新 Roboflow Model ID，可通过 `CAROTID_MODEL_ID` 覆盖
[detector/config.py](../../infer/Camera_wrist/detector/config.py) 的默认值，或在相机入口显式传入
`--model-id`；类别改变时传入 `--model-class`。如使用终端启动脚本，可先导出
`CAROTID_MODEL_ID='新的完整 Model ID'`，再运行
`bash infer/Camera_wrist/launch_wrist_terminal.sh camera`。
不要把 API Key 写入命令；本机凭据检查命令为
`/home/cair-jacen/anaconda3/envs/ultralytics/bin/python -m infer.Camera_wrist.detector.credentials check`。
