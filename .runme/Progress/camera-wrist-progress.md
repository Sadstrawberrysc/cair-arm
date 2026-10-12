# Camera_wrist 项目进展（Runme notebook）

> 本文件是纳入 Git 的 Runme notebook；当前架构见
> [Camera_wrist/ARCHITECTURE.md](../../infer/Camera_wrist/ARCHITECTURE.md)。

更新日期：2026-10-02。09-17 起的记录保留当时日期、配置和验证范围；当前入口以本节为准。

## 当前入口与验证（2026-10-02）

`bash infer/Camera_wrist/start_camera.sh` 无参数默认模型模式；显式 `--manual` 使用左键
选点跟踪，不加载模型。两种模式都连接 Redis 并订阅 Robot 状态，`--no-redis` 已移除。
模型模式 `b` 新帧选点后请求开始、`d` 只选点；手动模式左键选点后按 `b/r` 请求跟随。
启动、模型加载和收到 Robot 状态本身不自动开始运动。

模型服务在 `detector/`，由相机启动 `infer.Camera_wrist.detector.worker`；
`follow_session.py` 管理会话、单键开始和暂停回执后的自动恢复。
配置见 [detector/config.py](../../infer/Camera_wrist/detector/config.py)：显式 `--model-id`
优先于 `CAROTID_MODEL_ID`，再取 `DEFAULT_MODEL_ID`；默认选点阈值 0.4。
缓存位于 `Camera_wrist/data/model_cache/`，可由 `MODEL_CACHE_DIR` 覆盖；相机 `camera`
环境与模型 `ultralytics` 环境、私有 socketpair 保留。

本次迁移 `camera` / `carotid` 环境各 79 项离线测试、`ultralytics` 模型相关 14 项通过，
含真实 socketpair 和假模型 worker；未运行真实 GPU 推理、相机、Redis 回环或机械臂运动。
当前离线入口是在仓库根目录运行
`/home/cair-jacen/anaconda3/envs/camera/bin/python -m unittest discover -s infer/Camera_wrist/tests -v`；
具体覆盖见 [迁移验证记录](#2026-10-02carotid_begin-合并到腕部模块)。

日常操作见 [腕部操作 notebook](../run/camera-wrist-operations.md)与
[模型最简启动](../run/carotid-begin-operations.md)；其他进展见
[模型进展](carotid-begin-progress.md)、[项目进展](project-progress.md)和
[早期方案历史](yolo-roboflow-visual-servo-plan.md)。

## 2026-10-12：腕部相机未枚举与启动诊断

10:49、10:51 的相机终端均在选择 `CV2L360000HZ` 时退出，尚未启动视频流。
宿主只读 Orbbec SDK 枚举为 0 台，sysfs 可见 D455 与普通 UVC 摄像头，未见 Gemini 305。
当前证据只确定腕部相机未被识别，不能区分线缆、供电、接口或相机本体原因。

`click_follow.py` 启动时先检查枚举结果：0 台设备明确提示连接检查，序列号不匹配则列出
实际设备名及序列号，保持按指定序列号选择设备。13 项 CLI/设备选择离线测试通过；
宿主只读调用新选择逻辑也返回明确的 0 台设备提示。未打开视频流、加载模型或执行运动，
连接恢复与完整 RGB-D 启动仍待现场验证。

## 2026-09-17 状态汇总（历史）

- 点击入口已接通 RGB-D → 局部追踪/法向 → Redis → Robot 坐标转换、跟随控制和 planner。
- 支持独立相机、observe、dry-run-control、显式执行及腕部无力模式。
- 全局 seed 投影和 --track 预览保留，仅写本地观测日志。
- 失效进入 Hold；重新选点并确认恢复。追踪丢失记录事件、匹配统计及局部图片。
- Gemini 数字人叠加实验已按用户反馈回退。

## 2026-09-17 已有证据汇总（历史）

| 日期 | 结果 |
| --- | --- |
| 09-11 | 标定 48 组中 47 组有效，重投影 RMS 0.569113 px；保存候选 Camera→ArmTip 外参，未独立验证 |
| 09-15 | 640×480 彩深 30 fps 实采 120 帧；用户确认全局投影对应原皮肤位置，约 24 mm 深度差待核验 |
| 09-15 | 两次追踪分别持续 62/18 帧后因仿射共识失败；已增加失效证据，根因待复测 |
| 09-16 | 历史记录：Robot 构建、7 项 CTest、28 项 Python 测试及私有 Redis 仿真通过；未发送 ServoJ |

历史文件核查见下方；测试日志：/tmp/wrist_loopback_otnd84zy（754 周期）、
/tmp/wrist_loopback_n5ichamd（757 周期），均无故障、ServoJ 发送数 0，后者验证 planner 接受过目标。

## 当前待验收事项

- 在真实目标上复核检测框中心与实际扫描点的对应关系，建立独立真值与定位误差记录。
- 验证模型选点、同源帧融合与遮挡后的暂停确认、自动重选/恢复，保留两端日志及图像证据。
- 复测静止、缓慢移动、遮挡、弱纹理、深度失效及重选恢复，分析追踪丢失图像。
- 独立核验腕部外参、同一标记多姿态 Base 位置、曝光与机器人反馈时间对应。
- 对照实测 Probe TCP 间距，核查约 24 mm 深度差及根目录记录的约 11.7 mm 间距差，分别记录来源。
- 完成悬停间距和低速真实联动验证；带力模式另需完成力残差验收。

当前实现与离线测试不能替代空间、时序或真实运动验证。以下日期条目为实现与试验历史，
旧独立视觉模式、限速、累计运动门和启动命令不能替代上方当前入口与模块架构。

## 2026-09-16 标定文件核查

| 文件 | 当时 SHA-256 / 状态 |
| --- | --- |
| gemini305_to_rm75_armtip.json | `7e34b8a775ad9ed84d0336ed18379f1e5fd6324a3d34d36db6960b68074b52e7`；independent_validation_recorded=false |
| ../Robot/build/rm75_force_calibration.json | `c16c3b0599f25b040049db9270ce25640c7817be71a873eb04d8ca86b2d99112`；tool_chain_verified=true，residuals_verified=false |

力最大残差 1.2488263462 N（门限 0.6 N），力矩最大残差 0.0509908875 N·m（门限 0.1 N·m）。
当日用户确认了纹理平板及现场准备条件；该次核查未启动机器人、未修改标定或执行运动。
几何确认不代表腕部 50 mm 间距验收；以上摘要为历史值，不代表当前文件核查。

## 2026-09-17：无力模式重新核查与真实状态读取

按用户明确要求继续完全不连接/不读取力传感器的腕部模式。修复summary v2兼容性：
力采集状态放入control.force_sensor_enabled，保持原顶层键集合；更新回归和使用文档。
7项CTest、28项Python测试通过。隔离无力仿真/tmp/wrist_loopback_q1rt98m_共752周期，
验证planner接受目标、超时/断线/乱序Hold、显式恢复；wrench始终无效、ServoJ发送0。

实机只读检查：/tmp/wrist_no_force_observe_20260917_check.csv及summary，显式
--observe --wrist-no-force、5秒自动结束。500周期robot_valid=1、wrench_valid=0，
force_sensor_enabled=false，ServoJ发送0，无故障；等待点击，不再因wrench无效拒绝。
本轮未启动相机、未做新间距测量、未进行真实运动。

剩余执行阻塞是腕部independent_validation_recorded=false和空间差值待验证；
此前日志间距81.7mm与用户约70mm测距差11.7mm未作为修正写入。目标50mm与当前估计间距
相差31.7mm，超过原5mm总运动包络，不能用当前起点直接完成接近。力标定不再是无力模式的前置条件。

## 2026-09-17：候选外参执行开关与累计运动包络取消

按用户明确授权完成--wrist-candidate-trial和--wrist-unlimited-excursion。
只在显式腕部无力候选试验中允许不设累计位移/累计转角上限，仍限时1..30秒；
不改外参验证标志，不改原速度/关节/IK/通信/Hold/急停保护。
编译与7项CTest通过，隔离仿真/tmp/wrist_loopback_4snorfg0共752周期正常结束，
测试断言合成位移超过5mm；旧帧/超时/断线仍Hold，模拟ServoJ发送0。
完整真实启动命令见 `../run/camera-wrist-operations.md`。此记录仅为实现/离线验证，不代表真实运动已通过。

### 本轮真实入口检查结果

已按用户授权启动真实--execute-wrist-follow --wrist-no-force --wrist-candidate-trial
--wrist-unlimited-excursion，30秒/3000周期正常结束，无故障。
日志/tmp/wrist_candidate_execute_20260917.csv及summary，control显示force_sensor_enabled=false、
wrist_candidate_trial=true、wrist_total_excursion_limits_enabled=false。
全程Hold，planner调用0，发送2418个保持目标；不能将这些ServoJ计为目标跟随成功。
相机日志/tmp/wrist_candidate_camera_20260917.jsonl记录点击和end，但没有有效begin；
终端曾提示select a valid target and wait for Robot before starting。
相机与Robot均已结束。下一轮需在Robot状态已到达、当前观测有效后按b，才能验证实际跟随。

## 2026-09-17：无参数常驻启动

新增start_robot.sh，预置用户授权的真实腕部无力候选模式及累计门取消选项；duration=0持续运行，
不再30秒退出。相机b/r开始确认仍保留，q只结束本轮，Robot以Ctrl+C或故障退出，不自动重启。
普通main_rm75无参数入口不变。日志使用唯一/tmp/wrist_continuous_*/runtime.csv。
编译、7项CTest和shell语法检查通过；隔离连续回归/tmp/wrist_loopback_ssez8qmj，
3832周期（超过38秒）仍工作，超时/断线/旧帧/恢复通过，SIGINT正常结束，模拟ServoJ发送0。
本次未启动常驻真实运动进程。命令：bash infer/Camera_wrist/start_robot.sh。

## 2026-09-17：持续跟随时位姿时间不匹配排查与修复

日志/tmp/wrist_continuous_1bi0Rwdh的4次停止均为wrist_pose_time_unmatched，对应反馈跨度
55.53、50.10、50.90、53.69ms；触发时光流仿射内点比例100%，653次planner调用无拒绝。
入口原固定40ms查询，反馈间隔中位数41.69ms，长记录最大75.40ms，过50ms共247次。
仅腕部点击模式查询改20ms，普通模式仍40ms；50ms插值跨度与200ms观测年龄门不变。
相机失效提示现在包含Robot的follow_reason，而不再只有笼统的restarted/held。

构建、7项CTest、28项Python测试通过。实机20秒observe无运动验证：
/tmp/wrist_poll20_observe_20260917.csv及summary，2000周期、920个不同有效状态，
反馈间隔中位数21.757ms、P95 22.239ms、P99 25.170ms、最大40.509ms，过50ms为0，ServoJ发送0。
这是较短的只读采样，不能代替ServoJ负载下的持续跟随验收；下一轮按原启动命令重启两端复测。

## 2026-09-17：按用户要求将腕部查询周期改为10ms

RobotRuntimeConfig.StatePollPeriodMs腕部模式由20ms改为10ms，普通模式仍40ms。
仅改变查询周期，50ms插值跨度、200ms观测年龄及25mm跟踪误差保护保持不变。
查询周期不是实测反馈周期保证；尚未进行本次10ms配置的实机采样或运动验证。
此前25.72mm实际位置与模型偏差故障仍需独立排查，不能认为提高查询频率已解决。

## 2026-09-21：80 mm 工作距离的相机启动配置

点击追踪和投影预览统一通过 `gemini_config.py` 在开流前加载设备的 Close-Range
预设，将视差搜索模式设为 SDK 枚举 2（256），读回预设及模式，不匹配即退出。
彩色 MJPG、深度 Y16 均明确请求 1280×800 @ 30 FPS，不自动降级。
使用所选彩色流内参和畸变参数继续对齐、反投影；配置写入 JSONL。
依据用户提供 Gemini 305 Minimum-Z 表，该组合下限为 50 mm，供约 80 mm
相机工作距离使用，不改变机器人 TCP 的 50 mm 悬停目标或追踪门限。
曝光等参数采用设备 Close-Range 预设；实际深度质量与处理延迟仍需实测。
现有 28 项 Python 离线测试及语法检查通过。设备可枚举，但打开返回
`uvc_open -6`，尚未完成实际预设读回、1280×800 双流和追踪性能验收。
关闭 OrbbecViewer/其他占用相机的进程后，原点击追踪命令即可自动应用配置。

## 2026-09-21：腕部跟随禁用超声 Tool-Y 参考重置

在 main_rm75 的 visual_y_tracking_reference_rebased 分支增加模式判断：
仅 wrist_follow_calibration 为空时允许原 Tool-Y 重置逻辑。腕部跟随不会因
Tool-Y 主导误差自动重置 model_joints、cartesian_reference_pose 或清零
previous_joint_delta；开始/恢复时由腕部控制器从实测状态建立参考的行为保留。
原因：先前腕部运行记录出现 27 次该重置，造成参考不连续的风险。
后续位置、关节和姿态跟踪误差保护保持原样，不以重置误差掩盖跟踪落后。
已完成生产入口和离线测试目标构建；真机抖动改善仍需运行日志验证，未启动机械臂。

## 2026-09-21：采集分辨率调整

按用户要求，共享采集配置改为彩色 MJPG、深度 Y16 均 640×480 @ 30 FPS；
点击追踪与投影预览共同生效。保留 Close Range Default、视差 256 和读回检查。
内参继续读取所选流。28 项现有离线测试通过；本次未启动相机或机械臂。

## 2026-09-21：腕部位置跟踪误差门限 50 mm

按用户明确要求，将腕部跟随实测 TCP 与规划模型的位置误差门限由 25 mm 改为 50 mm。
参数属于 Rm75RuntimeSafetyConfig，由运行配置按腕部模式选择，启动输出与 summary 使用有效值。
普通超声模式仍为 25 mm，其他保护和速度限制保持不变。
此变更允许更大的跟踪偏差，不代表抖动根因消除；验证采用配置回归测试、构建与离线 CTest，未启动真实运动。

## 2026-09-21：姿态目标增加 1° 更新死区

针对静止目标下法向波动引起持续姿态修正，在 WristFollowController 中以 Base
坐标的上次接受目标旋转作为比较基准。小于等于 1° 时保留该旋转，超过时更新；
开始和恢复时重置到当前有效法向构造的姿态。仍以 5°/s 平滑参考，不改变原始
法向、点、有效性、间距计算及 50 mm 偏置，因此位置侧法向噪声仍存在。
死区内 Tool-Z 可以与瞬时负法向存在最多约 1° 目标偏差（实测姿态不作此保证）。
最新相机日志 1789958645420059791：1043 有效样本，法向帧差中位 0.325°、
P95 0.717°、最大 1.154°；平面 RMS 中位 0.231 mm、最大 0.399 mm。
这不是静止相机深度噪声验收：相机随臂运动，未保存连续原始深度用于独立判定，
不能把深度 71.9～111.4 mm 的变化直接归因于传感器故障。
增加小扰动不更新、大倾斜更新及角速度边界离线测试；真实抖动改善仍待验证。

## 2026-09-21：姿态目标更新死区调整为 5°

按用户要求，将上次接受的目标朝向与新目标之间的更新死区从 1° 改为 5°。
变化不超过 5° 保持目标朝向，超过才更新；开始/恢复初始化行为与 5°/s 角速度限制不变。
瞬时法向与接受的目标 Tool-Z 可存在约 5° 偏差；此值不是实际姿态误差保证。
回归检查调整为 4° 保持、6° 更新，并继续检查角速度上限。

## 文档职责整理

架构文档只维护当前模块职责、数据流、坐标链、接口和控制行为。
变更历史、测试结果、运行问题及验收状态统一由本文件维护。
当前验证状态：候选外参未独立验收，已有真实运行抖动、ROI 越界与误差超限记录；
编译、离线测试和二维追踪通过不代表真实运动或空间精度已验收。

## 日志统一保存到 Camera_wrist/log

机器人启动脚本、点击追踪和投影入口默认日志改为模块下 log/，使用绝对路径解析，不依赖启动目录。
相机诊断图像随 JSONL 保存，SDK 日志保存于 log/sdk/。运行时自动建目录，日志加入 Git 忽略。
既有 /tmp 日志保留原位；相机 --log 自定义路径仍有效。

## 自动输出精简运行日志

start_robot.sh 在控制进程结束并关闭日志后，自动通过 export_runtime_simple.py
生成同目录 runtime_simple.csv（用户指定 22 列）。保留原始日志与控制进程退出码，
转发停止信号并等待控制进程结束再导出；启动前失败无 CSV 时跳过。
已用已有 4042 行运行日志验证导出，完成 Shell 语法检查；未启动机械臂。

## 2026-09-23：Runme 相机短时退出排查

16:31 的相机会话出帧约 2.8 秒后日志突然中止；该 JSONL 未记录主动 `q` 或连续 5 秒
无 RGB-D 的异常。同步 Robot 摘要显示 `stop_requested`，但无法仅凭这些日志判定相机
结束是 Runme 终端信号、原生库崩溃还是操作中断。16:33 后从普通 Terminator
终端启动的相机和 Robot 持续运行超过 5 分钟，且相机持续输出有效观测；这提示应先隔离
Runme 执行终端生命周期。

`camera-wrist-operations.md` 的 `start-wrist-camera` 已改为打开独立 Terminator 标签页，
由 `start_camera.sh` 在 `camera` 环境启动相机。脚本保存完整终端输出、退出码，异常时
保留标签页等待 Enter；不自动重启相机，不改变 Robot 控制或跟随安全门。

17:06:49 在无其他相机/Robot 进程时实际执行 `runme run start-wrist-camera`：Runme 返回 0，
相机在 Terminator 独立标签页启动。17:09:19 相机仍运行约 2 分 30 秒，JSONL 记录 4463 条
跟踪事件、149.2 秒连续出帧，无 Redis 错误；未选点时的 `not initialized` 属正常等待状态。
终端仍出现 `QObject::moveToThread` 与缺少 OpenCV Qt 字体目录的警告，但这次没有导致退出。
本次只验证相机启动/持续采集，未启动 Robot 或执行跟随运动。

## 2026-09-23：Robot offline 与首次无画面退出

17:06 的 Runme 相机验证只启动了相机，没有启动 RM75，因此窗口显示 Robot offline
是预期行为。后续会话曾记录每秒多次 `state_rejected`，但旧代码把所有字段失败
统一写成 `invalid/stale Robot state or calibration mismatch`，不能据此判断具体字段。
在 Robot 持续运行时，被动订阅的 401 条状态延迟均小于 34 ms，boot ID、标定 SHA、
坐标系、有效位全部匹配；相机实际 `ClickSession` 对实时状态也接受成功，
年龄约 1.5 ms、状态为 hold。新启动的相机日志无状态拒绝，说明当前连接正常。
`click_follow.py` 已改为将不匹配字段和状态年龄分别显示/记录，便于以后定位间歇性拒绝。

17:14:31 和 17:18:49 两次首次无画面退出的终端日志明确写 `BLOCKED: no complete
RGB-D for 5 seconds`，JSONL 对应 `NO FRAMES`。旧计时从 SDK `pipeline.start` 之前
开始，包含设备初始化耗时。现在从流启动完成后计时，首次完整帧宽限 15 秒；
出过帧后继续使用 5 秒断流门。本轮新启动已从短暂 NO FRAMES 恢复到连续出帧。
相机与 Robot 仍在现场运行，未按 b 发送跟随开始请求。

## 2026-09-23：Robot offline 的终端生命周期修复

17:24 的相机和 Robot 同时运行时，Redis 真实状态年龄均值约 8 ms、最大 33 ms，
boot ID/标定 SHA/坐标系/有效位匹配；用相机的 `ClickSession` 校验一条实时状态通过，
年龄 1.54 ms，follow_state=hold。相机日志连续运行约 4 分钟无状态拒绝。
17:27:58 Robot 从 VS Code Runme 管理终端启动后突然消失；CSV 最后仍显示
`robot_valid=1` 和 Redis 发布连接，未生成正常退出 summary，内核日志无段错误/OOM。
终端被关闭/挂起导致 SIGHUP 是最符合证据的解释，但旧日志不能确定具体信号。
Robot 不在运行时相机界面显示 offline 属于预期。

修复：`start-wrist-robot` 和 `start-wrist-camera` 均以 `nohup setsid -f` 启动独立
`terminator --no-dbus` 窗口，避免进程依附 Runme 执行终端；机器人新增
`start_robot_terminal.sh` 记录终端输出/退出码，`start_robot.sh` 转发 HUP，
`main_rm75` 捕获 HUP 并沿原有 StopMotion/静止确认路径结束。`main_rm75` 构建通过。
在真实 RM75 连接、**只读 observe** 模式下注入 HUP，进程退出码 0、生成 summary，
`completion_reason=stop_requested`，没有执行运动；尚未对真实 execute 模式注入 HUP。

## 2026-09-23：首次零帧与 USB 链路进一步核查

独立窗口实测 17:35:14 启动时，SDK 报流配置匹配，但完整 RGB-D 始终为 `NO FRAMES`，
在延长的 15 秒首次宽限后仍以退出码 2 结束。因此单纯延长等待不能解决此故障。
`lsusb -t` 与 sysfs 显示 Gemini 305 接在多级 Hub 下的 USB 2.0／480 Mbps 链路；
Orbbec 官方推荐 USB 3.0，故该链路是间歇性零帧的可能因素，但当前日志不能证明
它是唯一原因。现修改为首次流启动后 10 秒无完整 RGB-D 时返回专用退出码 7，
终端脚本完整退出旧 SDK 进程、等待 1 秒，只重新初始化一次；若第二次仍失败，
保留错误和退出码，不循环重试、不降低 640×480 30 fps。已经取得完整帧后仍用
5 秒断流门，不能自动重启正在跟随的相机。首次启动失败时不会再发送未曾开始的
`end` 请求，避免将 Robot 无故置为 `operator_ended`。
17:38 的 Runme 相机正常启动并持续出帧，因此只验证了正常路径，未触发重试路径。
相机新增一次性 `robot_state_connected` 事件，后续可从日志区分状态已接受与单纯
没有拒绝记录。

17:46、17:47 和 17:53 的后续相机会话均记录 `robot_state_connected`，说明 Robot
运行时相机能接受其实时状态；17:53 的会话还跨 Robot 重启记录了第二次连接。
17:48 和 17:53 各出现一次带明确年龄的旧状态拒绝（约 1130 ms、268 ms），
不是持续的标定不匹配；后续状态会恢复。17:38 的独立终端相机会话持续到 17:42
仍有跟踪日志，未复现立即闪退。17:55 检查时相机和 Robot 均未运行；如单独启动相机，
界面仍会显示 offline，需先通过 `start-wrist-robot` 启动 Robot。

## 2026-09-23：腕部启动命令简化

`start-wrist-robot` 与 `start-wrist-camera` 的 Runme 单元各收敛为一行，
共同调用 `launch_wrist_terminal.sh robot|camera`；独立 Terminator、日志和防重逻辑
移入脚本。防重匹配同时覆盖相机实际使用的 `python -u` 启动形式。
已通过 Bash 语法检查及两个 Runme 单元的 dry-run；本次未重新启动实体设备。

## 2026-10-02：Carotid_begin 合并到腕部模块

模型服务归入 `Camera_wrist/detector/`，采用 config、credentials、gpu_model、detection、
pipeline、protocol、worker 的职责拆分；模型入口改为
`python -m infer.Camera_wrist.detector.worker`。相机与模型仍使用各自 Conda 环境和私有
socketpair，不改变 Robot Redis 公共协议与控制。ClickSession、ModelStart 和 ModelRecovery
统一放在 `follow_session.py`，相机入口保留采集、窗口、跟踪融合和 Redis I/O。

无参数默认模型模式；默认 ID 由 `detector/config.py` 定义，`CAROTID_MODEL_ID` 可覆盖，
显式 `--model-id` 优先。`--manual` 保留手动点击、光流和深度跟踪，且不启动模型进程。
模型加载不自动开始运动，仍由人工 `b/r` 启用跟随意图。

默认缓存迁到 `Camera_wrist/data/model_cache/`，原私有凭据路径继续兼容；原独立模型日志
归档到 `Camera_wrist/log/carotid_begin/`。模型测试合并到 `Camera_wrist/tests/`；
架构与操作入口同步更新。模型历史证据保留在
[模型进展 notebook](carotid-begin-progress.md)，当前服务边界见
[detector 架构](../../infer/Camera_wrist/detector/ARCHITECTURE.md)。

本次 `camera`（Python 3.8）与 `carotid` 环境各运行 79 项离线测试全部通过，
包含本机真实 socketpair 和假模型 worker 子进程；`ultralytics`（Python 3.11）环境的
模型相关 14 项全部通过。Python 源码语法、Bash 语法、相机和新 worker 的 `--help`
检查通过，提取的三个会话类与迁移前 AST 一致。
这些验证使用假模型及离线输入，未运行真实 GPU 推理、相机、Redis 回环或机械臂运动；
真实目标连续跟踪、恢复和定位精度仍待验收。

## 2026-10-09：恢复原 RT 内核默认启动项

17:46 相机模型 worker 启动失败时，当前内核为 `6.8.0-45-generic`，该内核没有
NVIDIA 驱动模块，`/dev/nvidia*` 不存在，`nvidia-smi` 无法连接驱动。已核对旧驱动
`535.129.03` 的模块仅安装在 `6.2.1-rt3`，与本机 NVIDIA 用户态库版本一致；
原 RT 内核的 vmlinuz、initrd 和 GRUB 菜单均存在，默认模型权重缓存也存在。

原 `GRUB_DEFAULT="1>4"` 的数字索引因新增内核而指向 generic 内核。按用户选择，
已将 `/etc/default/grub` 的默认项改为原 RT 内核的固定菜单 ID，并执行 `update-grub`；
生成菜单包含正确默认项，`grub-script-check` 通过。原配置和菜单备份保存在
`/var/backups/uspilot-grub-20261009_175038/`。

本次未安装或升级驱动、未修改模型代码、未重启电脑。重启后需先确认 `uname -r`
为 `6.2.1-rt3` 且 `nvidia-smi` 正常，再复测模型加载和相机；GPU 推理与真机运行尚未验证。

## 2026-10-09：改用指定 generic 内核并恢复 NVIDIA 驱动

用户随后明确要求使用 `6.8.0-45-generic`，该要求替代上一条记录中的 RT 默认启动选择。
本次核对 `/etc/default/grub` 和 `/boot/grub/grub.cfg` 均已固定为该内核的菜单 ID：
`gnulinux-advanced-4dd208b4-ab9b-42ca-9eae-bafb8df79db7>gnulinux-6.8.0-45-generic-advanced-4dd208b4-ab9b-42ca-9eae-bafb8df79db7`。
当前运行内核也是 `6.8.0-45-generic`，GRUB 语法检查通过。

使用原 `nvidia-uninstall` 清理手动安装的 `535.129.03`，随后安装 Ubuntu 官方仓库中
`ubuntu-drivers devices` 推荐的 `nvidia-driver-595`（`595.99.02`）及 DKMS。
安装新增 19 个包，未升级现有包、未删除其他内核；DKMS 为当前 `6.8.0-45-generic`
和已安装的 `6.8.0-138-generic` 构建并安装模块。另已更新指定内核的 initramfs。
旧 535 驱动已卸载，旧 RT 内核不再作为本次 GPU 验证目标。
卸载日志为 `/var/log/uspilot-nvidia-uninstall-20261009.log`；
卸载器提示无法删除非空的 `vdpau` 目录，退出码为 0，新驱动安装与验证成功。

无需重启即成功加载 `nvidia`、`nvidia_uvm`：`nvidia-smi` 显示驱动 `595.99.02`，
GPU 为 RTX 4080 Laptop；`dkms status` 确认当前内核模块已安装，`dpkg --audit` 无输出。
通过项目原有 `camera` 环境 → `ModelSelector` → `ultralytics` 环境 worker 通道，
对本地保存的 `infer/Camera_wrist/data/pictures/1.png` 连续运行 3 次真实模型推理：
ONNX 会话启用 `CUDAExecutionProvider`，worker 正常就绪，源时间戳一致，均返回有效选点。
检测到一个 `carotid` 框，置信度约 `0.7974`；首次推理约 `260 ms`，随后约 `10.4/10.7 ms`。
这些耗时仅代表该保存图片的模型推理，不代表真实相机全链路延迟或定位精度。

本次未修改模型源码、未升级 Python 推理包、未重启电脑；尚未复测真实相机、深度拟合、
追踪或机械臂。SAM/SAM3/YoloWorld 等可选依赖及版本提示仍存在，但不阻碍当前模型推理。

## 2026-10-12：速度翻倍与自动初始开始

按用户要求，腕部 TCP 参考速度由 2 mm/s 改为 4 mm/s，姿态参考和每关节 ServoJ
命令上限由 2°/s 改为 4°/s。`main_rm75` 已重新编译，常用启动入口直接使用新二进制。
默认模型相机新增 `ModelStartup`，取得新鲜 Robot Hold 和完整 RGB-D 后自动从新帧
选取初始目标，不需要按 `b`；继续经过追踪/深度/表面拟合及新目标回传确认后请求跟随。
无唯一合格目标或追踪无效时每隔至少 0.1 秒重试，未确认目标时不发开始请求。
人工 `p` 取消自动开始和恢复；之后可用 `b` 重选并跟随。手动模式保持原操作。

新增初始启动离线用例覆盖等待 Robot、选点与目标回传顺序、无效选点重试、人工取消、
故障/重启取消及状态/忙碌门；速度测试确认远目标每周期以新限速推进。
`camera` 环境 90 项腕部离线测试通过（私有 socketpair 需在允许本机 socket 的执行环境运行）；
CMake 配置、`main_rm75` 和 `robot_offline_tests` 构建成功，CTest 6/6 通过。
启动 notebook、架构和终端提示已同步；未启动真实相机或机械臂，实机速度和自动开始待验收。
