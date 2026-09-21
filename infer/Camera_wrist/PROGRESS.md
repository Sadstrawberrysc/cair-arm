# Camera_wrist 进度

整理日期：2026-09-17。以下为已有实现和历史验证汇总，本次文档重写未新增实测。

## 当前状态

- 点击入口已接通 RGB-D → 局部追踪/法向 → Redis → Robot 坐标转换、跟随控制和 planner。
- 支持独立相机、observe、dry-run-control、显式执行及腕部无力模式。
- 全局 seed 投影和 --track 预览保留，仅写本地观测日志。
- 失效进入 Hold；重新选点并确认恢复。追踪丢失记录事件、匹配统计及局部图片。
- Gemini 数字人叠加实验已按用户反馈回退。

## 已有证据

| 日期 | 结果 |
| --- | --- |
| 09-11 | 标定 48 组中 47 组有效，重投影 RMS 0.569113 px；保存候选 Camera→ArmTip 外参，未独立验证 |
| 09-15 | 640×480 彩深 30 fps 实采 120 帧；用户确认全局投影对应原皮肤位置，约 24 mm 深度差待核验 |
| 09-15 | 两次追踪分别持续 62/18 帧后因仿射共识失败；已增加失效证据，根因待复测 |
| 09-16 | 历史记录：Robot 构建、7 项 CTest、28 项 Python 测试及私有 Redis 仿真通过；未发送 ServoJ |

历史文件核查见下方；测试日志：/tmp/wrist_loopback_otnd84zy（754 周期）、
/tmp/wrist_loopback_n5ichamd（757 周期），均无故障、ServoJ 发送数 0，后者验证 planner 接受过目标。

## 待办

- 复测静止、缓慢移动、遮挡、弱纹理、深度失效及重选恢复，分析追踪丢失图像。
- 独立核验腕部外参、同一标记多姿态 Base 位置、曝光与机器人反馈时间对应。
- 对照实测 Probe TCP 间距，核查约 24 mm 深度差及根目录记录的约 11.7 mm 间距差，分别记录来源。
- 完成悬停间距和低速真实联动验证；带力模式另需完成力残差验收。

当前实现与离线测试不能替代空间、时序或真实运动验证。启动见
[点击跟随](USAGE.md#点击跟随) 与 [投影预览](USAGE.md#全局投影与追踪预览)。

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
完整真实启动命令见USAGE.md最后一节。此记录仅为实现/离线验证，不代表真实运动已通过。

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
