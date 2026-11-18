# 全局粗定位的关节路径执行

默认入口只读。显式 `--execute` 启用 Robot C++ MoveJ 关节路径执行，
只接受本进程生成的完整 OMPL 路径，不读取任意旧路径文件执行，不使用 MoveJ_P 或重新求 IK。
逐个执行 OMPL 原始路点，`r=0`、`trajectory_connect=0`，不插入 2° 的运动指令点。
运动中的相机场景监督已按用户要求取消：不进行新点云路径复查或续期，新增／移动障碍物不会触发相机避障停止。
点云仍显示，模型、标定、控制器插补路径及实际制动行为仍需现场验证。

## 从采集到执行

在仓库根目录运行。C++ 独占 D455，相机图像和对齐深度共享给现有颈动脉识别程序，
同时显示颈动脉图像与点云窗口：

颈动脉图像使用 `color stream`（原始相机画面加采样点，首点用红圈突出），不显示骨骼／血管模型叠加窗口 `result`。

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
python infer/hand_eye_calibration-main/run_probe_target.py --collision
```

等待识别后，点云中的红点是颈动脉首点在当前保留环境点云中的欧氏最近点。
点云窗口按 P 固定该红点，沿原识别外法向后退 100 mm 生成 TCP 目标并规划；按 E 执行。
到位验收（关节、FK TCP 和静止确认）全部通过后，C++ 自动返回 0 并关闭点云窗口，
启动器停止并等待识别进程退出、关闭颈动脉窗口，完成资源释放后才返回上层流程。
一键流程随后自动启动腕部相机／Robot；单独运行粗定位命令只自动关闭窗口，不自行启动腕部。
未完成验收、停止未确认、人工信号中止或提前退出均不能作为成功交接；失败时 GUI 可保留供查看原因。
不需要先按 s 保存 TXT；图像窗口 s 仍可保存，q 结束双窗口会话。
规划失败不发送 MoveJ，不回退 MoveJ_P。加 `--dry-run` 只显示／规划，不启用运动。
原 `--velocity` 只作用于旧 MoveJ_P 分支。
Collision 模式暂不支持 `--wrist-handoff` 投影 seed，避免把名义姿态当作选中的倾斜姿态。

直接启动双窗口及可选执行：

```bash
./infer/Collision/start_monitor.sh --carotid --execute
```

去掉 --execute 只读预览。--carotid 模式由 C++ 发布共享帧，Python 只消费图像，不再打开相机。
识别结果带会话、相机／标定身份、源帧号和采集时间；只接受 2 s 内结果，在当前过滤后环境点云重新匹配。
两窗口异步更新，不声称硬同步；窗口显示匹配距离和源帧年龄。机器人、遮挡、稀疏点及 ROI 外点不作为匹配候选。
无有效识别或保留点时清除实时红点，按 P 报 target_not_available，绝不回退旧 TXT。
规划使用冻结快照，搜索期间不更新相机帧；目标预览中的红点保留本次规划使用的位置。
最近点匹配不证明识别的是正确解剖点，需核对两个窗口的对应关系。

仅使用已有 TXT 的旧预览／执行入口仍可用：

```bash
./infer/Collision/start_monitor.sh --execute
```

| 按键 | 行为 |
| --- | --- |
| P | 静止时按当前快照规划；双窗口模式使用红点；执行中不启动新搜索 |
| E | 用当前快照一次性复查完整路径，通过后执行；到位验收通过后自动退出 |
| 空格 | 取消待执行请求，或请求停止当前执行 |
| Esc／关闭窗口／Ctrl+C | 结束会话；执行中先停止并确认静止 |
| G | 目标快照／实时显示切换；开始执行时切回实时显示 |

已有目标文件的无窗口单次执行：

```bash
./infer/Collision/start_monitor.sh --execute --plan-once --headless
```

全流程可选接入；第一步到位验收并确认静止，自动关闭双窗口和清理资源后启动腕部跟随：

```bash
python3 scripts/run_scan_workflow.py --collision
```

不加 `--collision` 继续走原 MoveJ_P 分支。启动前结束其他控制／标定进程。
执行器与已重编译的 main_rm75、arm_probe_pose 共享同一机器人地址的本机运动锁；
旧二进制、其他维护工具和其他机器不受该锁约束。

## Runtime

1. 检查规划身份、关节名称、当前起点及 URDF／Robot 限位。保留 OMPL 原始路点及必要的起点连接，
   不新增 2° 运动路点、不重新求 IK、不启用交融。
2. 按 E 时要求有效的新鲜快照，固定该快照及检查时钟，一次性复查起点连接和所有关节路径段。
   碰撞、净空不足、预算不足或中止则拒绝启动，不发送 MoveJ；复查期间尚未运动。
3. Robot 请求 Stop、确认新反馈静止后，逐个提交 MoveJ，v=movej_velocity_percent、r=0、trajectory_connect=0。
   同一 I/O 线程读取反馈、区分接受 ACK／到位事件并处理 Stop；不停止读取线程，也不使用阻塞式 TryMoveJ。
4. 执行中不等待相机场景许可、不要求 500 ms 续期，不调用相机路径复查。点云／当前状态颜色仅用于观察，
   机器人线程仍独立检查反馈新鲜度、控制器错误、关节限位及相对活动关节线段的 5° 偏差。
5. 本段完成事件和实际关节到位均通过后才发送下一原始路点；关闭交融，中间原始路点可能减速停顿。
6. 取消、机器人故障或结束整个会话时仍请求 Stop 并确认静止。相机／识别进程退出导致整个双窗口会话结束时也会停止，
   这属于会话退出，不是点云场景监督。
7. 终点到位后 Stop、确认静止，并复查关节及 FK TCP 误差；通过才返回 0 并允许阶段交接。

实现位于 [Robot 接口](../Robot/include/joint_path_executor.hpp)、[Robot 执行器](../Robot/src/joint_path_executor.cpp)、
[异步通信](../Robot/src/realman_command.cpp)及[执行适配](apps/coarse_execution.hpp)。腕部视觉伺服仍使用原 ServoJ 通道。

## 配置与当前约束

[monitor.example.json](config/monitor.example.json) 新增：

```json
"movej_velocity_percent": 6
```

取值为整数 1～100，表示控制器的速度比例，默认沿用粗定位的 6%；不是 6°/s，
也不与此前 ServoJ 的 30°/s、200°/s² 等价。控制器负责插补和加减速，实际速度需测量。
参数及回执格式见 [官方 JSON 运动协议](https://develop.realman-robotics.com/robot/json/motionConfig/)。
启动命令不变，重新启动程序读取配置；不需要额外参数。

movej_lookahead_s 已移除；自定义旧配置也应删除该字段。

| 项目 | 当前值／行为 |
| --- | --- |
| 运动后端 | 原始路点 MoveJ，`r=0`、`trajectory_connect=0`，无交融或运动队列 |
| 状态查询／监督周期 | 查询约 20 ms；执行线程约 10 ms 检查健康和时效，不周期发送 ServoJ |
| 关节限位裕量 | 至少 3°；IK、搜索、复查和执行共同限制 |
| 起点变化 | 任一关节超过 0.5° 拒绝，需重新规划 |
| 路径偏差 | 实际关节相对活动关节线段的投影偏差任一轴超过 5°，报 joint_path_deviation 并停止 |
| 执行前快照 | 进入复查时帧龄 ≤500 ms；固定快照一次性复查完整路径，运动中不续期 |
| 运动中场景监督 | 已取消；不因新点云、遮挡或相机场景期限停止 |
| 反馈异常 | 超过 200 ms／无效／控制器报错即停止 |
| MoveJ 发送／接受超时 | 提交后 500 ms 内未发出、2 s 内未被接受则停止 |
| 单段完成／总时限 | 每个 MoveJ 最多 120 s；总执行最多 10 min |
| 中间路点到位 | 接受到位事件后，关节误差 ≤0.3° 且连续两次新反馈确认；最多等 2 s |
| 终点到位 | 接受到位事件后，关节误差 ≤0.5° 连续 300 ms；最多等 3 s |
| 最终验收 | Stop 后检查关节误差 ≤0.5°；FK TCP 位置误差 ≤5 mm、姿态误差 ≤2°，静止确认通过 |
| Stop | ACK 后检查多次新反馈；未确认静止报告 stop_unconfirmed，拒绝交接 |

MoveJ 终点可能离当前关节几十度，不能把“离终点超过 5°”当作 ServoJ 跟踪故障。
当前 5° 门检查相对活动关节线段的偏离，沿路径方向的推进变慢由单段完成时限约束。
这一变化没有扩大碰撞模型或证明路径误差包络；20 mm 几何净空与物理停止扫掠仍需独立验证。

MoveJ 被中断或协议失败后，需要退出并重新启动本会话，再按 P／E。
原因是协议没有请求 ID，旧运动可能迟到的完成事件不能被误用于下一条 MoveJ；
软件锁住该会话的后续 MoveJ，不通过放宽停止回执来恢复。
正常完成后程序自动退出，再次运行可重新规划执行；IK／规划失败而未启动运动不要求重启。

反馈异常、取消或整个会话退出仍停止，尚不自动重规划。执行继承规划的未知空间策略。
默认 unknown_space_allowed 不证明盲区无障碍；真实跟踪、时序及制动扫掠仍待现场验收。

## 构建与结果

使用已有依赖和下载的 RealSense SDK，不构建 SDK：

```bash
cmake --build infer/Collision/build-monitor --target collision_monitor -j2
cmake --build infer/Robot/build --target main_rm75 arm_probe_pose -j2
```

默认仅增加 execution_started、execution_rejected、execution_result 关键事件，结果标记 backend=movej。
execution_started 显示 velocity_percent、blend=false、scene_supervision=false 及实际 MoveJ 段数。
失败／取消时 execution_result 附 stop_diagnostics，仅输出一次，不逐周期刷屏。
诊断在首次触发停止时冻结，不被随后 Stop／静止确认的反馈覆盖：
segment／phase 为停止所在段和阶段；joint 是绝对偏差最大的关节，
reference_deg／actual_deg／error_deg 为当前关节线段投影参考、反馈及有符号差值（反馈减参考），
reference_kind=active_joint_edge_projection；
feedback_age_ms 是主机反馈接收年龄，不是控制器采样延迟，另记录反馈序号、有效／过期标记、
控制器错误码。场景期限、前瞻及 scene_check 诊断已随运行中场景监督一起移除。
只有 execution_result.status=completed 且 stationary_confirmed=true 才表示软件到位验收通过。
规划失败、停止、超时、到位不合格或停止未确认均不启动下一阶段。
