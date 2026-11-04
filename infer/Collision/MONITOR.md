# C++ 只读相机—机械臂碰撞监测

更新：2026-11-03。入口为 `build-monitor/collision_monitor`，当前已完成编译、SDK 动态链接和离线夹具验证；真实 D455／RM75 联机与空间误差尚未验收。

模块分层、坐标／时间契约及与 Robot 的连接边界见 [ARCHITECTURE.md](ARCHITECTURE.md)；后续规划与执行路线见 [README.md](README.md)。

本工具可以打开 D455、通过独立 TCP 连接读取七轴反馈，显示 Base 下点云和机械臂，查询已观测障碍物的距离。
它不发送 MoveJ、ServoJ、Hold 或 Stop，不会控制或自动保护正在运动的机械臂；已支持按 P 用相机快照触发 OMPL，Robot 停止监督尚未接入。
独立人工场景 OMPL 入口见 [PLANNING.md](PLANNING.md)，系统架构与进展见 [ARCHITECTURE.md](ARCHITECTURE.md)、[PROGRESS.md](PROGRESS.md)。
所有非机器人深度观测都参与障碍物检查，包括人体、床和设备，当前不进行人体分类。

## 当前本机直接使用

实时启动默认启用粗定位，无需 `--coarse`；在窗口按 P 规划。加 `--monitor-only` 仅监测，原 `--coarse` 参数仍兼容。
回放仍默认只监测，需要规划时显式加 `--coarse` 或 `--goal`。
默认精简输出：启用规划时不打印正常采集帧，只在规划后输出一行 `camera_plan` 结果；
无目标监测时只在状态或风险对象变化时打印。输入失效和致命错误仍报告。
不再默认打印重复候选 IK 检查、完整关节路径、标定摘要和大量时间字段。
需要完整 JSON（包括逐帧信息和路径）时使用 `--verbose`：

```bash
./infer/Collision/start_monitor.sh --plan-once --headless --verbose
```

精简规划结果仅包含 `event`、`status`、`elapsed_ms`（保留 0.1 ms）、`waypoint_count`、
`unknown_space_allowed` 和 `ready_for_execution`；失败时附 `detail`，有碰撞／净空违规时附主要
`failure`（`body`、`other_body`、`distance_mm`）。原始帧录制不受影响。
起终点检查、逐候选结果、查询计数、最近点坐标、法向偏差及未知空间详细诊断仅在 `--verbose` 完整日志中输出。

按用户最新要求，粗定位 `planning_options.allow_unknown_space=true`：目标候选、OMPL 搜索、路径段及最终复查均允许未知区域，
不再因缺少自由射线而拒绝。已观测占据、整臂净空、自碰撞、关节限位、预算、相机和反馈时效仍检查。
地图不会把未知体素改为自由；独立监测仍使用严格未知空间判据。规划结果包含 `unknown_space_allowed:true`，
`exact_solution` 仅表示在允许未知的策略下找到并复查路径，不代表已验证未观测区域无障碍；`ready_for_execution` 仍为 false。
将该选项设为 false 恢复严格模式，库及其他未配置场景默认 false。下文的未知体素拒绝和原因诊断适用于严格模式。
`--plan-once` 默认最多等待 30 帧；未取得有效帧时也输出 `event=camera_plan`、`status=no_valid_frame`，
并附最后一帧的 `input_status`／`detail` 和帧数，原有 grep 过滤不会再隐藏此次失败原因。

日常启动在仓库根目录执行一条命令：

```bash
./infer/Collision/start_monitor.sh
```

也可以在 `infer/Collision` 目录执行 `./start_monitor.sh`。脚本按自身位置定位资源，不依赖当前工作目录，
默认打开支持 P 粗定位规划的只读实时窗口，加载 `config/coarse.goal.example.json`、`rm_75.urdf`、`mesh/`、`config/monitor.example.json` 和
`../hand_eye_calibration-main/d455_to_rm75_base.json`，启用已有模型所需的内存零面积面清理。
机械臂默认 `192.168.50.254:8080`，相机序列号取自标定文件；修改体素等设置直接编辑上述 JSON。
脚本自动添加本地 RealSense 运行库和 FCL 临时前缀到本次进程的动态库路径，不编译或下载 SDK。

覆盖参数的示例：

```bash
./infer/Collision/start_monitor.sh --robot-ip 192.168.50.254
./infer/Collision/start_monitor.sh --headless --max-frames 30
./infer/Collision/start_monitor.sh --pixel-stride 16
./infer/Collision/start_monitor.sh --config /absolute/path/to/monitor.json
```

参数覆盖优先于脚本默认值，仍支持原生 CLI 的所有现有参数。
也可用环境变量 `COLLISION_ROBOT_IP`／`COLLISION_ROBOT_PORT` 设置本终端的默认地址，
`COLLISION_MONITOR_BIN` 指定其他已构建的程序，`COLLISION_DEPS_PREFIX` 指定其他 FCL 依赖前缀
（默认 `/tmp/uspilot-collision-deps/root/usr`，脚本取其 `lib/x86_64-linux-gnu`）。
原生 `build-monitor/collision_monitor` 的显式参数要求保持不变。

检查 SDK 头文件和运行库，不连接硬件：

```bash
./infer/Collision/start_monitor.sh --sdk-info
```

当前头文件版本应为 `2.59.1`，运行库 API 应为 `25901`。

回放使用用户录制的 JSONL；合成测试夹具已移除。加入 `--headless` 可在没有桌面的终端只输出结果。
回放不会访问相机或机器人，录制与回放方法见本文相应操作说明。

设备连接好后，直接执行无参数启动命令即可使用真实 RM75 模型与实际 D455 外参。等价的原生调用为：

```bash
infer/Collision/build-monitor/collision_monitor \
  --urdf infer/Collision/rm_75.urdf \
  --mesh-dir infer/Collision/mesh --drop-zero-area-triangles \
  --calibration infer/hand_eye_calibration-main/d455_to_rm75_base.json \
  --config infer/Collision/config/monitor.example.json \
  --live --robot-ip 192.168.50.254 --robot-port 8080
```

这是**独立只读调试入口**：关闭占用同一 D455 的其他采集程序；不要把它作为生产 Robot 的并行反馈／控制集成方式。
现有 Robot 控制进程已运行时，后续应从其反馈缓存或受控的遥测接口取状态，避免依赖控制器的多连接行为；当前没有实现该桥接。
相机序列号从外参文件读取，不自动选择另一台相机。当前实际标定文件记录 D455 序列号 `318122303626`。
机械臂 IP 需与实际网络配置一致。脚本检测到 `--replay` 时不添加实时模式或默认机械臂地址，回放不连接设备。

鼠标左键拖动旋转视角，滚轮缩放，Esc 或 Ctrl+C 退出。退出仅关闭自身连接和相机，不向机械臂发送停止或运动命令。
需要无人值守的有限帧诊断时加入 `--max-frames 30 --headless`。

## 用当前相机场景规划

最小实现直接复用本进程的相机、七轴反馈、标定和 OctoMap，不启动第二个采集进程。
先在 [planning.goal.example.json](config/planning.goal.example.json) 填入目标 `joint1`～`joint7`，单位 **rad**。
文件里的全零角度仅为占位示例，真实 RM75 零位目前会报告自碰撞／净空违规。

```bash
./infer/Collision/start_monitor.sh --goal infer/Collision/config/planning.goal.example.json
```

窗口中按 **P**：重读目标文件，以当前帧反馈为起点、该帧 `CollisionScene` 为环境，调用已有 RRTConnect。
成功时显示青绿色末端曲线，终端输出一行 `event=camera_plan` JSON，默认只包含关键结果。
`--verbose` 才输出完整关节 `waypoints`、场景／标定／模型身份和时间。曲线依据已检查的关节线性插值路径，
按各轴最大 0.02 rad 间隔采样 FK；粗定位使用 Link7 与本次 TCP 标定，显示探头 TCP，而非 STL 原点或法兰。
曲线覆盖模型绘制，起点绿色、终点品红。快照过期后保留橙色历史曲线，窗口标记 `STALE snapshot`，
不表示当前场景有效。P 重新规划会替换曲线，失败则清除旧曲线，不绘制未经规划的起终点直线。
这是几何路径预览，没有时间参数化，也不驱动机械臂。目标文件可在运行中修改，下一次按 P 时生效。

若没有曲线，先看窗口标题开头：`NO PATH` 后显示本次规划状态；`invalid_goal`、`timeout` 且
`waypoint_count=0` 表示没有生成路径，目标白点／模型预览并不是轨迹。标题在目标快照模式下也保留规划失败原因。
成功生成曲线时自动调整观察中心与距离，包含整条曲线；完整日志的 `state_queries`、`segment_queries` 可用于定位耗时。
规划器只在同一次静态快照调用内缓存通过的状态和完整路径段证书，最多各 4096 项，正反方向共享有效段证书；
命中前仍检查截止时间／取消／查询预算，不跨场景复用，不缓存失败段或未经完整认证的部分路径。
最终路径验证可复用同一快照的有效证书，不能将缓存命中解释为新鲜场景复查。
路径段认证使用每个碰撞对的距离及相对位移界：环境障碍物静止，只扣对应机器人部件的位移；
自碰撞对不计共同祖先关节施加的共同刚体运动。仍验证整段，证明不了则细分，预算耗尽拒绝。
完整日志包含 `segment_state_checks`，统计路径段内部实际调用状态检查的次数；它不等于 `state_queries`，
可用来区分少量段查询内发生大量递归细分的情况。两者都不包含已缓存证书的命中。

无窗口计算一次：

```bash
./infer/Collision/start_monitor.sh --goal infer/Collision/config/planning.goal.example.json --plan-once --headless
```

`--plan-once` 在首个输入有效的帧上尝试一次规划并退出，默认最多等待 30 帧，可用 `--max-frames N` 调整。
退出 0 为精确快照路径，2 为规划拒绝／没有有效帧，1 为程序或输入错误。
`--replay FILE` 可用同一入口对已录制的相机帧规划，使用录制时的模型、外参和配置。

实时规划同步运行，期间暂停本进程的逐帧处理。计算预算同时受目标文件 `timeout_s` 和快照剩余帧龄约束，
当前粗定位总预算 60.0 秒，日常监测配置帧龄上限 61.0 秒，为采集和建图留出时间；
该帧龄配置也影响独立监测，不代表 61 秒内人体场景保持不变。仍是只读快照规划，不授予执行权限。
结束后再次检查帧龄，过期不输出有效路径；不会刷新采集时间或绕过未知空间。
`invalid_start`／`invalid_goal` 的检查细节会区分碰撞、未知空间和无效关节。

粗定位日常配置已设置 `planning_options.skip_start_precheck=true`：跳过 IK 前及进入 OMPL 前的单独起点预检查，
日志 `start_check.status` 为 `skipped`，`detail` 为 `skipped_start_precheck`，不表示起点有效。
目标候选、OMPL 内部状态检查、整段运动验证及最终路径复查仍保留，因此起点不满足条件时仍可能无解。
此选项不会清空未知空间或允许执行；设为 `false` 可恢复预检查，库及其他配置默认 `false`。
除直接输入关节角外，已支持下节的颈动脉粗定位目标和多候选 IK；仍无轨迹执行、动态避障或停止监督，始终 `ready_for_execution=false`。

此功能要求监测与规划同时编译。本机已有构建可执行：

```bash
cmake -S infer/Collision -B infer/Collision/build-monitor -DBUILD_COLLISION_PLANNING=ON
cmake --build infer/Collision/build-monitor --target collision_monitor -j2
```

## 从颈动脉点开始粗定位规划

在上游数字人／目标采集窗口保存 `infer/Camera_RT/artery_path.txt`，然后退出采集程序，释放 D455。
默认允许读取已有旧文件，不要求每次重新保存；旧点不代表当前人体位置仍然一致。
只需要导出目标数据；本入口不调用 Python，不启动旧的 `run_probe_target.py` 或 `arm_probe_pose`。
旧入口可能直接执行 MoveJ_P，不用于验证本规划链路。

```bash
./infer/Collision/start_monitor.sh --coarse
```

窗口按 **P** 后执行：TXT 顶部首点／法向／切向 → 同一相机标定下的 Base 目标 → 探头 TCP 沿外法向后退 200 mm →
URDF 数值 IK 候选 → 当前相机场景的整臂状态检查 → OMPL → 全路径复查。绿色线和 JSON 结果仍为只读预览。
无窗口计算一次使用 `./infer/Collision/start_monitor.sh --coarse --plan-once --headless`。

默认配置见 [coarse.goal.example.json](config/coarse.goal.example.json)，可用 `--goal FILE` 覆盖。
该 JSON 的 `coarse_target` 与 `goal_joint_positions_rad` 互斥；文件路径相对于目标 JSON 所在目录解析。

| 配置 | 默认 | 含义 |
| --- | --- | --- |
| `artery_path_file` | `Camera_RT/artery_path.txt` | 第一行是朝相机的外法向，后续是 Color optical 下的米制点；首点为本次目标 |
| `camera_calibration_file` | 当前 D455→Base 文件 | 摘要及序列号必须与当前监测场景一致 |
| `tool_calibration_file` | `Robot/build/rm75_force_calibration.json` | 复用 Robot schema-v2 中的 ArmTip／Sensor／Tool 和探头 TCP 几何，要求几何链已确认 |
| `arm_tip_link` | `Link7` | 控制器 ArmTip 对应的 URDF 连杆，不能把 CAD 的 `Lprobe_link` 原点当作 TCP |
| `standoff_m` | `0.20` | 沿表面外法向（目标 Tool -Z）后退 200 mm；不是整臂净空，整臂仍受 FCL 约束 |
| `max_target_age_s` | `0` | 0 关闭文件时效检查，允许旧文件；正数（最多 3600 秒）恢复读取前及规划后的时效检查 |
| `surface_match_tolerance_m` | `0` | 0 关闭目标点云匹配门限，距离仅用于诊断；正数（m，最大 0.1）可重新启用 |
| `ik_options` | 每姿态最多 8 初值、80 次迭代，共享 4 s | 有界数值 IK；收敛容差 1 mm／0.01 rad，不是物理精度承诺 |
| `ik_options.yaw_offsets_deg` | 0 | 不额外绕法向旋转，不改变 TCP 目标位置 |
| `ik_options.tilt_offsets_deg` | `[0,0]`、`[0,9]`、`[9,0]`、`[-9,0]`、`[0,-9]` | TCP 局部轴的倾斜旋转向量 `[rx,ry]`，单位度，按右手定则；共 5 种姿态 |
| `ik_options.max_normal_deviation_deg` | 10 | 每个收敛 IK 解的终点法向偏差必须不超过此值，配置上限为 10° |
| `ik_options.max_candidates_per_pose` | 1 | 每种姿态最多保留的关节构型，允许 1～4；当前全局最多 5 个候选 |

原始法向与切向给出名义姿态；终点搜索保持沿原外法向后退 0.20 m 的位置，组合绕法向旋转与 9° 倾斜。
采样 9° 为 10° 上限留数值余量，最终仍通过 FK 检查实际轴线角度，超限 IK 解不会进入碰撞或路径规划。
轴线约定沿用原工程：TCP -Z 对应朝外方向，与原外法向比较；等价于 TCP +Z 与内法向比较。
10° 限制针对目标姿态，不是整条粗定位路径的姿态约束。仅设置旋转采样 `[0]`、倾斜采样 `[[0,0]]` 可恢复名义姿态。
各姿态均分剩余 IK 时间，得到的候选经过去重、按当前关节角距离排序，再筛掉碰撞／净空不合格目标。
某个姿态碰撞不再直接结束其他姿态的尝试；全部采样均无有效目标时仍返回明确失败，不保证总有可行解。

用户已确认末端固定装配正常，当前 `monitor.example.json` 显式排除
`Link7、camera_link、connector_link、force_sensor_link、Lprobe_link` 五个固定连杆内部的全部 10 对自碰撞。
按用户后续要求，另排除 base_link–Link1、Link1–Link2 至 Link6–Link7 的七对活动关节相邻连杆，
以及 support_link–base_link 的支撑安装接触，总计 18 对。
排除涵盖整对连杆的接触和自净空检查；环境及未列入排除表的其他连杆对继续检查。
这七对属于用户要求关闭的活动关节检查，不表示它们没有相对运动。修改配置后重启即可生效，无需重新编译。
排除项参与模型身份计算，旧模型身份下的路径不能直接复用。

名义目标姿态沿用原粗定位约定：Tool +Z 朝体内，+X 沿投影后的血管切向，Tool YZ 为短轴平面；
启用姿态搜索后，选中终点可绕法向旋转并在 10° 法向锥内倾斜，不能再假定最终 +X 严格沿血管切向。
TCP 变换与 Robot 的几何定义一致：`R_ArmTip_Tool = R_ArmTip_Sensor * R_Sensor_Tool`，
`p_ArmTip_TCP = R_ArmTip_Sensor * (p_Sensor_TCP - t_Sensor_Tool)`；不使用 STL 安装原点代替探头 TCP。
确认配置中的工具标定确实对应当前安装；本机默认文件仍带旧传感器标识 `DU0DU5LC`，不会自动改写成 KWR57。

IK 与 OMPL 共用总时间和碰撞查询预算；IK 自身还受 `ik_options.timeout_s` 限制，优先尝试当前构型附近的候选。
所有 IK 候选先检查终点，碰撞／净空等不合格者直接记录 `invalid_goal`，不占用 OMPL 搜索时隙。
有效候选逐一搜索，每次按“剩余总时间／尚未搜索的有效候选数”分配限时，查询预算也同样划分。
找到一条路径后继续搜索其他有效候选，最终选择已验证路径中关节空间折线长度最小者，不保证全局最优。
当前总时间 60 秒，4 个有效候选在前置处理后通常各分得约 15 秒；提前完成的余量留给后续候选。
当前采样按顺序为原姿态、局部 Y 轴 +9°、X 轴 +9°、X 轴 -9°、Y 轴 -9°，最多 5 个候选。
此处正负表示绕对应轴按右手定则的旋转方向，不是 Base 坐标系下的位置移动方向。
实际搜索时隙按通过检查的候选数量分配；4 秒 IK 也计入 60 秒共享预算。
`--verbose` 的 `target.candidate_searches` 输出每个候选的编号、状态、分配预算、耗时、路点数和关节路径长度；
其中 `yaw_offset_deg`、`normal_deviation_deg` 分别表示绕法向的旋转角和实际终点法向偏差，二者含义不同。
完整日志中的选中目标另含 `selected_yaw_offset_deg`、`selected_normal_deviation_deg`，并保留名义与选中目标矩阵。
`not_searched` 表示取消／总预算等阻止搜索，不能描述为已搜索无解。限时结束同样不证明不存在路径。
总预算耗尽时仍可保留此前已完整认证的最佳路径，但取消、目标或相机过期仍清除路径并拒绝。
`ik_no_solution` 表示预算内未找到收敛解；`invalid_goal` 带候选碰撞／未知空间原因，不回退直接运动。
`--verbose` 输出的 `target` 包含源文件摘要、保存时间、标定摘要、表面点、目标 TCP 变换、IK 候选和残差。

按用户要求，当前已取消 30 mm 目标点云匹配限制：默认 `surface_match_tolerance_m=0`，
即使最近点距离超过原门限或没有可用点，也不会仅因此拒绝规划。输出 `surface_match_required=false`，距离诊断仍保留。
目标文件时效默认关闭，身份、机器人遮挡和路径碰撞／未知空间检查继续生效；相机帧与反馈时效不变。
仅在显式配置正数门限后，匹配不足才返回 `target_not_observed`，并停在 IK／OMPL 之前。
诊断 `target.surface_observation_stages_m` 分别记录最近点距离：
`valid_depth`（深度有效性过滤后）、`after_robot_mask`（自身过滤后）、`after_occlusion_filter`（遮挡过滤后）、
`after_outlier_filter`（稀疏过滤后）。单位为 m，无对应点时为 null。
若第一阶段已超过门限，需核对本轮目标与当前深度是否对应；若某一步才增大，核对该过滤的模型、外参或参数。
`surface_observation_error_m` 仍为最后阶段的距离，新增 `surface_match_tolerance_m` 和
`nearest_surface_point_base_m` 便于核对阈值及最近保留点的位置。
目标阶段拒绝时，起终点检查中的默认 `invalid_input` 不表示实际关节检查失败，需结合 `state_queries=0` 判断尚未检查。

原 TXT 没有相机身份或曝光时间，本次依照指定标定及文件保存时间关联，并用当前点云检查目标位置支持。
保存时间不等于曝光时间，也不能证明患者未移动或模型点对应真实血管；换机、换标定或目标移动后必须重新采集。
不要通过刷新旧文件时间代替采集。真实空间与运动验收仍由后续现场验证完成。

## 相机射线遮挡盲区

当前 [monitor.example.json](config/monitor.example.json) 已开启：

```json
"robot_occlusion_filter": true
```

盲区由相机视线与机器人碰撞实体自动确定，不使用固定 XYZ 方框。
射线起点是经过 Color→Base 变换的真实深度光心；机械臂位姿使用当前帧七轴反馈，附件和支撑使用 URDF 固定链。
相机到首个膨胀模型表面之前的观测可保留；模型表面及其后的观测被判为遮挡，跳过占据写入与整条自由射线，窗口也不再显示。
使用既有 `mask_padding_m` 作为模型遮挡外扩；自碰撞排除不影响实体投影，所有机器人部件仍可产生遮挡。

对拟写入的自由／占据体素也做可见性检查，模型额外外扩体素包围球半径，以保护擦过轮廓的体素。
遮挡体素保持未知，其他射线不会将其清成自由。过滤不会自动授予穿过盲区的规划许可，仍可能返回 `unknown_space`。
凸包会填平支架空腔，因此这种遮挡判据偏保守，须结合实际装配及外参验证。

- `occluded_points`：因视线被机器人截获而丢弃的点数，包含表面自身点；可能与 `masked_points` 重合。
- `occluded_voxels`：因体素可见性不足而取消写入的候选体素数，包含自由和占据候选。
- `camera_plan.point_filter`：在 `--verbose` 完整输出中包含同样的开关及计数。
- `target_occluded`：颈动脉表面目标处于当前机器人遮挡下，拒绝这次目标规划。

设为 `false` 可关闭该过滤。库及未提供新字段的旧配置默认关闭；日常配置已显式开启。
每批射线检查限制为 2000 万次几何工作量计数，超限拒绝整帧，不使用部分结果。
当前实现只针对已建模机器人实体的遮挡；它不是相机量程、RGB 视场或未建模物体遮挡的自动识别。
本轮仅完成编译与静态检查，未执行点云算法、回放或设备测试，运行验证由用户完成。

## 稀疏噪点过滤

当前 C++ 监测在建 OctoMap 前执行半径邻域过滤，不依赖 PCL、Python 或新的点云库。
默认 [monitor.example.json](config/monitor.example.json) 已启用：

```json
"outlier_radius_m": 0.03,
"outlier_min_neighbors": 5
```

每个有效、非机器人点必须在半径 3 cm 内找到至少 5 个**其他未被机器人过滤或遮挡的点**，否则标记为稀疏离群点。
机器人过滤点和遮挡点不提供环境密度支持；这是单帧点密度判断，不识别人体、线缆或物体类别。

- `outlier_radius_m`：搜索半径，支持 0.005～0.2 m。相同邻居阈值下，减小半径会过滤更多点。
- `outlier_min_neighbors`：其他邻居数量，支持 0～64。增大数量会过滤更多点，设为 0 关闭过滤。
- 稀疏的远处表面、细小物体也可能被标记，调参需结合距离及 `--pixel-stride` 核对，不能把过滤等同于确认没有障碍。

修改 JSON 后重启即可，无需增加启动参数：

```bash
./infer/Collision/start_monitor.sh
```

橙色点是被过滤的离群点，不写入占据，也不提供整条深度射线的自由空间证据。
其端点邻域保护为未知，不能被其他射线清为自由；同体素有保留观测占据时，占据仍优先。
因此过滤后碰撞报告可能减少，但仍可能返回 `unknown_space`。粗定位体表匹配也不使用这些离群点。
`--verbose` 时终端逐帧输出 `outlier_points`、有效半径和邻居阈值；完整 `camera_plan.point_filter` 带相同诊断。
录制保存原始深度输入，回放时按所选配置重新过滤。

邻域查询采用空间网格限制候选，不进行全点两两比较；单帧超过 200 万次候选距离查询时拒绝该帧，
不会拿部分过滤结果继续建图。C++ 库默认邻居阈值为 0，旧配置未提供新字段时保持原处理方式；当前 RM75 配置由用户调整为 5。

## 看懂窗口和输出

粗定位目标净空诊断：运行 `./infer/Collision/start_monitor.sh --coarse`，在窗口按 **P**。
完成 IK 候选检查后自动进入 `GOAL SNAPSHOT (not live)`，冻结本次规划场景及采集时关节构型；
成功时显示选中的最佳候选，失败时显示最后一个已搜索／检查的候选（`goal_candidate_index`，从 0 开始），与 `goal_check` 对应。
使用 **G** 在该快照和实时监测之间切换，再按 P 使用新帧重新诊断；预览不会因快照过期自动消失，
它只用于检查当时失败的位置，不表示路径当前仍有效。

| 显示 | 含义 |
| --- | --- |
| 灰色原模型 | 采集时机械臂构型 |
| 橙色目标线框 | 目标构型下实际参与 FCL 的凸包／基本体 |
| 红色目标线框 | 报告违规的目标碰撞体 |
| 绿色目标表面 | 违规部件在同一目标姿态的原始 STL，便于与凸包对照 |
| 白色点 | 目标 TCP，按颈动脉表面点沿外法向后退 200 mm |
| 黄色端点和连线 | FCL 报告的部件／环境最近点及间距 |
| 附近橙色体素框 | 环境最近点周围 50 mm 内的占据体素，来自同一规划快照 |

完整日志的 `goal_check.violations` 包含 `robot_point_base_m`、`environment_point_base_m`，
均为 Base 下 m 坐标。只在分离但净空不足的环境对上请求最近点，输出前检查有限性及线段长度与距离的一致性。
最近点归属按 FCL 结果的 `o1/o2` 映射，不能假定与调用参数顺序相同；shape/octree 查询可能反向返回。
相交、无可用最近点或检查不一致时输出 null，不将其解释为接触点或最近体素中心。
绿色 STL 与红色凸包有差异只能提示建模保守性，不能单凭表面渲染证明体积无碰撞。
无窗口时使用 `--headless --plan-once --verbose` 查看坐标；窗口诊断不要加前两个选项，否则无法停留查看。

| 显示／字段 | 含义 |
| --- | --- |
| 青色点 | 保留的非机器人深度观测，已转到 Base |
| 橙色点 | 邻域密度不足的离群点，不作为占据或自由射线证据 |
| 灰色点 | 落在机器人保守包络内的点；对应端点附近保留未知，不作自由空间 |
| 紫色点 | 已占据体素中心，包含配置的端点膨胀 |
| 红色机械臂部件 | 与已观测环境碰撞或净空不足 |
| 琥珀色部件 | 自碰撞报告，不表示已确认实体装配错误 |
| `environment_distance_m` | 机器人到当前占据体素的最小距离；没有可用环境查询时为 null |
| `self_distance_m` | 当前自碰撞对的最小距离，与环境风险分开显示 |
| `status=unknown_space` | 机器人及裕量范围没有被完整观测为自由空间 |
| `detail=capture_time_untrusted` | SDK 时间域没有可用的主机时间映射，不用接收时间冒充曝光时间 |
| `detail=robot_feedback_not_time_matched` | 相机和反馈超出配置时间匹配范围 |
| `detail=camera_frame_expired_during_processing` | 计算结束时原始采集时间已过期，未刷新成新观测 |
| `input_valid` | 本帧身份／时序／输入处理是否有效，不代表无碰撞、完整覆盖或可运动 |
| `outlier_points` | 本帧被邻域密度过滤的非机器人点数 |
| `occluded_points` / `occluded_voxels` | 射线遮挡过滤的点数及取消写入的体素数 |

当前标准模型零位仍可能报告安装接触和单凸包重叠；本工具不会因此忽略人体／环境查询，也不会自动扩大自碰撞豁免。
`violations` 中 `self_collision=false` 对应已观测环境，`other_body=environment/depth_occupancy` 表示相机占据体素。
JSON 的 `robot_motion_enabled` 和 `ready_for_motion_planning` 始终为 false。

退出码：0 表示处理过有效输入帧或用户正常退出；1 表示源／配置／文件错误；2 表示处理的帧全部无效。
**退出 0 不代表净空通过**，应读取逐帧 status、detail 和 violations。
SDK／相机报错会结束本进程，不会静默切换相机；机器人状态故障或陈旧帧会标为无效。

## 录制和回放

在实时命令末尾增加：

```bash
--record /absolute/path/to/new_recording.jsonl
```

父目录必须存在，文件必须是新文件，工具拒绝覆盖旧录制。
每行保存该帧的彩色光学坐标点、深度射线原点、七轴角度、相机身份及原始时序信息；不录制 RGB 图像。
录制是同步诊断 I/O，不用于生产实时控制预算。日常录制和回放可直接使用：

```bash
./infer/Collision/start_monitor.sh --record /absolute/path/to/new_recording.jsonl
./infer/Collision/start_monitor.sh --replay /absolute/path/to/new_recording.jsonl
```

回放保留采集时使用的同一真实 URDF、外参和配置；自定义过这些文件时，继续传入对应覆盖参数。

录制字段 `captured_at_ns`、`received_at_ns`、`robot_at_ns`、`now_ns` 属于同一主机单调时钟域。
SDK 设备时间经主机时钟映射后才接受为采集时间；只接受 global/system 时间域且年龄在允许范围内的深度帧。
机械臂协议没有设备采样时间，当前用状态响应接收时间配对，读取往返超过 100 ms 的结果拒绝。
这仍有反馈取样时刻与曝光同步的不确定性，当前适合静止／受控条件下的空间诊断，不能当作动态停止包络验证。

## 场景与配置边界

配置：[monitor.example.json](config/monitor.example.json)。这些参数都是开发初值，未作为真机安全裕量验收。

| 参数 | 当前示例 | 用途 |
| --- | --- | --- |
| `region_min_m/max_m` | Base 下约 2×2×1.7 m | 建图裁剪范围，**不是已知自由空间** |
| `voxel_m` | 0.01 m | 占据分辨率 |
| `min_depth_m/max_depth_m` | 0.05～3 m | 深度端点过滤范围，不代表设备保证的测距范围 |
| `endpoint_padding_voxels` | 0 | 非机器人端点向各轴邻近体素膨胀；当前不额外膨胀 |
| `mask_padding_m` | 0.01 m | 机器人保守包络的过滤外扩 |
| `robot_occlusion_filter` | true | 按当前机器人实体剔除表面和遮挡后方的点，盲区不作为自由空间 |
| `outlier_radius_m` | 0.03 m | 稀疏点过滤的邻域搜索半径 |
| `outlier_min_neighbors` | 5 | 半径内至少需要的其他有效环境点数，0 关闭过滤 |
| `clearance_m` | 0.02 m | 对当前几何／体素的示例距离要求 |
| `self_collision_exclusions` | 共 18 对 | 末端固定组件内部 10 对、活动关节相邻 7 对、支撑安装 1 对；环境及其他连杆对仍检查 |
| `max_points/max_voxels` | 60000／300000 | 超限拒绝，不用截断后的不完整场景报通过 |
| `max_feedback_offset_ns` | 100 ms | 反馈接收与深度采集时间的最大偏移 |
| `max_frame_age_ns` | 61000 ms | 原始观测最大年龄，为 60 秒规划预算留余量；计算结束仍再次检查 |

默认 1280×720、30 FPS 请求彩色和深度，`--pixel-stride 8` 每隔八个像素取一个深度点。
用原始深度内参与 SDK 生成点云，再应用深度→彩色外参，最后应用文件中的彩色光学系→Base 外参；不混用深度光学系与彩色标定。
实际帧率、采集延迟、体素误差和 CPU 开销需要在现场测量，程序不保证达到相机请求帧率。

每帧从未知空间重新建图：只有有效射线经过的体素可记为自由，端点膨胀记为占据；无效深度和视野外保持未知。
占据优先于自由，机器人过滤端点周围也不会被另一条射线清成自由。由于采用单凸包，靠近支架凹槽的人体点也可能被标为机器人候选；
这些点被保留为未知，不能产生通行结论。单相机无法观测的人体背面和机器人自身遮挡区域仍会使检查拒绝。
运动部件的 AABB 仅枚举候选体素。FCL 检查实际碰撞几何与完整体素立方体的距离，
只要求实体加至少 30 mm 查询裕量所涉及的体素由观测自由空间或采集姿态的机器人实体内部覆盖。
无关位置可以保持未知，路径新增覆盖的相关未知体素仍拒绝；整段使用位移界认证，并非只检查路点。
自身覆盖只接受整个体素包围球完全处于一个模型实体内的情况，不外扩，不包含遮挡阴影，也不会随候选姿态移动。
沿用实心凸包／基本体建模假设，实际空腔不能依靠外包络认领为自身实体，模型一致性须现场核验。
Base 固定底座及支撑跳过自由空间证明，但保留 FCL 障碍物和未排除的自碰撞检查；末端附件仍随整臂检查。
相关未知体素（包括超出 ROI）仍拒绝，不能填写人工 known_space 绕过。边界体素和查询裕量仍偏保守，
改动不保证单相机可以规划成功，也不会扩大默认 ROI 或清空盲区。

`--verbose` 完整日志的 `camera_plan.start_check/goal_check`、候选 IK 检查以及监测报告包含 `unknown_space_bodies`，例如
`["Link1:unobserved_cell"]`。`unobserved_cell` 表示未观测；`outside_observation_region` 表示自身实体以外的
查询体素超出建图范围；`known_space_query_budget` 表示查询体素预算不足；`occupied_cell` 表示查询覆盖占据体素。
该列表列出每个部件首先遇到的原因，可能与碰撞报告同时存在。空列表不能单独视为路径有效。
`unknown_space_points_base_m` 与该列表按下标对应，给出首个失败体素中心（Base，m）；
地址／预算失败未定位体素时为 `null`。该坐标是被拒体素位置，不是人体表面或实际接触点。

`unknown_space_diagnostics` 对每个失败碰撞体增加以下诊断（同样出现在 IK 候选检查中）：

| 字段 | 用途 |
| --- | --- |
| `body_to_cell_distance_lower_bound_m` | 当前碰撞体到整个失败体素立方体的距离下界，已扣数值保护，单位 m |
| `first_cell_scope` | `margin_only`：首个失败体素只落在外扩范围；`body_or_numeric_guard`：与实体相交或处于数值保护范围；`not_located`：未定位 |
| `query_margin_m` | 实际查询裕量，当前至少 0.03 m，与障碍物净空仍相关联 |
| `geometry_only_status/reason` | 对完整碰撞体再做一次 0 外扩对照检查；`covered` 表示该部件仅因外扩拒绝；`rejected` 表示去掉外扩仍有覆盖问题；`not_checked` 表示未执行 |
| `geometry_only_failure_point_base_m` | 0 外扩对照检查的首个失败体素中心，Base，m |
| `cell_evidence` / `geometry_only_cell_evidence` | 两个失败位置在本帧建图中的实际处理记录，见下文 |
| `center_inside_captured_robot` / `geometry_only_center_inside_captured_robot` | 体素中心是否在采集时自身实体内；true 仍不能证明整个体素都属于自身 |
| `capture_robot_occludes_cell` / `geometry_only_capture_robot_occludes_cell` | 采集姿态实体按现有保守体素遮挡判据是否遮挡该位置；null 表示未启用或无上下文 |

`cell_evidence` 为 `robot_occlusion_withheld` 时，确认该候选体素被机器人遮挡过滤保留为未知；
`robot_or_outlier_endpoint_protected` 表示端点被自身或稀疏点过滤保护；`no_retained_free_ray` 表示本帧没有保留自由证据，
仅凭此值不能区分视野外、无效深度、射线采样空隙或被提前剔除的射线。观测占据优先报告 `observed_occupied`。
首个失败体素按遍历顺序选取，不是最近未知位置；不能只凭 `margin_only` 就认定整个构型没有实体覆盖问题。
0 外扩仅供定位原因，不改变状态、净空、未知拒绝、路径认证或执行权限，也不是降低裕量的建议。
诊断会增加失败状态查询耗时；仍受既有体素预算限制，整体规划／帧龄超时照常拒绝。

目前没有跨帧融合、人体分割或在线避障／停止功能。

## 使用下载的 SDK 构建

本机用户 SDK 目录是 `librealsense/`，提供 2.59.1 头文件。目录中没有 `librealsense2.so`；已额外下载相同版本的**官方预编译运行包**，
解包到 `.deps/realsense/`，不编译 SDK，也不修改系统安装。第三方 SDK、运行库、构建及录制均按本地资源忽略，不纳入提交。

运行库来源：<https://librealsense.realsenseai.com/Debian/apt-repo/pool/jammy/main/librealsense2_2.59.1-0~realsense.20269_amd64.deb>。
包 SHA256：`672f2fca3b97fdf68cdc3a2524fc0f3b4ba8bdf9b32933bd8a508954cfa8d9bd`，已与官方索引一致性核对。

```bash
cmake -S infer/Collision -B infer/Collision/build-monitor \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_COLLISION_CHECKING=ON -DBUILD_COLLISION_MONITOR=ON \
  -DBUILD_REALSENSE_SOURCE=ON \
  -DCMAKE_PREFIX_PATH=/tmp/uspilot-collision-deps/root/usr \
  -DCOLLISION_REALSENSE_INCLUDE_DIR="$PWD/infer/Collision/librealsense/include" \
  -DCOLLISION_REALSENSE_LIBRARY="$PWD/infer/Collision/.deps/realsense/usr/lib/x86_64-linux-gnu/librealsense2.so.2.59.1"
cmake --build infer/Collision/build-monitor -j2
```

脚本只替自身启动的进程设置动态库路径；直接运行原生程序时仍需自行设置运行库环境。
若换用其他预编译 SDK，调整 include／library 参数并检查 `--sdk-info` 的版本匹配。也支持系统提供的 `realsense2::realsense2` CMake 包。
SDK 头文件或共享库缺失时配置直接报错，不下载、不触发 SDK 源码构建。回放可以关闭 `BUILD_REALSENSE_SOURCE`，
纯无桌面构建还可关闭 `BUILD_MONITOR_VIEWER`；原几何构建和仅 FCL 构建保持独立。

实现入口：[监测处理](src/monitor.cpp)、[实时源](src/live_source.cpp)、[只读机械臂封装](src/robot_read_only.cpp)、[命令行](apps/collision_monitor.cpp)、[3D 窗口](apps/monitor_viewer.cpp)。
当前验证覆盖合成观测、未知空间、过滤、时效、预算、录制回放，以及伪控制器对只读查询的检查；不等于真实相机／机械臂联机或空间精度验收。
