# C++ 只读关节路径规划

更新日期：2026-11-03。当前实现 OMPL RRTConnect、FCL 状态／关节段约束、路径简化与最终复查、JSON 输出和可选动画。
程序不连接机械臂、相机或 Redis，不发送任何运动或停止指令。系统架构见 [ARCHITECTURE.md](ARCHITECTURE.md)，进展见 [PROGRESS.md](PROGRESS.md)。

以上指独立 `collision_plan`。相机与 OMPL 的最小联动已在 `collision_monitor` 中接通：
`./infer/Collision/start_monitor.sh --goal infer/Collision/config/planning.goal.example.json`，窗口按 P 使用当前相机快照规划。
目标角度须先填写，完整操作见 [MONITOR.md](MONITOR.md#用当前相机场景规划)。

颈动脉粗定位使用 `./infer/Collision/start_monitor.sh --coarse`：读取新保存的采样点，转换探头 TCP 目标，
求多候选 IK 后在当前相机场景检查及规划；操作与坐标约定见 [粗定位说明](MONITOR.md#从颈动脉点开始粗定位规划)。

## 日常命令

在仓库根目录执行，无参数使用随仓库提供的**合成七转轴模型及球形障碍物**，用于验证绕行，不能代表 RM75：

```bash
./infer/Collision/start_plan.sh
./infer/Collision/start_plan.sh --preview
```

`--preview` 显示机械臂关节线性插值动画、紫色人工障碍物及青绿色末端采样曲线。
绿色线仅连接路点处的末端位置，真实末端运动可能呈曲线，碰撞检查依据整臂关节运动。
动画每段约 0.5 s，只表达几何路径；没有时间参数化，不代表真机速度或执行轨迹。
窗口左键旋转、滚轮缩放、Esc 退出，Ctrl+C 取消搜索或结束显示。
默认不打开窗口，便于无人值守离线检查。脚本设置本次进程的依赖路径，不自动安装或构建任何依赖。

保存报告时，输出文件必须不存在，父目录须已存在：

```bash
./infer/Collision/start_plan.sh --output /tmp/new_joint_plan.json
./infer/Collision/start_plan.sh --load-plan /tmp/new_joint_plan.json --preview
```

对真实模型，准备包含真实起终点的人工静态场景请求，再覆盖默认输入：

```bash
./infer/Collision/start_plan.sh \
  --urdf infer/Collision/rm_75.urdf \
  --request /absolute/path/to/rm75_planning_request.json
```

保存路径重放会重新检查模型、原请求 SHA256、起终点和整条路径，不重新搜索。
真实模型的路径重放仍需同时提供原来的 `--urdf`、`--request`；身份或输入变化直接拒绝。
上方请求路径为待准备的文件；不能将演示场景当作真实现场。`--mesh-dir` 和零面积面内存清理已由脚本配置。
真实模型若起点自碰撞会返回 `invalid_start`，报告具体碰撞体对，不会自动增加排除。
覆写 `COLLISION_PLANNER_BIN` 可选用其他已构建的程序，`COLLISION_DEPS_PREFIX` 与监测脚本相同。
原生 CLI 参数为 `--urdf`、`--request`、`--mesh-dir`、`--drop-zero-area-triangles`、`--output`、`--load-plan`、`--preview` 和 `--help`。

## 请求和结果

请求格式见 [planning.demo.json](config/planning.demo.json)，复用已有人工场景字段：

- `scene`：身份、标定身份、`base_link`、采集时间、已完整测量的 `known_space` 及 box／sphere／cylinder 障碍物。
  缺少已知自由区域会被拒绝，不能拿人工自由区域覆盖相机未知空间。
- `joint_positions_rad`、`goal_joint_positions_rad`：完整的命名关节角，单位 rad；检查限位，不夹紧、不回绕。
  当前规划器支持有界转动关节，RM75 为七维；continuous／prismatic 不进入本规划器。
- `now_ns`：与场景采集时间同域的离线快照查询时刻；合成示例显式使用 0。
- `options`：复用碰撞检查的 `clearance_m`、`numerical_guard_m`、`max_scene_age_ns`、
  `max_state_checks`、`max_subdivision_depth`。后两个限制每次路径段检查内部的工作量。
- `self_collision_exclusions`：仅显式、经核对并带原因的自碰撞连杆对；不影响环境碰撞。

| `planning_options` 字段 | 默认 | 作用 |
| --- | --- | --- |
| `timeout_s` | 2 | 总计算时间预算，包含起终点、搜索、简化和最终复查；大于 0 且不超过 300 s |
| `range` | 0.3 | RRTConnect 每次扩展的关节空间欧氏距离，rad |
| `max_queries` | 20000 | 全程状态／路径段查询总预算，范围 1～1000000 |
| `shortcut_attempts` | 50 | 有检查约束的确定性路径简化尝试次数，0～10000 |
| `seed` | 42 | 采样器及 RRT 的局部随机种子，1～uint32 最大值；不修改进程全局种子 |

演示配置将时间设为 3 s、扩展距离设为 0.4。随机种子用于复现实验，不承诺不同版本／机器／时间预算下产生完全相同路径。
时间预算在查询之间及路径段细分时检查；单次 FCL 查询不可抢占，不能将此预算当作硬实时期限。

结果为单个 JSON，包含 `status`、`detail`、模型／场景／标定身份、采集／检查时间、
随机种子、起终点诊断、查询计数、耗时、`joint_order` 和完整命名关节 `waypoints`。
`last_segment_failure` 是最近一次被拒绝的搜索边，成功绕行时也可能存在，不能据此单独判定整条路径失败。

| `status` | 含义 |
| --- | --- |
| `exact_solution` | 精确起终点路径，所有路点与关节线性段已复查 |
| `invalid_start`／`invalid_goal` | 对应端点未通过检查，细节包括碰撞对、未知空间、过期、缺失／越限关节等 |
| `timeout`／`budget_exceeded`／`cancelled` | 计算中断，输出路径为空 |
| `no_exact_solution` | 本次搜索未找到精确路径；不证明数学上无解 |
| `path_validation_failed` | 搜索结果未通过完整复查，不输出可用路径 |
| `invalid_input` | 模型身份、规划参数或关节类型等不符合要求 |

退出码 0 表示精确离线路径，2 表示规划拒绝，1 表示 CLI／文件／模型加载错误。
报告始终保持 `robot_motion_enabled=false`、`ready_for_execution=false`、`trajectory_time_parameterized=false`。

## 构建与验证

需要 OMPL ≥ 1.5、Boost 头文件，以及已有 FCL／Qhull／Eigen 等依赖。
本机将 Ubuntu Jammy 官方 OMPL 1.5.2、Boost 1.74 和 ODE 包解压到 `/tmp/uspilot-collision-deps/root`，未修改系统安装。
与 RealSense SDK 无关，不构建 SDK。本机已有完整监测构建时：

```bash
cmake -S infer/Collision -B infer/Collision/build-monitor -DBUILD_COLLISION_PLANNING=ON
cmake --build infer/Collision/build-monitor -j2
```

首次独立构建规划器（不编译监测及 SDK 接入）：

```bash
cmake -S infer/Collision -B infer/Collision/build-plan \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_COLLISION_CHECKING=ON -DBUILD_COLLISION_PLANNING=ON \
  -DBUILD_MONITOR_VIEWER=OFF -DCMAKE_PREFIX_PATH=/tmp/uspilot-collision-deps/root/usr
cmake --build infer/Collision/build-plan -j2
COLLISION_PLANNER_BIN="$PWD/infer/Collision/build-plan/collision_plan" ./infer/Collision/start_plan.sh
```

预览需要 `BUILD_MONITOR_VIEWER=ON`、GLFW ≥ 3.3 和 OpenGL，不要求实时相机源开启。
临时依赖目录删除后需要自行恢复依赖前缀；这些构建和运行库不纳入 Git。

## 当前边界

独立 CLI 仍只接受人工静态场景；监测入口已将实时／JSONL 回放帧生成的不可变 `CollisionScene` 和反馈关节角直接交给同一个规划库。
离线检查时间固定为请求的 `now_ns`，报告明确标注 `clock_mode=offline_snapshot`；新鲜性检查不会被解释为实时执行许可。
监测入口已支持颈动脉点的探头 TCP 目标和数值 IK；独立 `collision_plan` 仍输入关节目标。
生产状态桥接、时间参数化、Robot 执行及停止监督未实现。
先验证真实模型的自碰撞、单相机自由空间覆盖和起点有效性，再接录制场景、IK 与执行。

算法接口依据 [OMPL MotionValidator](https://ompl.kavrakilab.org/classompl_1_1base_1_1MotionValidator.html) 和
[RRTConnect](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html)；路径段复用本项目的保守位移界检查，不采用默认离散端点采样。
