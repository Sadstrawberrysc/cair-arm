# RM75 七轴系统迁移进度

更新日期：2026-07-21

本文是 RM75 机械臂、Haptron 六维力传感器、标定、接触估计、力控和 Redis
联调的唯一主进度文档。筛选后的关键试验数据保留在 `infer/Robot/build/logs/` 的 CSV
和 `.summary.json` 中，本文只记录当前可信结论、阻塞项和下一步。

## 当前结论

项目已经完成“机械臂与传感器分别可用”以及“七轴集成控制软件可运行”两个阶段，
当前处于 **低速真机力控调试** 阶段。

当前不是通信、构建或实时性问题，而是标定在不同姿态下的泛化不足。2026-07-20
使用 v3 的 5 秒 observe 检测到稳定残差，补偿后合力最大 `4.240 N`、合力矩最大
`0.1111 N·m`，因此 provisional tare 在发送 ServoJ 前拒绝启动。随后完成 v5、v6
重新采集；v6 的拟合力 RMS/最大残差改善为 `0.351/0.606 N`，力矩 RMS/最大残差为
`0.0112/0.0176 N·m`。按当前阶段决策，裸 `--execute` 已切换到 v6 provisional，
但仍保留 `3 N / 0.1 N·m` 的姿态现场 tare 门。

最短技术路径为：

```text
回到已验证姿态并复查悬空残差
  → 验证超过 -3 N 后轴向导纳自然反向卸载
  → 重新完成正式多姿态标定
  → 验收接触点定位
  → 按旧六轴参数验收 -3 N 基线力控与连续 Tool-X 扫描
  → 接入 Redis、Y/RZ 修正和完整扫描状态机
  → 在可复现基线上优化速度、增益和保护阈值
```

## 技术路径完成情况

| 技术阶段 | 状态 | 当前结论 |
| --- | --- | --- |
| RM75 TCP 通信、状态读取、MoveJ | 已完成 | 真机通信和只读状态通过，七关节与 TCP 位姿可稳定读取 |
| FK、6×7 Jacobian、数值 IK | 已完成 | 已迁移到 RM75 七轴模型 |
| ServoJ、J1–J7 限位、速度/加速度、奇异检查 | 已完成基线 | 独立精度测试和集成真机发送均已完成；特定位姿仍可能触发奇异保护 |
| Haptron 六维力传感器 Modbus RTU | 已完成 | `115200/8N1`、从站 1、输入寄存器 `0x0038`；1507 帧连续读取无协议错误 |
| 20 ms 非阻塞控制架构 | 已完成基线 | 60 秒 observe 为 3000 周期零 missed；真实 I/O 连续 10 分钟验收待完成 |
| RM75 工具链标定与重力补偿 | v6 临时启用，正式未完成 | v6 最大力残差 `0.606 N`，优于 v3 的 `0.714 N`，但仍未达到 `0.5 N` 目标 |
| 启动悬空 tare | 已实现，v6 待真机复查 | v3 在当前姿态为 `4.240 N / 0.1111 N·m`；切换 v6 后仍必须通过 `3 N / 0.1 N·m` 门 |
| 接触点估计 | 软件完成，真机未验收 | 已使用 SVD、残差和显式有效性；静态按压定位误差 `≤3 mm` 待验证 |
| 轴向控制律和接触状态 | 软件完成，真机调试中 | 独立 retreat 已关闭；超过目标力后仅由同一导纳公式自然反向卸载 |
| 旧六轴 `-3 N` 力控参数基线 | 软件已配置，真机待验收 | 已迁移 `0.99 N` 接触阈值、`M=3`、`D=20` 和 `-3 N` 目标；保留 RM75 低速、关节与奇异约束 |
| 旧六轴 wrench/接触点滤波与接触姿态 | 已回退、默认关闭 | 实现代码保留用于离线对照；裸 execute 不再启用，接触点不再产生姿态增量 |
| Redis 兼容与新消息 | 软件完成 | 旧命令和 `sensor_data` 保留，新增 `robot:sensor:v1`；部署端口 `7777` 待联调 |
| Tool-X 扫描 | 稳定力启动门已完成，真机待验收 | 接触后先保持 Z 导纳；`Fz` 连续 `0.5 s` 位于目标 `-3±0.3 N` 才锁存启动 `-Tool X`，无时间和扫描距离终点 |
| Y/RZ 修正与完整扫描闭环 | 未完成 | 等基础力控和 X 扫描真机通过后接入 Redis 阶段命令 |
| 源码收敛与生产构建 | 已完成 | 默认只构建 `main_rm75`；诊断和标定工具按需开启 |

## 当前阻塞项与下一步

### 1. 先恢复可用的悬空残差

最新 observe 日志：

```text
infer/Robot/build/logs/rm75_residual_check.csv
infer/Robot/build/logs/rm75_residual_check.summary.json
```

250 个周期全部有效，通信与实时性正常；补偿后工具系均值为：

```text
F = [-0.239, -3.614, 2.084] N
T = [-0.0510, 0.0506, 0.0821] N·m
|F|max = 4.240 N
|T|max = 0.1111 N·m
```

当前关节姿态约为：

```text
[27.916, 74.579, 34.118, 55.051, -10.795, -78.084, -81.931] deg
```

先使用示教器回到此前能够通过 tare 的已知姿态附近，并确保探头完全悬空、器材安装
状态不变、线缆不拉扯传感器。可参考此前成功运行的姿态：

```text
[10.177, 58.158, 38.486, 90.883, -19.636, -68.778, 31.674] deg
```

切换 v6 后先重新运行 5 秒 observe。只有悬空残差低于 provisional 的
`3 N / 0.1 N·m` 启动门，才继续受限真机测试；不要因更换标定文件而跳过现场检查。

### 2. 验证旧六轴轴向导纳行为

当前默认 profile 不再使用独立 retreat。下一轮验证：

1. 工具 Z 受力达到 `0.99 N` 后进入接触导纳控制，并按 `M=3`、`D=20` 向目标
   `-3 N` 收敛。
2. `Fz` 小于目标幅值（例如 `-1 N`）时请求 `+Tool Z` 前进；接近 `-3 N` 时趋稳；
   超过目标幅值（例如 `-4 N`）时由同一导纳公式请求负向运动。
3. 不再发生 `4 N` 进入 retreat、`3.2 N` 释放或 `5 N` 补偿 wrench 停止；当前边界
   恢复为旧逻辑的 `Fx/Fy=20 N`、`Fz=50 N`、力矩 `5 N·m`，原始量程为
   `50 N / 5 N·m`。

离线控制律验证已得到 `dz(-1N)=+0.01 mm`、`dz(-3N)=0`、
`dz(-4N)=-0.01 mm`，且 `-4 N` 不产生 retreat 请求；真机稳态仍待验证。

### 3. 完成后续正式标定

- 独立测量 `R_arm_tip_from_tool_sensor`、Arm_Tip/Tool 原点平移和
  `probe_tcp_sensor_m`；当前硬件定义为 `Tool=Sensor`，不能再把控制器返回的
  Arm_Tip 姿态直接命名为 Tool 姿态。
- 保持传感器末端器材与最终运行配置完全一致。
- 采集至少 8 个无接触、静止、线缆松弛且方向覆盖充分的姿态。
- 目标：最大力残差 `≤0.5 N`、最大力矩残差 `≤0.05 N·m`。
- 标定 JSON 必须设置正确的设备标识、探头模型摘要和工具链验证状态。

### 4. 依次完成系统验收

1. 已知表面静态按压，接触点误差 `≤3 mm`。
2. 软质仿体低速 `-3 N` 基线闭环，稳态误差 `±0.5 N`。
3. 接触力连续 `0.5 s` 稳定在 `-3±0.3 N` 后，验证锁存启动 `-Tool X`
   持续运动，同时保持 Z 向力控。
4. 真实机器人和传感器连续 10 分钟实时性测试。
5. 30 分钟 observe 和串口、机器人、Redis 故障注入。
6. Redis observe、Redis dry-run、Y/RZ 修正和完整扫描端到端联调。
7. 基线通过后再优化目标力、导纳增益、速度、滤波和回退阈值，每次只改变一组参数。

## 当前硬件与控制配置

### 机械臂与传感器

| 项目 | 当前值 |
| --- | --- |
| RM75 控制器 | `192.168.50.254:8080` |
| USB/串口桥 | FTDI FT231X，USB `0403:6015`，序列号 `DU0DU5LC` |
| 稳定设备路径 | `/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0DU5LC-if00-port0` |
| 串口 | RS485，Modbus RTU，`115200/8N1`，从站地址 1 |
| 读取请求 | `01 04 00 38 00 0C 71 C2` |
| 数据 | 大端 float32：`Fx/Fy/Fz/Tx/Ty/Tz` |

串口必须由单一进程独占；控制器、标定采集器和其他串口工具不能同时运行。

### 当前 provisional 工具链

当前文件为 `infer/Robot/build/rm75_force_calibration_v6_provisional.json`：

```text
controller_pose_frame        = Arm_Tip
control_tool_frame           = Sensor
R_arm_tip_from_tool_sensor   = Rz(30 deg)
translation_sensor_to_tool_m = [0, 0, 0]
probe_tcp_sensor_m           = [0, 0, 0.188]
tool_chain_verified          = false
residuals_verified           = false
```

为兼容 v6 标定 JSON，文件中的旧字段 `sensor_to_tool.rotation_row_major` 暂时仍保存
上述 `R_arm_tip_from_tool_sensor`。控制器已显式解释该兼容字段，不再把它误认为
Sensor 到逻辑 Tool 的旋转；逻辑 Tool 与 Sensor 的轴完全一致。

`0.188 m` TCP 来自旧六轴几何假设，仅用于当前受限调试，不等于正式测量结果。

### 裸 `--execute` 隐式 profile

`./main_rm75 --execute` 当前加载：

| 参数 | 当前值 |
| --- | --- |
| 周期/最长时间 | `20 ms / 无固定时限`；Ctrl+C、终止命令或故障才结束 |
| 目标力 | `-3 N`（复现旧六轴主循环覆盖值） |
| 接近方向/速度 | `+Tool Z / 0.05 cm/s` |
| 接触阈值 | `0.99 N` |
| 轴向导纳参数 | 虚拟质量 `M=3`、虚拟阻尼 `D=20` |
| 坐标链 | 控制器位姿为 `Base→Arm_Tip`；控制增量为 `Tool=Sensor`；通过 `R_arm_tip_from_tool=Rz(30°)` 转为 Arm_Tip 目标 |
| 扫描阶段 | `phase_index=0`；`|Fz|≥2 N` 为粗接触下限，且 `Fz` 必须连续 `0.5 s` 位于 `-3±0.3 N` 才启动 |
| X 扫描 | `-Tool X / 0.2 cm/s / 连续`，同时保持 Z 向力导纳 |
| Tool-X 参考轨迹 | 独立持久笛卡尔参考；IK 使用参考位姿与关节目标 FK 之间的完整误差反馈，不再从漂移 FK 自由累计 |
| 接触后 Z 力控速度上限 | `0.05 cm/s`，在 XYZ 综合限幅前独立应用 |
| XYZ 综合速度上限 | `0.2 cm/s`；X 与 Z 同时运动时共同分配此向量速度 |
| 旧 wrench/接触点滤波 | 关闭；补偿 wrench 与原始接触估计直接进入基础控制链 |
| 接触姿态导纳 | 关闭；接触点不产生 roll/pitch/yaw 姿态增量 |
| 独立过力回退 | 关闭；超过目标力后由轴向导纳自然反向 |
| 未接触接近行程上限 | `250 mm`；接触后的连续 X 扫描不使用累计距离终点 |
| 未接触接近速度/XYZ 综合最大线速度 | `0.05 / 0.2 cm/s` |
| 总姿态行程限制 | 关闭（配置值 `0 deg`） |
| 关节/位置/姿态跟踪误差 | `5 deg / 10 mm / 关闭（0 deg）` |
| 补偿 wrench 边界 | `Fx/Fy=20 N`、`Fz=50 N`、力矩 `5 N·m` |
| 原始 wrench 上限 | `50 N / 5 N·m` |
| 启动流程 | Stop、确认静止、3 秒完全悬空 tare |
| Redis、模型 Y/RZ、额外 pitch 修正 | 关闭；旧接触姿态 roll 也已关闭 |

当前速度分三层：未接触 Z 接近为 `0.05 cm/s`；接触后 Z 导纳先单独限制为
`0.05 cm/s`；Tool-X 扫描请求为 `0.2 cm/s`。最后仍对完整 XYZ 平移向量应用
`maximum_linear_speed_cm_s=0.2` 综合安全限幅。X 与 Z 同时运动且合成速度超限时，
三维向量会按比例缩放，因此两轴不会各自同时达到其独立上限。

Tool-X/Z 增量现在积分到独立 `cartesian_reference_pose`。关节规划仍从当前
`model_pose`（关节目标 FK）出发，但目标始终是未被 IK 残差污染的持久参考位姿，
因此上一周期的笛卡尔误差会在下一周期继续进入 Jacobian 求解。只有规划和发送成功
后才推进参考；Hold 会将参考重置到最后有效 FK。CSV/summary 记录参考与 FK 的位置、
姿态误差及其运行最大值，用于真机验证反馈是否收敛。

`cartesian_reference_pose` 本身是 Arm_Tip 位姿，但 `requested_dx/dy/dz_tool` 始终
定义在真实 Tool/Sensor 坐标系。每周期使用
`R_base_from_tool = R_base_from_arm_tip * R_arm_tip_from_tool` 将控制增量转换到基座系，
再生成 Arm_Tip 目标交给 IK。因此 `-Tool-X` 在当前 `+30°` 安装下会正确转换成
Arm_Tip 平面内的 `[-0.866, -0.500, 0]`，而不再误发为 `-ArmTip-X`。

扫描启动增加了连续稳定力门。进入接触后先保持 Tool-Z 导纳，扫描计时器只在
`|Fz-(-3 N)|≤0.3 N` 且 `|Fz|≥2 N` 时累积；任一周期越界都会重新计时。连续满足
`0.5 s`（20 ms 周期下为 25 个周期）后锁存 Tool-X 扫描。锁存后不会因小幅力波动
反复切换成仅 Z 运动；停止动作、切换扫描 phase 或控制复位后才重新等待稳定力门。

### 旧六轴力相关参数映射

| 旧六轴参数/逻辑 | 旧值 | 当前 RM75 基线 | 处理结论 |
| --- | --- | --- | --- |
| 主循环目标力 | `-3 N` | `-3 N` | 已复现 |
| 接触判定 | `|Fz|>0.99 N` | `|Fz|>=0.99 N` | 已复现 |
| Z 向导纳 | `M=3`、`D=20` | `M=3`、`D=20` | 已复现 |
| 导纳模型周期 | `10 ms`，实际主循环约 `20 ms` | 模型与控制循环统一为 `20 ms` | 修复旧时基不一致 |
| 未接触推进 | 每周期 `3 mm` | `0.05 cm/s` 连续限速 | 不复现旧六轴大步进 |
| Z 向速度夹紧 | `±0.5 m/s` | 接触后先限 `±0.0005 m/s`（`0.05 cm/s`），随后与 X/Y 共用 `0.2 cm/s` XYZ 综合限幅 | 保留低速真机边界 |
| 原始传感器量程检查 | `50 N / 5 N·m` | `50 N / 5 N·m` | 已恢复旧边界 |
| 补偿 wrench 控制边界 | `Fx/Fy=20 N`、`Fz=50 N` | `Fx/Fy=20 N`、`Fz=50 N`、力矩 `5 N·m` | 已取消临时 `5 N` 轴向门 |
| 独立过力回退 | 无 | 关闭 | 已取消 `4.0/3.2 N` retreat 迟滞 |
| 启动零点 | 100 点、每点 `10 ms` | 悬空 `3 s`，约 150 点 | 等效覆盖并增加静止/残差门 |
| wrench Kalman 参数 | `Rk=0.0002`、`Bk=0.000001` | 实现代码保留，当前控制输入直接使用补偿 wrench | 裸 execute 已关闭 |
| 接触点滤波参数 | `Rk=0.0002`、`Bk=0.0000005` | 实现代码保留，当前不参与控制 | 裸 execute 已关闭 |
| 接触姿态导纳 | `Rx/Ry: M=2,D=10`；最终 `Rx×0, Ry×1.5` | 实现代码保留，当前接触点不产生任何姿态修正 | 裸 execute 已关闭 |
| 模型修正 | `Kp_y=0.5`、`Kp_rz=0.05`；`|Fz|>2 N` 才启用 | 配置值已保留，当前隐式 profile 关闭 Y/RZ | 扫描阶段再启用 |

这里的“复现”指保留可对照的参数和实现，而不是重新引入旧程序中已知的时基错误、
无接触大步进、超高速夹紧和直接 `exit`。当前裸 `--execute` 已回退为基础轴向力控；
补偿后的未滤波 wrench 直接进入导纳，不经过旧 Kalman，也不触发独立 retreat。

## 关键验证结果

| 日期/项目 | 结果 | 结论 |
| --- | --- | --- |
| RM75 精度基线 | 位置误差 `0.0087–0.0334 mm`，姿态最大约 `0.073 deg`，横向偏差 `0.19–0.56 mm` | 达到当前运动精度目标 |
| Haptron monitor | 1507 帧/1508 请求；CRC、协议、数值和 I/O 错误均为 0 | 传感器通信稳定 |
| v3 八姿态标定 | 力 RMS/最大 `0.427/0.714 N`；力矩 RMS/最大 `0.0121/0.0186 N·m` | 可临时调试，未达到正式力残差目标 |
| v6 八姿态标定 | 力 RMS/最大 `0.351/0.606 N`；力矩 RMS/最大 `0.0112/0.0176 N·m` | 当前 provisional；优于 v3，但未达到正式 `0.5 N` 门 |
| 60 秒 observe | 3000 周期，missed 0，全部周期 `≤22 ms`，悬空合力均值/最大 `0.432/0.560 N` | 该姿态下补偿和实时性良好 |
| 20 秒受力/回退运行 | 1000 周期，missed 0，`result=completed`；208 approach、233 contact、559 retreat | 已证明能进入回退状态，但未证明卸载方向正确 |
| 2026-07-20 当前姿态 observe | 250 周期，missed 0，最大工作时间 `286.311 us`；合力/合力矩最大 `4.240 N / 0.1111 N·m` | 通信和周期正常，标定残差阻塞 execute |
| v6 60 秒力控诊断 | 3000 周期、missed 0；132 approach、2058 contact、810 retreat | 小力时规划方向确为 `+Tool Z`，但旧 `0.5 cm/s` 正常力控速度使 Fz 在约 80 ms 内从 `-0.85 N` 越过目标到 `-1.27 N`，随后反向卸载；已将正常力控限速降到 `0.05 cm/s` |
| Tool-X/Z 综合限幅离线回放 | X 扫描和 Z 导纳可同时输出；合成 XYZ 步长不超过 `0.2 cm/s × 20 ms = 0.04 mm` | X/Z 共同参与三维向量归一化限幅；真机待验收 |
| Tool-X 稳定力启动门 | `-3±0.3 N` 前 24 个连续周期无 X，第 25 周期启动；中途越界会清零计时；启动后力波动不停止 X | 离线确定性时序验证通过，真机待验收 |
| Arm_Tip/Tool 30° 坐标变换 | `-Tool-X 0.04 mm` 正确转换为 Arm_Tip `[-0.034641,-0.020000] mm`；Tool 请求仍保持 `[-0.04,0,0] mm` | 离线方向验证通过，真机待验收 |

历史调试中曾因接触迟滞和方向问题达到约 `-20 N`；随后使用过 `5 N` 轴向门以及
`4.0/3.2 N` retreat 迟滞。按 2026-07-21 当前阶段决策，这三项已从默认 profile
关闭，恢复旧六轴单一导纳逻辑；历史日志不替代当前参数下的正式稳态验收。

## 构建与程序入口

### 默认生产构建

日常使用只需要以下最简命令，不启用并行编译，也不需要附加其他参数：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build
cmake --build infer/Robot/build
```

编译完成后运行：

```bash
./infer/Robot/build/main_rm75 --execute
```

默认只生成生产入口 `main_rm75`。

### 包含诊断和标定工具

```bash
cmake -S infer/Robot -B infer/Robot/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build -j
```

| 可执行文件 | 用途 | 是否运动 |
| --- | --- | --- |
| `main_rm75` | 集成控制器 | 只有显式 `--execute` 才运动 |
| `arm_read_state` | 读取七轴状态与 TCP 位姿 | 否 |
| `arm_preset_pose` | MoveJ 到命名关节姿态 | 只有显式 `--execute` |
| `rm75_servoj_diagnostic` | 七轴 ServoJ/精度诊断 | 只有显式 `--execute` |
| `force_sensor_calibration_capture` | 同步采集姿态和原始 wrench | 否 |
| `force_sensor_calibrate` | 离线拟合标定 JSON | 否 |

原项目的 `src/main.cpp`、`src/main_nomove.cpp`、`src/robot_control.cpp` 等文件继续保留；
旧六轴入口不参与 RM75 生产构建。

## 推荐操作命令

后续命令从构建目录运行：

```bash
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
```

### 1. 只读检查当前残差

```bash
./main_rm75 \
  --robot-ip 192.168.50.254 \
  --robot-port 8080 \
  --sensor-device /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0DU5LC-if00-port0 \
  --sensor-baud 115200 \
  --calibration rm75_force_calibration_v6_provisional.json \
  --expected-sensor-id DU0DU5LC \
  --probe-model ../model/Lprobe-IFS.STL \
  --observe \
  --no-redis \
  --duration-sec 5 \
  --runtime-log logs/rm75_residual_check.csv
```

该命令不会发送运动指令。先检查日志中的补偿 wrench；当前阶段不要在残差超门时运行
`--execute`。

### 2. 重新采集正式标定

```bash
./force_sensor_calibration_capture \
  --robot-ip 192.168.50.254 \
  --robot-port 8080 \
  --sensor-device /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0DU5LC-if00-port0 \
  --sensor-baud 115200 \
  --sensor-to-tool-rpy-deg RX,RY,RZ \
  --poses 8 \
  --samples-per-pose 50 \
  --min-window-duration-ms 1500 \
  --output calibration_samples_v4.csv
```

每个姿态都必须停止运动、完全悬空并保持最终器材安装状态。之后运行：

```bash
./force_sensor_calibrate \
  --input calibration_samples_v4.csv \
  --output rm75_force_calibration_v4.json \
  --sensor-id DU0DU5LC \
  --probe-model Lprobe-IFS.STL \
  --probe-file ../model/Lprobe-IFS.STL \
  --sensor-to-tool-rpy-deg RX,RY,RZ \
  --sensor-to-tool-translation-m X,Y,Z \
  --probe-tcp-sensor-m X,Y,Z \
  --tool-chain-verified
```

### 3. 真机控制

仅当悬空 tare 通过、工具 Z 方向已确认、软质仿体和急停均就绪时运行：

```bash
./main_rm75 --execute
```

当前裸 execute 使用未滤波补偿 wrench 的基础轴向力控；接触后先等待 `Fz` 连续
`0.5 s` 稳定在 `-3±0.3 N`，再锁存并沿 `-Tool X` 持续运动，Z 向 `-3 N` 力控
继续运行。默认无时间和 X 扫描距离终点，
直到 Ctrl+C、终止命令或故障才结束；`250 mm` 只限制未接触接近阶段。不启用旧
wrench Kalman、接触点 Kalman、旧接触姿态导纳或独立 retreat；总姿态行程和姿态
跟踪误差检查保持关闭。关节规划、奇异、关节/位置跟踪误差、通信故障和旧量程边界
仍保留。附加任何其他参数后都不会自动套用上述隐式 profile，应使用完整显式配置。

## 完成标准

- 标定补偿最大残差：力 `≤0.5 N`，力矩 `≤0.05 N·m`。
- 接触点定位误差：`≤3 mm`。
- 20 ms 控制循环连续 10 分钟：至少 `99.9%` 周期 `≤22 ms`，无连续超期，任何周期 `≤40 ms`。
- 运动精度不低于现有基线：横向偏差 `≤0.6 mm`，姿态误差 `≤0.1 deg`。
- `-3 N` 六轴参数基线稳态误差：`±0.5 N`，且无超力、限位、奇异或连续 deadline miss。
- 传感器陈旧/断线、机器人超时、Redis 中断、无效命令、限位和奇异均能进入 Hold/Stop。
- 所有真实运动保存 CSV、摘要和版本化标定信息。

## 源码结构

| 路径 | 职责 |
| --- | --- |
| `infer/Robot/src/main_rm75.cpp` | 唯一生产入口：配置、启动门、20 ms 主循环和收尾摘要；生成 `main_rm75` 可执行文件 |
| `infer/Robot/src/redis_bridge.cpp` | Redis 命令/状态/传感器异步 I/O；不进入控制截止路径 |
| `infer/Robot/src/realman_command.cpp` | RM75 TCP、状态和命令通信 |
| `infer/Robot/src/realman_kinematics.cpp` | RM75 七轴运动学 |
| `infer/Robot/src/rm75_control.cpp`、`include/rm75_control.hpp` | 轴向/6D 控制状态机、ServoJ 规划和约束；原独立 planner 头已合并 |
| `infer/Robot/src/force_sensor.cpp` | 串口、Haptron Modbus、快照、标定与补偿 |
| `infer/Robot/src/contact_sensing.cpp` | STL 接触点估计 |
| `infer/Robot/src/force_sensor_calibration_capture.cpp` | 同步标定采集入口和窗口实现 |
| `infer/Robot/src/force_sensor_calibrate.cpp` | 离线标定拟合入口 |
| `infer/Robot/config/` | 标定模板和模拟配置 |

按原项目的职责边界，Modbus 与标定补偿归入传感器模块，控制律与 ServoJ planner
归入 `rm75_control`，生产侧保持单一入口。Redis 因包含阻塞网络 I/O 和独立线程，已从
3187 行的入口尾部抽回 `redis_bridge.cpp`；这只拆分源码职责，不增加运行程序。用户原有
六轴源码未删除，诊断与标定入口仍由 CMake 选项控制。

2026-07-20 已清理历史构建与试验副本：删除 v3–v5 采集、失败 `.partial`、旧真机
试运行日志、嵌套 `force_haptron/build`、Python 缓存和旧 CMake 目标。当前 build 从
约 `165 MB` 收敛到约 `11 MB`，保留 v6 标定、通信/残差证据、一次成功轴向力控日志
和最新故障日志。

重构前曾有 20 项离线测试记录；按此前“清理测试文件”的要求，后来新增的
`infer/Robot/tests/` 已删除。当前回归依据是 Release 构建、simulate、observe 和真机
日志。后续进入长期维护前，应重新建立最小 CI 回放测试，至少覆盖 Modbus 解析、标定
补偿、控制状态机和 execute 安全门。

## 运行边界

- `observe`、`simulate` 和 `dry-run-control` 不发送运动命令；真实运动必须显式使用 `--execute`。
- v6 provisional 不能用于生产、人体实验或正式闭环验收。
- 当前残差门失败时不要放宽阈值，应检查姿态、器材、线缆或重新标定。
- 每次真机测试前确认工具 Z、力符号、接近/回退方向和物理急停。
- `--allow-near-singularity` 只能用于 dry-run，execute 会拒绝。
- 不要同时使用示教器、其他 JSON 客户端和本程序发送运动命令。

## 基础流程完成后的优化

- 重建确定性录制/回放和最小 CI 测试。
- 增加标定独立验证集、姿态覆盖度和条件数记录。
- 增加硬件身份自动核验和 commissioning manifest。
- 增加 Redis `session_id/run_nonce` 与更严格的命令序号。
- 在验证方向和卸载趋势后设计分级自动撤离 profile。
