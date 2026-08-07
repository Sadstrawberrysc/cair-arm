# RM75 当前进度与运行边界

更新日期：2026-08-05

本文记录当前 RM75 视觉力控闭环的真实配置、已完成能力和剩余验收项。源码学习请配合
[RM75 学习路径](rm75_learning_path.md)，启动顺序见
[RM75 构建与启动命令](rm75_build_and_start_commands.md)。

## 当前系统结论

RM75 已具备一条可运行的闭环链路：超声视觉程序通过 Redis 发布 Y、RZ、phase 与 action；
`main_rm75` 在 10 ms 周期内读取 RM75 与 Haptron 状态，经补偿、力控状态机、七轴数值 IK
和 ServoJ 驱动机器人。当前属于**临时标定下的工程调试系统**，不是已经完成全量计量验收的
生产系统。

当前正式运行入口没有额外命令行参数：

```bash
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./main_rm75
```

运行前必须确认探头悬空、线缆不受力、急停可用，并先启动本机 Redis。

## 启动链路与当前配置

```text
超声图像
  → main_redis_seg_newphase_recovery_mode.py
  → Redis 127.0.0.1:7777
  → main_rm75（10 ms）
  → rm75_control（状态机、Tool-X/Y/Z/RZ 请求）
  → 七轴数值 IK / ServoJ
  → RM75

Haptron Modbus → 标定补偿 wrench → 接触点估计 → rm75_control
RM75 实际关节与 TCP 状态 ────────────────────────────────┘
```

| 项目 | 当前值 / 说明 |
| --- | --- |
| RM75 | `192.168.50.254:8080` |
| Haptron 串口 | `/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0DU5LC-if00-port0`，`115200/8N1` |
| Redis | `127.0.0.1:7777`；视觉端每 200 ms 发布心跳，命令陈旧门为 500 ms |
| phase 视觉模型 | `intergrate_infer/infer_realtime.py` 的单帧 ConvNeXt-Base；权重 `weights/runs_ConvNeXtBase_single.pth`；五分类 `0/1/2/3/4` 映射为 RM75 `0/0/0/1/2` |
| 控制周期 | 10 ms |
| 标定文件 | `build/rm75_force_calibration_v9_provisional.json`，当前运行时临时标定；力最大拟合残差 `1.745 N`，未通过正式 `0.6 N` 门限 |
| 最新采样 | `build/calibration_samples_v9.csv`，12姿态，已按用户要求强制生成 provisional JSON；`residuals_verified=false` |
| Tool/TCP | Tool 与传感器坐标重合；当前 TCP 为 `0,0,0.188 m`；Arm_Tip 与 Tool 的固定旋转来自标定 |
| 启动 tare | 3 s 悬空 tare；静止跨度门：关节 `0.02°`、TCP `0.35 mm`、姿态 `0.10°` |
| provisional 残差门 | 未施加 tare 前合力/合力矩不超过 `5 N / 0.5 N·m` |

`./main_rm75` 会先停止遗留运动并确认静止，再执行 tare，随后等待视觉端的新 session 的
idle 握手。只有视觉端发布 fresh 的 moving 命令才开始接近。Redis 命令失效、断线或 idle 时
进入 Hold；Hold 不退出程序、不继续累计视觉位移。

## 参数归属

控制参数不再在 `main_rm75.cpp` 中重复覆盖。调参时以如下结构为准：

| 文件 / 结构 | 负责内容 |
| --- | --- |
| `include/rm75_control.hpp` 的 `Rm75ControlConfig` | 接触、导纳、卸载、Kalman、Tool-X/Y/Z/RZ、状态机和坐标链默认参数 |
| 同文件的 `Rm75ServoPlannerConfig` | IK 阻尼、关节速度比例与 ServoJ 发送节奏 |
| 同文件的 `Rm75RuntimeSafetyConfig` | 原始 wrench、跟踪误差等运行门限 |
| `src/rm75_control.cpp` | 上述参数对应的控制律、滤波、状态转换和目标位姿增量合成 |
| `src/main_rm75.cpp` | 固定启动流程、通信/线程、标定读取、坐标链注入、实际反馈与 CSV/summary 日志 |

`main_rm75.cpp` 中的 `Options` 和 implicit-profile 保留用于启动契约与兼容校验；常规算法调参
不应修改那里，而应修改 `rm75_control.hpp` 的对应字段。

### 当前关键控制值

| 模块 | 当前值 |
| --- | --- |
| 目标轴向力 | `Fz = -2 N` |
| 接触门 | `|Fz| >= 0.99 N` |
| Tool-Z 导纳 | 虚拟质量 `M=3`，阻尼 `D=20`，普通接触速度上限 `0.2 cm/s` |
| 悬空接近 | `+Tool-Z`，`2 cm/s` |
| 超力卸载 | 相对目标额外压入 `1.0 N` 且持续 `0.50 s` 才卸载（当前低于 `-3 N`）；最大 `-Tool-Z` 速度 `1 cm/s`；卸载/制动/恢复期间禁用 X/Y/RZ |
| Tool-Y | 接近/force_settle 增益 `1.0 s⁻¹`；Scan 缩放 `0.5`、rotate-align 缩放 `0.3`，单周期不超过 `0.5 mm`；视觉死区 `±0.15 mm`、无 EMA、换向确认 3 帧；`|Fz|>1.5 N` 时允许 |
| Tool-X | 力连续 `0.5 s` 位于 `-2±0.8 N` 后，沿 `-Tool-X` 连续扫描 `1 cm/s` |
| Tool-RZ | Trigger 后按有符号视觉 RZ 修正，最大 `5°/s`，累计额度 `140°`；Action/phase=2 丢失血管时 RZ 归零，不再用于恢复搜索 |
| 旋转丢失恢复 | phase=2 丢失血管后 Tool-X/RZ 均归零，冻结 Roll/Pitch；锁存丢失前掩膜侧别并以左侧 `+Tool-Y`、右侧 `-Tool-Y`、固定 `0.002 m/s` 恢复，Tool-Z 继续力控；重新连续检测到血管 5 帧后退出 |
| wrench 硬门 | 补偿后 Fx/Fy/Fz `50 N`，力矩 `5 N·m`；原始数据门同为 `50 N / 5 N·m` |
| 跟踪门 | 关节 `5°`，位置 `25 mm`，姿态门 `0` 表示关闭 |

## 已完成与待验收

| 能力 | 状态 | 当前说明 |
| --- | --- | --- |
| RM75 TCP、状态读取、ServoJ、Stop | 已实现 | 已有独立通信层和实际状态反馈；仍需在完整扫描中持续观察通信稳定性。 |
| 七轴 FK、6×7 Jacobian、数值 IK | 已实现 | IK 对 Arm-Tip 目标求解，ServoJ 下发七关节目标；奇异、限位与通信失败会中止。 |
| Haptron Modbus 与补偿 wrench | 已实现，正式标定待完成 | 当前使用 v9 provisional；保留运行时 tare 与安全门，但 v9 力残差未通过正式验收。 |
| 启动悬空 tare | 已实现 | 可消除当次悬空偏置，但不能代替多姿态重力、质量和质心标定。 |
| 接触点估计 | 已实现，真值验收待完成 | 由补偿 wrench 与探头 STL 求解；需要在已知表面上量化静态定位误差。 |
| wrench Kalman | 已开启 | 6 个 wrench 分量各自采用一维随机游走滤波；未滤波补偿 wrench 仍先经过硬门。 |
| 接触点 Kalman | 已开启 | 仅在有效接触点时更新；无接触时重置，避免把无接触的零点混入下一次真实接触。 |
| 接触姿态调整 | 已开启 | 过滤后的接触点驱动 Tool-X/Roll；Pitch 接线同样使用过滤结果，但默认增益为 0，因此当前不产生 Pitch 修正。 |
| Z 力控、超力卸载与恢复 | 已实现，需持续调参验收 | 超力时独立于 Redis Hold 也可纯轴向卸载；idle 下制动结束后不自动重新接近。 |
| Tool-Y 居中 | 已实现，真机量化待完成 | 视觉误差转比例速度；有接触、fresh moving、非卸载/恢复且实际 TCP 可跟上时生效。 |
| Tool-X 扫描与 Trigger/RZ | 已实现，端到端验收待完成 | `b` 接近和 Y；`m` 请求 scan phase；视觉稳定 phase=1 后进入 Trigger/RZ。 |
| 旋转阶段血管丢失恢复 | 已接入，真机待验收 | phase=2 且 `recovery_mode=true` 时 Tool-X/RZ 均为零，Tool-Y 按锁存侧别以 `2 mm/s` 恢复，保留 Tool-Z 力控。 |
| Redis session、sequence、Hold、状态发布 | 已实现 | 新 session 先 idle，序号只增不重复累计；过期命令进入 Hold。 |
| CSV/summary 运行记录 | 已实现 | 包括 state、视觉命令、Tool 增量、力、接触、卸载/恢复、跟踪门和时序。 |
| 自动测试 / 回放验证 | 未完成 | 当前 CTest 没有已注册测试；需建立 Redis/状态机回放与离线控制测试。 |

## 当前状态机

```text
Armed / Redis Hold
  └─ fresh moving（按 b）→ Approach：+Tool-Z 接近，允许 Tool-Y
      └─ 接触 → Force settle：Z 导纳，等待扫描稳定力带
          └─ fresh scan phase（按 m）且稳定 → Scan：-Tool-X + Tool-Y + Z 力控
              └─ phase=1 → Trigger / Rotate align：停止 X，保留 Y，执行有符号 RZ
                  └─ phase=2 且丢失血管 → X/RZ归零，锁存左右侧并固定速度移动Y，保留Z力控

任意阶段：超力 → Unload → Brake →
  moving 时可 Reacquire / 恢复 Z 导纳；idle 时回 Armed
terminate / t → Armed；Ctrl+C 或真实故障 → Stop 并退出
```

无接触、超力卸载、制动和恢复均会压制横向/旋转，因而 X 或 Y 的“停顿”首先应结合 CSV 中的
`state`、`target_force_unloading`、`target_force_recovering` 与 `command_fresh` 判断，而不是先归因于
10 ms 周期。

## 日志与故障定位

每次运行将产生一对文件：

```text
infer/Robot/build/logs/*.csv
infer/Robot/build/logs/*.summary.json
```

优先检查：

| 项目 | 用途 |
| --- | --- |
| `result`、`fault_code`、`completion_reason` | 确认是正常退出、视觉 terminate、Ctrl+C 还是故障。 |
| `requested_dx/dy/dz_tool_m` | 控制器是否真的请求了 Tool-X/Y/Z。 |
| `control_fz_n`、卸载/恢复周期 | 判断 Z 导纳、超力卸载和恢复是否运行。 |
| `command_model_y_m`、`visual_y_tracking_paused` | 判断视觉 Y 输入、Y 积分暂停与参考重置。 |
| `command_recovery_mode`、`recovery_search_active`、`recovery_locked_mask_side`、`recovery_tool_y_velocity_m_s` | 判断视觉是否请求恢复、锁存的左右侧及固定 Tool-Y 恢复方向；该状态下 `requested_dx/drz` 应均为零。 |
| `actual_*`、模型误差、`robot_state_*` | 判断实际 TCP 是否跟上，以及是否为通信状态陈旧。 |
| `planned_scan_distance_mm`、phase/state | 判断是否进入扫描及累计距离。 |

当前程序会因以下情况进入 Stop/Fault：机器人或传感器通信/数据异常、原始或补偿 wrench 超硬门、
关节限位、IK/奇异或 ServoJ 错误、全局跟踪误差、控制周期无法持续调度，以及 Ctrl+C。Redis
超时本身进入 Hold；若仍有超力，独立卸载层可以继续 `-Tool-Z` 卸载。

## 下一步验收优先级

1. 核对并独立实测 R/t/TCP，重新采集低外力多姿态样本，将力最大残差降到 `0.6 N` 内，再替换当前 v9 provisional 并完成多姿态悬空验收。
2. 在已知表面测量接触点估计误差，并对比 Kalman 开/关的接触点与 Roll 轨迹。
3. 在软质仿体上单独验收：`-3 N` 稳态、超力卸载、制动、idle 不重接近与重新接触。
4. 依次验收 Tool-Y 正负方向、Tool-X 连续扫描、phase=1 的 RZ 符号与 140° 额度，以及旋转丢失恢复期间 Tool-X/RZ 均保持为零。
5. 建立 Redis JSON/session/超时和状态机的自动回放测试，再进行长时间连续真机稳定性验证。

每次只改变一个参数或门控条件；保留对应 CSV/summary 后再进入下一项。不要在
`main_rm75` 运行期间清理 `build/logs/`。
