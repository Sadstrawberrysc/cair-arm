# RM75 当前进度与运行边界

更新日期：2026-08-20

本文记录当前工作树中 RM75 视觉力控闭环的实际实现、有效配置、已知分叉和待验收项。
架构边界见 [ARCHITECTURE.md](ARCHITECTURE.md)，启动顺序见
[RM75 构建与启动命令](docs/rm75_build_and_start_commands.md)，源码阅读路径见
[RM75 学习路径](docs/rm75_learning_path.md)。

## 当前结论

系统已形成可运行的闭环链路：视觉端通过 Redis v1 发布 Tool-Y、RZ、phase、action 和
recovery 意图；`main_rm75` 在 10 ms 周期内融合 RM75 与 Haptron 快照，依次完成原始量程
检查、标定/tare 补偿、接触估计、状态机、七轴数值 IK、安全规划和异步 ServoJ 下发。

当前仍是**使用 provisional 标定的工程调试系统**，不是完成计量与端到端验收的生产系统。
自动测试/回放 harness 尚未建立，CTest 当前没有已注册测试。

### P0：配置存在两套来源且实际未完全一致

当前无参数入口的 `ApplyImplicitCommissioningProfile()` 仍以 `-3 N`、`1 cm/s` 接近和
`rm75_v6_redis_visual_closed_loop_3n_tcp188` 为校验/日志 profile；但 `main_rm75` 创建
`Rm75ControlConfig` 后只注入周期和标定坐标链，没有把这些 `Options` 中的力控、扫描和跟踪
值写入控制结构。

因此按当前源码，真正进入 `Rm75ControlLaw`/summary 的值来自 `rm75_control.hpp`，包括
`-2 N` 目标力和 `2 cm/s` 接近速度。profile 名称、启动约束错误信息以及部分旧文档仍描述
`-3 N / 1 cm/s`。在统一参数所有权并完成 dry-run/真机复核前，不得根据 profile 名称推断
实际控制值；每次运行必须以启动输出和 `.summary.json` 的 `control` 字段确认。

## 在线链路

```text
超声图像
  → intergrate_infer/main_redis_seg_newphase_recovery_mode.py
  → Redis 127.0.0.1:7777（command/status/sensor v1）
  → main_rm75（10 ms，只消费内存快照）
  → Rm75ControlLaw（状态机与 Tool-X/Y/Z/RZ 增量）
  → Rm75ServoPlanner（6×7 Jacobian、IK、限位/奇异/速度检查）
  → RMCommand ServoJ mailbox
  → RM75

Haptron Modbus RTU → ForceSensorReader → 标定与 runtime tare → 接触点估计 ─┘
RM75 TCP 状态线程 ────────────────────────────────────────────────────┘
```

| 项目 | 当前实现 |
| --- | --- |
| RM75 | `192.168.50.254:8080`，JSON-over-TCP，七关节状态与 ServoJ |
| Haptron | `/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0DU5LC-if00-port0`，`115200/8N1`，Modbus RTU |
| Redis | `127.0.0.1:7777`；命令 fresh 门 500 ms，视觉心跳 200 ms |
| 控制周期 | 10 ms；连续两次超期或任一周期超过 40 ms 触发 deadline fault |
| 标定 | `infer/Robot/build/rm75_force_calibration_v9_provisional.json` |
| 探头模型 | `infer/Robot/model/Lprobe-IFS.STL`，启动时核对 SHA-256 |
| Tool/TCP | Tool 与 Sensor 原点重合；Probe TCP 为 sensor 原点 `+Z 0.188 m`；Arm_Tip→Tool 固定旋转来自标定 |
| 日志 | 控制周期记录由后台线程写 CSV，结束时生成 schema v2 summary JSON |

无参数 `./main_rm75` 是真机 execute 模式：先停止遗留运动并确认静止，完成 3 s 悬空 tare，
再等待新 Redis session 的 idle 握手和 fresh moving 命令。断线、过期/回放命令、反馈短暂陈旧
或 ServoJ mailbox 暂时阻塞进入 Hold；可恢复 Hold 释放前必须收到更新的生产者命令。

## 当前实际生效配置

以下值取自当前 `Rm75ControlConfig`、`Rm75ServoPlannerConfig` 和
`Rm75RuntimeSafetyConfig`。它们是控制对象和 summary 的实际来源，不是无参数 profile 中
尚未注入的 `Options` 值。

| 模块 | 当前实际值 |
| --- | --- |
| 目标轴向力 | `Fz = -2 N` |
| 接触门 | `|Fz| >= 0.99 N`，接触后锁存至 idle/reset |
| Tool-Z 导纳 | `M=3`、`D=20`；普通速度上限 `0.002 m/s`；加速度上限 `0.01 m/s²` |
| 悬空接近 | `+Tool-Z`，`0.020 m/s` |
| 超力卸载 | 相对目标额外压入 `1.0 N` 且持续 `0.50 s`；最大卸载 `0.010 m/s`；重新接近 `0.005 m/s` |
| Tool-Y | 比例增益 `1.0 s⁻¹`，方向 `-1`；单周期 `0.5 mm`；Scan 缩放 `0.8`，Rotate 缩放 `0.3` |
| Tool-Y 门 | `|Fz| > 1.2 N`；实际 TCP 落后 `20 mm` 暂停、回到 `10 mm` 恢复；Y 主导且达到 `5 mm` 可重建参考 |
| Tool-X | 力位于 `-2±0.8 N` 且连续 `0.5 s` 后沿 `-Tool-X` 以 `0.010 m/s` 连续扫描 |
| Tool-RZ | 增益 `0.05`，最大 `5°/s`，Trigger 后累计执行额度 `120°` |
| 接触姿态 | wrench/contact Kalman 开启；接触点驱动 Roll；Pitch 增益为 `0` |
| 丢失恢复 | phase 2 中 X/RZ=0、冻结 Roll/Pitch；固定 Tool-Y `0.0002 m/s`，单次最多 `20 mm` |
| wrench 硬门 | 原始与补偿力 `50 N`，力矩 `5 N·m`；硬门在滤波前检查 |
| 规划 | 10 ms、阻尼 `0.001`、关节限位停止裕量 `3°`、最小任务奇异值 `1e-5`、关节加速度 `90°/s²` |
| 跟踪 fault | 关节 `20°`、位置 `25 mm`；姿态门 `0` 表示关闭独立姿态跟踪 fault |

### 当前视觉端配置

| 模块 | 当前值 |
| --- | --- |
| phase 模型 | `weights/runs_ConvNeXtBase_single.pth`；原始五分类映射到 Scan/Trigger/Action |
| phase debounce | Scan `8` 帧且概率 `>=0.60`；Trigger/Action 各 `3` 帧且概率 `>=0.55` |
| Tool-Y 调理 | 死区 `0.00001 m`（0.01 mm）；EMA 当前帧权重 `0.7`；换向确认 `1` 帧 |
| recovery | YOLO 与分割同时连续丢失 `20` 帧进入；同时有效连续 `5` 帧退出 |
| 自动结束 | 最新 raw phase 为 rota、检测有效、血管像素达到旋转前 `3×`、`|RZ|<=12°`，连续 `20` 帧后 terminate |
| 发布门 | `b` 只开启 action/轴向过程；按 `m` 后才发布扫描所需 Y/RZ/phase；每 200 ms 心跳重发 |

视觉源码中的一处 recovery 注释仍写 `0.002 m/s`，但真实速度由 C++ 控制配置决定，当前为
`0.0002 m/s`。恢复方向以当前 C++ 约定为准：图像左侧锁存为 `-Tool-Y`，右侧为
`+Tool-Y`，并使用进入旋转前锁存的 Tool-Y 基方向。

## 标定状态

当前 v9 文件创建于 `2026-08-06T02:58:29Z`，传感器 ID 为 `DU0DU5LC`，探头模型摘要已
记录。质量结果为：

| 指标 | 当前值 | 接受门 |
| --- | ---: | ---: |
| Force RMS | `1.1534 N` | — |
| Force max | `1.7448 N` | `0.6 N` |
| Torque RMS | `0.02675 N·m` | — |
| Torque max | `0.05610 N·m` | `0.1 N·m` |

`residuals_verified=false` 且 `tool_chain_verified=false`。当前文件只能在显式 provisional
commissioning 约束与悬空 tare 下使用。tare 只能消除本次静态偏置，不能替代多姿态重力、
质量、质心以及 R/t/TCP 验证。

启动 tare 为 3 s，静止跨度门为关节 `0.02°`、TCP `0.35 mm`、姿态 `0.10°`；provisional
运行在施加 tare 前还要求合力/合力矩不超过 `5 N / 0.5 N·m`。

## 已完成与待验收

| 能力 | 状态 | 当前说明 |
| --- | --- | --- |
| RM75 TCP、状态快照、ServoJ、Hold、Stop | 已实现 | 单一 I/O owner；ServoJ 单槽 mailbox；Stop 优先并确认静止 |
| 七轴 FK、6×7 Jacobian、数值 IK | 已实现 | 规划器检查有限值、关节限位、速度/加速度和奇异性；真机边界仍需持续验收 |
| Haptron Modbus 与线程安全快照 | 已实现 | 包含 checksum、sequence、单调时间、stale 和统计信息 |
| 标定加载、模型摘要、runtime tare | 已实现，正式标定未完成 | v9 force max 不合格，tool chain 未验证 |
| 接触点估计 | 已实现，真值验收未完成 | STL 事务式加载；低力为正常无接触，不用零点伪装有效结果 |
| Z 导纳、超力卸载、制动与重接触 | 已实现，仿体验收未完成 | 卸载在 Redis Hold 下仍可独立工作；idle 不自动重接近 |
| Tool-Y/X/RZ 状态机 | 已实现，端到端验收未完成 | 包含跟踪暂停/恢复、Y 主导重建参考、连续扫描和 RZ 额度 |
| 旋转丢失恢复 | 已实现，真机验收未完成 | X/RZ 归零、固定基方向 Y 搜索、20 mm 上限、保留 Z 力控 |
| Redis v1 session/sequence/freshness | 已实现 | 新连接清空旧命令；重复/回放/过期/断开全部 fail-closed |
| 反馈恢复与新命令屏障 | 已实现 | 从实测关节重建参考，禁止恢复后继续旧视觉命令 |
| 异步 Redis 与运行日志 | 已实现 | 控制循环不做阻塞 Redis、JSON 序列化或磁盘写入 |
| CSV/summary 诊断 | 已实现 | summary schema v2，记录标定、控制、tare、安全、时序和 completion/fault |
| 自动测试 / 回放 harness | 未完成 | CTest 无注册测试；Redis parser、状态机、断线/恢复和离线控制均需覆盖 |

## 当前状态机

```text
Initializing / Observe
  → Armed / Redis Hold
    └─ fresh moving（b）→ Approach：+Tool-Z 接近
        └─ 接触 → Contact / ForceSettle：Z 导纳，等待稳定力带
            └─ fresh scan（m）→ Scan：-Tool-X + Tool-Y + Z 力控
                └─ phase=1 → TriggerAlign → RotateAlign：停止 X，保留 Y/Z，执行 RZ
                    └─ phase=2 且 recovery=true
                        → X/RZ=0，冻结 Roll/Pitch，固定基方向搜索 Y，保留 Z

任意运动阶段：目标相对超力 → Unload → Brake
  moving 且需要时 → Reacquire → 普通 Z 导纳
  idle → Armed，不自动重接近

Redis/反馈/mailbox 可恢复异常 → Hold → 重建参考 + 等待更新命令
真实硬件、安全门、规划或 deadline 故障 → Fault → Stop/静止确认 → 退出
terminate/t → 本轮完成并回到 Armed；Ctrl+C → Stop 并退出
```

## 日志与故障定位

每次运行生成 CSV 和同路径的 `.summary.json`。无参数 profile 默认写入：

```text
infer/Robot/build/logs/*.csv
infer/Robot/build/logs/*.summary.json
```

优先检查：

| 字段 | 用途 |
| --- | --- |
| `result`、`fault_code`、`completion_reason` | 区分正常结束、terminate、Ctrl+C 和 fault |
| summary `control` | 确认本次真正生效的目标力、速度、跟踪门和恢复参数 |
| `command_session_id`、producer/receive sequence、`command_age_ms`、hold reason | 定位重连、回放、过期或新命令屏障 |
| `requested_dx/dy/dz_tool_m`、旋转增量 | 判断控制律是否真正请求该自由度 |
| `control_fz_n`、unloading/recovering | 判断普通导纳、卸载、制动和重接触 |
| `recovery_search_active`、locked side、velocity/distance/limit | 判断恢复方向、速度和 20 mm 上限 |
| actual/model error、tracking paused/rebase | 判断实际 TCP 是否跟上及是否重建参考 |
| Servo submitted/consumed/sent/result sequence | 判断 mailbox 或异步发送故障 |
| work time、lateness、missed periods | 判断 10 ms 调度与 deadline fault |

Redis 命令异常通常进入 Hold 而不退出；机器人/传感器持续失效、原始或补偿 wrench 超硬门、
关节限位、IK/奇异、ServoJ 失败、全局跟踪误差、deadline 或无法确认 Stop 会进入 Fault。

## 下一步优先级

1. **统一配置所有权（P0）**：删除无参数 `Options` 与 `Rm75ControlConfig` 的双重力控/扫描值，
   让启动校验、实际控制、终端输出、profile 名称和 summary 使用同一配置；先离线验证，再决定
   正式目标是 `-2 N` 还是 `-3 N`。
2. **完成正式标定（P0）**：独立核对 R/t/TCP，重新采集低外力多姿态样本，将 force max
   降至 `<=0.6 N`，设置 residual/tool-chain verified 后完成多姿态悬空验收。
3. 修正视觉 recovery 速度注释，并离线/真机确认左右侧到 Tool-Y 的符号、旋转前基方向、
   `0.2 mm/s` 搜索速度和 `20 mm` 停止上限。
4. 在软质仿体上依次验收接触点真值、Z 稳态、超力卸载、idle 不重接近、Tool-Y、Tool-X、
   phase/RZ 及自动 terminate；每次只改变一个参数并保留 CSV/summary。
5. 建立 Redis JSON/session/replay/超时、状态机、反馈恢复、mailbox 和离线控制的自动测试，
   再执行长时间真机稳定性验证。

不要在 `main_rm75` 运行期间清理 `infer/Robot/build/logs/`。所有真机操作必须遵守
[AGENTS.md](AGENTS.md) 的硬约束。
