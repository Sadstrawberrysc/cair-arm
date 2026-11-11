# USPilot 项目进展（Runme notebook）

当前状态核对：2026-10-30。本文记录有效入口、待办与历史证据；接口见
[系统架构](../../ARCHITECTURE.md)，启动见 [Robot 操作](../run/robot-operations.md)。
历史测试和真机记录保留原执行范围，本次文档整理不代表新增设备验收。

## 当前状态

| 模块 | 实现与边界 |
| --- | --- |
| 超声闭环 | Redis v1 意图 → 10 ms 控制 → 接触 / 力控 → 七轴 planner → 异步 ServoJ；端到端仿体与计量验收未完成 |
| 力传感器 | 默认 KWR57 RS422/460800，保留 Haptron / AA55；单位、轴向和新装配多姿态标定待实测 |
| SensorMonitor（可选工具） | 独立监控，自动识别项目 Redis 遥测或独立串口，无需传参。清零 / CSV / PNG 仅供本地查看，不参与项目控制 |
| 全局定位 | 新相机快照的顶部首点，短轴 +X 沿切向、+Z 朝内，后退 100 mm；控制器 MoveJ_P，默认速度 3 |
| 腕部跟随 | 模型默认自动初始选点与请求，手动模式人工开始；无力候选脚本持续运行，TCP / 姿态 / 每关节命令限速为 4 mm/s、4°/s、4°/s |
| 相机标定 | `calibrate_realsense.py` / `calibrate_gemini.py` 默认采集 50 组后求解；提前结束不求解，不自动部署 |
| Collision | [几何／FCL](../../infer/Collision/README.md)、[只读监测](../../infer/Collision/MONITOR.md)：保留用户确认的 URDF，已实现 D455／独立机械臂状态源、单帧 OctoMap、距离与 3D 回放；下载 SDK 头文件直接链接匹配预编译库，无 SDK 构建，13 项 CTest 通过；实机、安装接触策略、生产反馈桥接、OMPL 和运动仍待完成 |

相机、模型、离线规划可用不代表实际定位精度、力残差、时序或净空合格。
新 session 先发 idle 是视觉端约定，Robot 接收端尚无独立 idle 握手状态机。

SensorMonitor 无需来源参数：启动时检查项目遥测，选定 Redis 后断流停止，不回退串口。
已有选源 / 协议、伪串口及隔离 Redis 回环验证通过；这些软件检查不代表真实传感器或机器人验收。

## 当前配置与装配

| 项目 | 配置 |
| --- | --- |
| RM75 / Redis | `192.168.50.254:8080` / `127.0.0.1:7777` |
| KWR57 | `/dev/ttyUSB0`，460800/8N1；有效原始力门 50 N、力矩门最多 2 N·m；协议无 CRC |
| 部署力标定 | 可执行文件旁 `rm75_force_calibration.json`，无参数入口要求 `sensor_id=KWR57`；文件名不代表新传感器标定完成 |
| Tool 几何 | Sensor 安装 +30°、探头相对 Sensor 0°、Sensor→TCP 188 mm、安装平移 `[0,0,-0.0125]` m，ArmTip→TCP 200.5 mm |
| 几何确认 | 用户要求保留 `tool_chain_verified=true`；记录用户确认，不等于新增独立测量 |
| 力验证 | `quality.residuals_verified=false`；旧 Haptron v10 拟合不能当作新 KWR57 标定 |
| 全局外参 | 本轮 D455 50 组结果已按授权部署；`user_confirmed=true`，独立验证 false，未叠加旧经验修正 |
| 腕部外参 | 候选 `T_armtip_camera`，独立验证尚未完成 |

装配参数、覆盖路径和执行记录见 [末端替换](#末端替换计划与验收记录历史)。
全局同批次位置一致性 RMS 2.670 mm、最大 7.487 mm，不代替独立定位误差。

## 控制与视觉基线

| 项目 | 当前值 / 行为 |
| --- | --- |
| 周期 / 陈旧门 | 控制 10 ms；力 50 ms、Robot 200 ms、超声命令 500 ms；视觉心跳 200 ms |
| 目标力 / 接近 | -2 N / +Tool-Z 20 mm/s；接触锁存 0.99 N |
| Z 导纳 / 稳定 | M=3、D=20，最高 2 mm/s；力误差 ≤0.8 N、`abs(Fz)≥2 N` 连续 0.5 s |
| 卸载 / 重接触 | 比目标更负 1 N 持续 0.5 s 后卸载，最高 10 mm/s；重接触 5 mm/s；idle 禁止重新接近 |
| Y 居中 | 增益 1.0 s⁻¹、符号 -1、最多 0.5 mm/周期；Scan / Rotate 缩放 0.8 / 0.3 |
| Y 跟踪 | 暂停 / 恢复 20 / 10 mm；Y 主导误差 ≥5 mm 可重建参考，腕部不使用此重置 |
| X / RZ | -Tool-X 10 mm/s 连续扫描；RZ 5°/s、额度 120°；Trigger 的 Y 门 8 mm / 0.4 s |
| recovery | 左 -Y、右 +Y，锁存旋转前轴，0.2 mm/s、最多 20 mm；停 X/RZ、冻结 Roll/Pitch、保留 Z |
| planner / 跟踪 | 关节停止裕量 3°；普通位置误差门 25 mm、关节 20°；腕部位置配置 50 mm，候选脚本关闭该位置门 |
| phase | ConvNeXt-Base 五分类映射三阶段；Scan 8 帧 / 0.60，Trigger、Action 各 3 帧 / 0.55 |
| Y 调理 / recovery 判定 | 死区 0.01 mm、EMA 0.7、换向 1 帧；检测与分割同时丢失 20 帧进入、同时有效 5 帧退出 |
| 自动结束 | raw phase=rota、检测有效、血管像素达旋转前 3×、`abs(RZ)≤12°` 连续 20 帧 |

完整参数以源码和该次 summary `control` 为准；视觉 recovery 的旧速度注释不改变 C++ 配置。

## 历史里程碑与验证

| 日期 / 阶段 | 事实与验证范围 |
| --- | --- |
| 2026-08-20 v10 力标定 | Haptron 身份 `DU0DU5LC`；Force RMS 0.7881 N、max 1.2488 N（门 0.6 N）；Torque RMS 0.02095 N·m、max 0.05099 N·m（门 0.1 N·m）；残差未通过 |
| 2026-09-04～09-07 单点工具 | accepted Base 路径、摘要和本地 IK 工具曾通过 9/9 CTest；用户确认 188 mm 几何，`arm_probe_pose` 预警改为 3°；当前定位已改用控制器 MoveJ_P，不能套用本地 IK 验证范围 |
| 配置 / API 收缩批次 | 配置所有权、Try* 结果、transport 单 I/O owner、七轴 FK / planner、坐标链与异步日志已统一；批次 2 记录 5/5、坐标链阶段 6/6 CTest，并有当时真机复核；不延伸为新装配验收 |
| 2026-09-15 腕部投影 | observe 位姿与 seed 交接：12 项 Python、7 项 CTest；相机恢复后采集 120 帧 30 fps，预热后 91 帧时钟检查通过、约 37 ms 帧龄；未做粗定位运动，投影待验收 |
| 2026-09-16 点击跟随 | RGB-D / Redis / 50 mm 悬停参考及 planner 接通；离线与隔离 Redis 仿真不涉及真实 ServoJ；原超声 / 投影入口保留 |
| 2026-09-23 Runme 核查 | 命名单元排除 Run All，Redis 复用检查返回 PONG；接触显示改用 carotid / 现有 STL，系统 libstdc++ 解决图形退出；未运行运动或标定采集单元。历史“默认选最新批次”现已由显式标定入口替代 |
| 2026-10-02 模型合并 | Carotid_begin → Camera_wrist/detector；camera / carotid 各 79 项、ultralytics 模型相关 14 项离线通过，含假 worker / socketpair；当轮无真实 GPU、相机、Redis 回环或运动 |
| 2026-10-12 腕部更新 | 自动初始开始与限速 4：90 项腕部测试、Robot 6/6 CTest；未运行真实相机或机械臂。设备枚举为 0 台的诊断另见腕部进展 |
| 2026-10-26 KWR57 | 用户确认单次及 500 Hz 输出；软件适配与两个标定工具构建通过；按要求删除新增测试，保留原测试；当轮未连真实设备 |
| 2026-10-26 旧记录工具 | 曾有小时轮转、漂移拟合和多格式报告，临时模拟 / 伪串口验证；这些已由当前双来源监控、单 CSV、软件清零和 PNG 分析替代，历史预热条件不追溯写入新会话 |
| 2026-10-29～10-30 装配 | schema v2 分离 Sensor / Tool 旋转，探头更正为 0°、平移改 -12.5 mm；历史记录含工具构建与 7/7 CTest，未做新装配真机验收 |
| 2026-10-30 KWR57 复核 | main_rm75、离线测试和两标定工具构建成功，CTest 7/7；临时拆包 / 单位 / 噪声 / 伪终端检查，未连真机；见接入说明 |

更多证据见 [全局相机进展](camera-rt-progress.md)、[腕部进展](camera-wrist-progress.md)、
[模型进展](carotid-begin-progress.md)和 [KWR57 接入](../../docs/kwr57_sensor_integration.md)。
历史日志和模型数据留在本机，不因文档更新纳入 Git。

## 待办与诊断

1. 用已知载荷确认 KWR57 单位、轴向和型号，完成新装配重力 / 零偏 / 质心标定及独立 R/t/TCP 核验。
2. 核查 3 s 悬空 tare、采样数量、Sensor-frame offset 与拒绝原因；tare 不代替正式标定。
3. 完成相机外参、动态时序、真实扫描点与 TCP 间距的独立验证；复测腕部自动开始、丢失恢复与停止。
4. 在工装 / 仿体依次验证 Z 稳态、卸载 / idle 不重接近、接触点、Y/X、phase/RZ、recovery 与 terminate。
5. 按实际缺口补 session replay、反馈恢复、mailbox 并发和周期回放，再做长时稳定性记录。

排查先看 summary 的首个 fault、control 有效配置、命令年龄 / 序号，再对照 CSV 请求与实际 TCP、
ServoJ 提交 / 发送结果和周期超期；Redis Hold 通常不退出进程，硬件 / 规划 / deadline 故障进入停止路径。
运行期间不清理 `build/logs/`。源码阅读见 [学习路径](../rm75_learning_path.md)。

## 末端替换计划与验收记录（历史）

以下保留 2026-10-29 至 2026-10-30 的装配替换计划及当时验收范围，未勾选项不代表已完成。
后续力标定与默认入口的更新以 [力传感器进展](force-sensor-progress.md) 为准；
全局相机部署记录见 [全局相机进展](camera-rt-progress.md)。历史“当前状态”和“下一步”均指当时。

建立日期：2026-10-29；当前状态核对：2026-10-30。适用于 RM75 本体不变、更换末端打印件和六维力传感器，以及在 ArmTip 与传感器之间增加安装长度的情况。

本文件是待执行计划，勾选项和结果表均不代表已经完成验证。按阶段执行，不使用 `runme run --all`。含 `<...>` 的命令是填写模板，不能直接执行。初次验证显式使用 Observe 或 DryRun；无参数 `main_rm75` 会进入真实 Execute profile。

### 当前状态

坐标链和 schema v2 已实现；探头相对旋转 0°、Sensor 安装 +30°、安装平移 -12.5 mm，ArmTip→TCP 200.5 mm。
用户确认保留 `tool_chain_verified=true`，力残差与相机独立验证仍未完成。下方勾选只代表各项记录范围，历史配置以文末更正为准。

### 任务总表

| 顺序 | 工作 | 完成后应得到 |
| --- | --- | --- |
| 1 | 装配登记与覆盖范围确认 | 实物信息、现有文件覆盖清单 |
| 2 | 区分传感器安装与探头旋转 | Sensor 安装 +30° 不变；探头相对 Sensor 为 0°，合成工作轴安装角 +30° |
| 3 | 传感器独立检查 | 协议、单位、轴向、噪声确认 |
| 4 | 更新接触模型 | Sensor 坐标下的 ASCII STL |
| 5 | 必要的软件调整与构建 | 编译及离线检查结果 |
| 6 | 重力和零偏标定 | 覆盖现有 CSV、JSON，记录独立验证结果 |
| 7 | 相机与控制器配置 | 新外参、工具负载与参考点确认 |
| 8 | 统一启动入口 | 每条路径读取正确的新配置 |
| 9 | Observe、预演与低速验证 | 几何、接触点与方向验证结果 |
| 10 | 恢复力控和视觉闭环 | 参数、日志及验收记录 |

阶段失败时先解决该阶段问题，再进行依赖它的下一阶段。相机相关步骤仅在使用对应功能时执行。

### 1. 装配登记与覆盖范围确认

- [ ] 关闭生产控制与串口采集程序，保存已有实验结果。
- [ ] 确认下表中的现有文件作为替换目标，测量和标定完成后直接覆盖内容。
- [ ] 登记新打印件 CAD 版本、传感器完整型号/编号/量程、探头型号。
- [ ] 拍摄装配与坐标轴照片，记录增加长度的位置及线缆固定方式。
- [ ] 确认安装牢固、线缆不会对测量侧产生不可控牵拉；最终固定后再正式标定。

采用原路径直接覆盖，不建立 `assembly_v2`、另一份标定 JSON 或模型副本。采样和运行日志沿用现有工具的输出机制，不提交构建产物或原始运行数据。

| 替换对象 | 直接覆盖的现有路径 |
| --- | --- |
| 力觉与 TCP 标定 | `infer/Robot/build/rm75_force_calibration.json` |
| 接触模型（需要变更时） | `infer/Robot/model/Lprobe-IFS.STL` |
| 腕部外参（需要变更时） | `infer/Camera_wrist/gemini305_to_rm75_armtip.json` |
| 全局外参（需要变更时） | `infer/hand_eye_calibration-main/d455_to_rm75_base.json` |
| 标定采样 | `infer/Robot/build/calibration_samples.csv` |

无参数入口已统一读取 `rm75_force_calibration.json`，要求身份 `KWR57`；部署文件仍需完成新装配力标定。

| 项目 | 填写 |
| --- | --- |
| 装配编号 / CAD 版本 | 待填写 |
| 传感器完整型号 / 编号 | 待填写 |
| 各轴量程 / 输出单位 | 待填写 |
| 协议 / 波特率 / 稳定设备路径 | 待填写 |
| 探头型号 / TCP 定义 | 待填写 |
| 新增件在传感器固定侧还是测量侧 | 待填写 |
| 腕部相机是否移动 | 待填写 |
| 采样与日志位置 | 沿用现有路径，待核对 |

通过条件：覆盖目标明确；所有依赖同一装配的入口统一使用更新后的文件。覆盖后重启读取配置的进程，不在运行过程中更换其标定。

### 2. 保留传感器安装参数，探头相对旋转为 0°

约定列向量，`T_A_B` 将 B 坐标映射到 A；长度 m，旋转为右手系。

| 符号 | 定义 | 填写 |
| --- | --- | --- |
| A | 机器人反馈 ArmTip 的原点与轴；现场确认当前工具/工作参考系 | 待确认 |
| S | 厂家定义的传感器测量原点与轴，不能直接用外壳表面代替 | 待确认 |
| P | 选定的探头工作参考点 TCP | 待确认 |
| T | 随探头旋转的物理工作坐标系，控制 X/Y 对应探头工作方向 | 本轮与 S 轴重合 |
| `R_S_T` | 探头工作轴 → Sensor 轴的旋转 | 用户最新确认恢复为 0°，`R_S_T = I` |
| `R_A_S` | Sensor → ArmTip 的旋转 | 用户确认保持绕 Z 轴 +30°，RPY = `[0,0,30]` deg |
| `p_A_S` | ArmTip 原点 → Sensor 原点，用 A 轴表达 | 按当前配置为 `[0,0,0.0125]` m（ArmTip → Sensor） |
| `p_S_P` | Sensor 原点 → TCP，用 S 轴表达 | 保持 `[0,0,0.188]` m |

当前 `R_S_T = I`，逻辑 Tool 轴与 Sensor 轴重合；`R_A_S` 保持 +30°。
探头旋转与 Sensor 安装旋转分别保存，重力补偿只用 Sensor 安装旋转：

```text
sensor_to_tool.rotation_row_major = R_A_S（按行展开）
translation_sensor_to_tool_m      = -transpose(R_A_S) * p_A_S
probe_tcp_sensor_m                = p_S_P

ArmTip → TCP（A 轴表达）
    = p_A_S + R_A_S * p_S_P
    = R_A_S * (probe_tcp_sensor_m - translation_sensor_to_tool_m)
```

- [ ] 核对实际装配与保留的几何建模约定一致，记录核对结果；用户确认保留数值不等于已完成独立几何验收。
- [ ] 采集和拟合统一使用 `--sensor-to-tool-rpy-deg 0,0,30`，满足 `R = Rz * Ry * Rx`。
- [ ] 确认逻辑 +Z 为预期接近方向、X/Y 与探头成像和扫描方向匹配。
- [ ] 现场核验第 5 步已实现的坐标链和探头相对 0° 配置，通过后再恢复运动和闭环。

2026-10-30 用户将安装平移配置为 `translation_sensor_to_tool_m = [0,0,-0.0125]` m，表示 Sensor → ArmTip 沿 Sensor-Z 为 -12.5 mm。传感器安装 +30°、探头相对旋转 0° 和 Sensor → TCP 188 mm 保持不变，因此 ArmTip → TCP 为 `[0,0,0.2005]` m。该配置描述的是两个参考原点间的总偏移，实物尺寸与方向仍需独立核对。

当前几何参数（2026-10-30 更新安装平移）：后续重新拟合新传感器的重力、零偏和质心时保留以下字段，并保留独立探头相对旋转为 0°：

```json
{
  "sensor_to_tool": {
    "rotation_row_major": [
      0.8660254037844387, -0.49999999999999994, 0.0,
      0.49999999999999994, 0.8660254037844387, 0.0,
      0.0, 0.0, 1.0
    ],
    "translation_sensor_to_tool_m": [0.0, 0.0, -0.0125]
  },
  "probe_tcp_sensor_m": [0.0, 0.0, 0.188]
}
```

通过条件：计算的 ArmTip → TCP 与独立几何测量相符，符合预先确定的定位误差预算。

新增探头旋转的约定（此处 T 只区分轴向，位置原点仍按相应参考点处理）：

```text
R_A_T = R_A_S * R_S_T
R_A_S = Rz(+30°)                 # 传感器安装角保持不变
R_S_T = I                       # 用户最新确认：探头相对 Sensor 为 0°
R_A_T = Rz(+30°)                 # 合成探头工作轴方向；不用于 Sensor 重力补偿
```

工作轴旋转后的方向、力和接触点必须一致。纯旋转情况下：

```text
f_T = transpose(R_S_T) * f_S
m_T = transpose(R_S_T) * (m_S - p_S_originT cross f_S)
p_contact_relativeTCP_T = transpose(R_S_T) * (p_contact_S - p_S_P)
```

这里 `p_S_originT` 是实际选定的 Tool 参考原点在 S 下的位置，不因只讨论轴旋转而自动改为 TCP。接触估计继续用 Sensor 下的 wrench 和 STL，控制增量、TCP 姿态与接触偏移使用探头工作轴。


### 3. 单独检查传感器

- [ ] 关闭 Robot、CuteCom 等串口占用者；使用 SensorMonitor 单独检查，确认读的是直连数据。
- [ ] 核对协议、设备路径、波特率、数据帧顺序和输出单位。
- [ ] 对三个力轴施加已知方向的载荷，核对符号与轴顺序；用已知力和力臂核对力矩。
- [ ] 固定姿态记录数据并分析噪声/漂移；初次可记录 10 分钟，再按正式实验时长补充记录。
- [ ] 轻微改变线缆布置，排查异常牵拉；最终固定后重新采样。

当前 KWR57 驱动默认 460800/8N1。C++ `force_sensor.cpp` 与 Python `kwr57.py` 都把六分量乘 9.80665，将 kgf/kgf·m 转为 N/N·m。若设备已输出 SI 单位，两边都需要调整；转换只做一次。

操作参考：[KWR57 接入](../../docs/kwr57_sensor_integration.md)、[长期记录](../../docs/sensor_drift_recording.md)。SensorMonitor 仅用于独立监控；项目由 Robot 采集并通过 Redis 发布遥测，监控启动时自动识别来源。切换 Robot 前退出直连会话。

通过条件：已知载荷下轴向、符号和比例正确；噪声与漂移小于所需接触识别和力控误差预算。仅看到稳定数字不算通过。

### 4. 核对接触模型

- [ ] 检查当前 `infer/Robot/model/Lprobe-IFS.STL` 是否符合新装配。
- [ ] 必要时从 CAD 导出模型，变换到 S 坐标系，单位 m，外法向朝外，格式 ASCII STL，直接覆盖 `infer/Robot/model/Lprobe-IFS.STL`。
- [x] 本轮探头相对 Sensor 恢复为 0°；已撤销上轮 STL 的 +30° 旋转，恢复原模型顶点与法向。实物匹配仍需现场核对。
- [ ] 在模型中核对 Sensor 原点、TCP 和已知接触表面的位置。
- [ ] 计算/生成模型 SHA-256，随新标定保存。

STL 表示用于反算接触点的表面，不是全装配碰撞模型。仅改变 ArmTip → Sensor、而 Sensor → 探头未变时，可保持原 STL；不要为补偿 ArmTip 加长而错误平移 Sensor 下的模型。

通过条件：模型坐标与 Sensor wrench 使用同一原点/轴，接触表面和 TCP 与实物一致。

### 5. 必要的软件调整与离线构建

按实际变化选择，避免无关修改：

| 条件 | 修改位置与范围 |
| --- | --- |
| 同协议、同单位且工作轴仍重合 | 主要更新部署 JSON、模型及启动路径 |
| 协议/单位/轴顺序变化 | `force_sensor.cpp/.hpp`、SensorMonitor 相应 parser 和采集入口；标定工具也必须获得一致 SI 数据 |
| 独立 Sensor/Tool 轴旋转支持（已完成，本轮设为 0°） | `calibrated_frame_chain.cpp/.hpp`、`main_rm75.cpp` 的 wrench/接触点表达及相关调用；按实际需要扩展标定 schema |
| 量程变化 | `rm75_runtime_config.hpp`、`rm75_control.hpp`，并核对标定采集工具的固定原始量程门 |
| 入口路径不一致 | `rm75_runtime_config.cpp`、腕部与粗定位脚本，第 8 步统一 |

本轮需在同一标定 JSON 内独立表达 `R_A_S` 与 `R_S_T`，同步扩展读写、工具参数、日志与相关消费者；不新建部署文件。原采集/拟合命令中的 `0,0,30` 始终表示传感器安装角。第 6 步命令已加入 `--probe-to-sensor-rpy-deg 0,0,0`；采集记录该字段，拟合核对后写回 schema v2。必须核对扫描/居中、姿态规划、接触点控制与遥测、粗定位和腕部入口全部使用正确工作轴。

新增非零平移应验证 TCP 正反变换；新增轴旋转需验证力、力矩和接触点表达一致。若修改标定 schema，必须同步 JSON 读写、采集/拟合工具及全部消费者；新的公共接口决策记录到 `DECISIONS.md`。仅更换末端通常不改 RM75 本体 DH 参数。

```bash {"name":"build-replacement-tools","excludeFromRunAll":"true"}
cd /home/cair-jacen/uspilot_ctrl-main
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target main_rm75 robot_offline_tests force_sensor_calibration_capture force_sensor_calibrate
ctest --test-dir infer/Robot/build --output-on-failure
```

- [x] 角度修改所需主程序、离线测试、标定工具和姿态工具编译通过（2026-10-29）。
- [x] 本轮角度修改离线检查通过：7/7 CTest；包含 +60° 合成、非零平移 TCP 往返、力/力矩/接触一致性及 JSON 兼容检查。

### 6. 新装配重力、零偏和质心标定

当前使用 [力传感器自动静态标定](../run/calibration_force.md)：以当前悬空姿态为基准，
自动完成 1000 个不同静态姿态，采满后拟合并写回项目现有 JSON，无需手动换姿态。

```bash {"name":"calibrate-replacement-force","excludeFromRunAll":"true","interactive":"true"}
python3 /home/cair-jacen/uspilot_ctrl-main/infer/Robot/tools/calibrate_force_static.py
```

- 默认按模型全关节范围选样，结合 TCP 三维空间覆盖和重力方向，保留 `--path-mode local` 局部扫描。
- 每点到位后直接采集至少 20 帧、0.5 秒窗口取平均，允许关节、位置和姿态变化，不等待停稳；1000 姿态输出 1000 行样本。
- 工具自动读取当前 JSON 的 Sensor/Probe 安装旋转、平移、TCP 和几何验证状态，采集与拟合保持一致。
- 输入为 KWR57 / 460800 / `/dev/ttyUSB0`；SensorMonitor 直连及其他机器人控制进程需先退出。
- 完成后返回起始关节，CSV 写 `infer/Robot/build/calibration_samples.csv`，
  力标定写 `infer/Robot/build/rm75_force_calibration.json`，传感器身份设为 `KWR57`。
- 原始 CSV、完整路径和拟合质量保留；独立无接触姿态的补偿误差仍需实际验证。

安装在传感器固定侧的加长件通常不计入传感器测量侧负载，但应计入机械臂承载的总工具。

### 7. 相机外参与控制器工具配置

- [ ] 设置/核对控制器总工具质量与质心；它们与传感器重力补偿是不同配置。
- [ ] 确认控制器反馈参考点为本计划采用的 ArmTip，防止 TCP 被重复补偿。
- [ ] 腕部相机相对 ArmTip 移动时，重标 `infer/Camera_wrist/gemini305_to_rm75_armtip.json` 的 `T_armtip_camera`。
- [ ] 相机更换时同时确认序列号、内参、深度尺度和图像配置。
- [ ] 使用全局粗定位时，复核 `infer/hand_eye_calibration-main/d455_to_rm75_base.json`；其中包含旧装配时期的经验平移修正，需重新评估。
- [ ] 用独立已知点验证外参；重投影误差不能代替实际三维定位误差。

采集/求解步骤见 [标定 notebook](../run/calibration_camera.md)。求解输出需按部署 JSON 格式直接写入上述现有外参文件，保留原路径和文件名；脚本不会自动完成部署。只有测得的新证据才能更新独立验证状态。

### 8. 统一启动入口并覆盖现有文件

统一使用现有 `infer/Robot/build/rm75_force_calibration.json`，直接覆盖其中的身份、几何和重力标定；不新增标定文件。文件名沿用 rm75 不影响其保存 KWR57 数据。

| 入口 | 当前读取位置 | 待办 |
| --- | --- | --- |
| 无参数 `main_rm75` | 可执行文件旁 `rm75_force_calibration.json` | 路径已统一；复核 KWR57 身份、设备、模型和新装配标定 |
| `infer/Camera_wrist/start_robot.sh` | `build/rm75_force_calibration.json` | 保持路径，复核已覆盖的标定、腕部外参和试验参数 |
| `infer/hand_eye_calibration-main/run_probe_target.py` | `build/rm75_force_calibration.json` | 保持路径，读取覆盖后的同一装配标定 |
| 手工命令及 notebooks | 各命令的 `--calibration` / `--tool-calibration` | 更新实际使用的命令 |

- [ ] 修改前后检查 Git 状态，保留他人已有修改。
- [ ] 新配置与启动日志一致；示例 JSON 只是模板，不是默认部署文件。
- [ ] 搜索旧身份、旧 TCP 和旧路径，逐项判断，保留历史记录而不是全仓库替换。
- [ ] 量程和控制门限匹配实物；当前 KWR57 `EffectiveSafety()` 将原始力矩门限制在最多 2 N·m。
- [ ] 首次腕部验证使用保留位移与跟踪误差保护的配置，不照搬脚本中的 unlimited 试验参数。

### 9. Observe、预演及低速几何验证

先用显式 Observe，在多个悬空姿态读取补偿结果。需要监看时先确认 Robot 已向本机 Redis 发布遥测，再直接运行 SensorMonitor 自动识别并订阅；不要同时直连串口。仅记录到日志时可显式加 `--no-redis`。

```text
infer/Robot/build/main_rm75 --observe \
  --calibration infer/Robot/build/rm75_force_calibration.json --expected-sensor-id <配置身份> \
  --probe-model infer/Robot/model/Lprobe-IFS.STL --robot-ip <IP> \
  --sensor-device <设备路径> --sensor-protocol <协议> --sensor-baud <波特率> \
  --duration-sec 30 --runtime-log infer/Robot/build/logs/replacement_observe.csv
```

- [ ] 初次不加 tare，先检查跨姿态重力补偿本身；需要时另测 tare 效果。
- [ ] 使用 DryRun 检查目标方向、规划结果与 IK；它不发送 ServoJ，但通常仍连接真实设备。
- [ ] 在空旷工作区分轴验证小位移，幅度根据误差预算和净空选择，例如 1～2 mm。
- [ ] 低速验证围绕 TCP 的小角度旋转；检查探头参考点是否保持在预期位置。
- [ ] 检查加长支架、相机、探头和线缆的整段净空。STL 接触估计并不提供全装配避碰。
- [ ] 在已知接触位置的工装上验证接触点，覆盖探头中心及不同偏移位置，记录误差和无效原因。

真实运动命令应在目标姿态、速度、持续时间和工作区确定后生成；本计划不提供可误触的一键运动单元。现有生产接近速度、力门限和行程不能自动视为新装配的首试参数。

通过条件：实际平移/旋转与预测一致，TCP 几何和接触点误差满足预算；没有通过关闭保护来隐藏的偏差。

### 10. 恢复力控、视觉与正式运行

先在工装或仿体完成测试，每次仅开启当前要验证的功能。

| 顺序 | 测试 | 记录与通过条件 |
| --- | --- | --- |
| 1 | 轴向接近与接触识别 | 方向正确、接触判定可靠，记录噪声/延迟 |
| 2 | 单轴力保持 | 记录稳态误差、过冲、振荡及漂移；目标与限速按测试方案设定 |
| 3 | 卸载与失联/停止 | 记录触发条件、动作方向、停止与物理静止结果 |
| 4 | 接触点驱动姿态 | 力矩和接触偏移方向正确，无异常耦合 |
| 5 | 视觉 Y 居中 | 方向与图像误差一致，力保持合格 |
| 6 | 低速 X 扫描 | 轨迹、接触点、力和线缆稳定 |
| 7 | RZ 对齐与恢复 | 小角度验证后逐步扩大范围，记录耦合与停止行为 |
| 8 | 多轮完整流程 | 重启、重连、结束本轮后状态和配置一致 |

需要时调整 `infer/Robot/include/rm75_control.hpp` 的接触阈值、导纳质量/阻尼、滤波、姿态增益和速度限制；保护参数还涉及 `rm75_runtime_config.hpp`。虚拟质量不直接等于末端实际重量。先测基线，按可解释的参数组修改并保留对比结果。

50 mm 悬停偏置仅是 TCP 到局部表面的目标距离，不保证支架或整段轨迹净空。无力腕部模式不验证力传感器和超声力闭环。

### 最终记录与恢复条件

| 项目 | 状态 / 证据路径 |
| --- | --- |
| Sensor 安装 R/t、TCP 位置 | 用户确认保持不变；实际装配一致性和独立几何验收待核对 |
| 探头相对 Sensor 的旋转 | 最新确认 0°，合成 ArmTip ← Tool 为 +30°；坐标链已实现、离线通过，真机方向验证待完成 |
| 传感器单位、轴向、量程 | 待完成 |
| 接触模型及摘要 | 待完成 |
| 编译与离线检查 | 角度修改构建通过、7/7 CTest 通过；合成标定和 STL 数值检查通过 |
| 重力拟合及独立姿态验证 | 待完成 |
| 控制器负载与参考系 | 待确认 |
| 相机外参独立验证（适用时） | 待完成 |
| 各入口文件/身份一致性 | 待完成 |
| 真机 TCP 与接触点误差 | 待完成 |
| 力控、视觉、停止验证 | 待完成 |
| 正式参数、适用范围、剩余限制 | 待填写 |

- [ ] 更新对应运行与进展 notebook，记录实际执行日期、配置版本和证据。
- [ ] 在现有进展记录中登记源码版本、标定日期、模型摘要、参数及结果，区分离线与真机验证。
- [ ] 核对覆盖后的文件与当前实物一致。若以后恢复旧实物，应同时恢复匹配的配置；本次不要求额外建立备份副本。

当前下一步：按第 3 步检查传感器，核对模型与实物，再执行第 6 步采样标定。Sensor/Tool 坐标链分离及离线验证已完成；探头相对旋转改为 0°，STL 已恢复原模型；Sensor 安装 +30°、TCP 188 mm 保留，安装平移更新为 `[0,0,-0.0125]` m，工作轴合成为 +30°。恢复运动和闭环前仍需完成现场几何及新传感器标定验证。
