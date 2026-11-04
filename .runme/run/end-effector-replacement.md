# 末端打印件与六维力传感器替换计划

建立日期：2026-10-29；当前状态核对：2026-10-30。适用于 RM75 本体不变、更换末端打印件和六维力传感器，以及在 ArmTip 与传感器之间增加安装长度的情况。

本文件是待执行计划，勾选项和结果表均不代表已经完成验证。按阶段执行，不使用 `runme run --all`。含 `<...>` 的命令是填写模板，不能直接执行。初次验证显式使用 Observe 或 DryRun；无参数 `main_rm75` 会进入真实 Execute profile。

## 当前状态

坐标链和 schema v2 已实现；探头相对旋转 0°、Sensor 安装 +30°、安装平移 -12.5 mm，ArmTip→TCP 200.5 mm。
用户确认保留 `tool_chain_verified=true`，力残差与相机独立验证仍未完成。下方勾选只代表各项记录范围，历史配置以文末更正为准。

## 任务总表

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

## 1. 装配登记与覆盖范围确认

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

## 2. 保留传感器安装参数，探头相对旋转为 0°

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


## 3. 单独检查传感器

- [ ] 关闭 Robot、CuteCom 等串口占用者；使用 SensorMonitor 单独检查，确认读的是直连数据。
- [ ] 核对协议、设备路径、波特率、数据帧顺序和输出单位。
- [ ] 对三个力轴施加已知方向的载荷，核对符号与轴顺序；用已知力和力臂核对力矩。
- [ ] 固定姿态记录数据并分析噪声/漂移；初次可记录 10 分钟，再按正式实验时长补充记录。
- [ ] 轻微改变线缆布置，排查异常牵拉；最终固定后重新采样。

当前 KWR57 驱动默认 460800/8N1。C++ `force_sensor.cpp` 与 Python `kwr57.py` 都把六分量乘 9.80665，将 kgf/kgf·m 转为 N/N·m。若设备已输出 SI 单位，两边都需要调整；转换只做一次。

操作参考：[KWR57 接入](../../docs/kwr57_sensor_integration.md)、[长期记录](../../docs/sensor_drift_recording.md)。SensorMonitor 仅用于独立监控；项目由 Robot 采集并通过 Redis 发布遥测，监控启动时自动识别来源。切换 Robot 前退出直连会话。

通过条件：已知载荷下轴向、符号和比例正确；噪声与漂移小于所需接触识别和力控误差预算。仅看到稳定数字不算通过。

## 4. 核对接触模型

- [ ] 检查当前 `infer/Robot/model/Lprobe-IFS.STL` 是否符合新装配。
- [ ] 必要时从 CAD 导出模型，变换到 S 坐标系，单位 m，外法向朝外，格式 ASCII STL，直接覆盖 `infer/Robot/model/Lprobe-IFS.STL`。
- [x] 本轮探头相对 Sensor 恢复为 0°；已撤销上轮 STL 的 +30° 旋转，恢复原模型顶点与法向。实物匹配仍需现场核对。
- [ ] 在模型中核对 Sensor 原点、TCP 和已知接触表面的位置。
- [ ] 计算/生成模型 SHA-256，随新标定保存。

STL 表示用于反算接触点的表面，不是全装配碰撞模型。仅改变 ArmTip → Sensor、而 Sensor → 探头未变时，可保持原 STL；不要为补偿 ArmTip 加长而错误平移 Sensor 下的模型。

通过条件：模型坐标与 Sensor wrench 使用同一原点/轴，接触表面和 TCP 与实物一致。

## 5. 必要的软件调整与离线构建

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

## 6. 新装配重力、零偏和质心标定

先确认第 2～5 步通过。采集工具会连接机器人读取反馈，并启停传感器输出；它不发送机器人运动命令。换姿态使用现场已有的受控操作方式，记录后保持静止、完全悬空。

- [ ] 采集至少 6 个明显不同的姿态，默认 8 个；覆盖不同重力方向，避免仅绕单轴变化。
- [ ] 每个姿态采集稳定窗口，保留完整 CSV 和元数据；中断的 partial 文件不作正式输入。
- [ ] 使用与采集一致的安装旋转拟合，不通过换旋转来掩盖残差。

命令模板（协议与波特率按实际填写）：

```text
infer/Robot/build/force_sensor_calibration_capture \
  --robot-ip <IP> --sensor-device <设备路径> \
  --sensor-protocol <协议> --sensor-baud <波特率> \
  --sensor-to-tool-rpy-deg 0,0,30 \
  --probe-to-sensor-rpy-deg 0,0,0 \
  --poses 8 --output infer/Robot/build/calibration_samples.csv --overwrite

infer/Robot/build/force_sensor_calibrate \
  --input infer/Robot/build/calibration_samples.csv \
  --output infer/Robot/build/rm75_force_calibration.json \
  --sensor-id <新装配传感器身份> --probe-model Lprobe-IFS.STL \
  --probe-file infer/Robot/model/Lprobe-IFS.STL \
  --sensor-to-tool-rpy-deg 0,0,30 \
  --probe-to-sensor-rpy-deg 0,0,0 \
  --sensor-to-tool-translation-m 0,0,-0.0125 \
  --probe-tcp-sensor-m 0,0,0.188
```

采集的 `--overwrite` 在本轮完整采集结束后替换现有 CSV，工具仍使用临时 partial 文件。拟合命令通过残差和参数校验后直接覆盖现有 JSON，无需增加 `--overwrite`；不要手动清空文件。

几何参数已独立测量并验证后，拟合命令才可增加 `--tool-chain-verified`；它只是记录这一事实，不执行几何标定。默认最大向量残差门限为 0.6 N、0.1 N·m，正式要求还应根据应用精度预先确定，可能需要更严。

- [ ] 拟合通过，不用 `--allow-high-residual` 绕过失败。
- [ ] 用未参与拟合的新姿态检查补偿力/力矩，记录最大值和 RMS。
- [ ] 确认 `gravity_base_n` 是承载部分的重力（N），不是固定填写的重力加速度。
- [ ] 标定 JSON 身份、模型、摘要、R/t/TCP 和验证标志均有对应依据。

安装在传感器固定侧的加长件通常不计入传感器测量侧负载，但应计入机械臂承载的总工具。tare 只能消除当前姿态下的剩余固定偏置，不能替代本步骤。

## 7. 相机外参与控制器工具配置

- [ ] 设置/核对控制器总工具质量与质心；它们与传感器重力补偿是不同配置。
- [ ] 确认控制器反馈参考点为本计划采用的 ArmTip，防止 TCP 被重复补偿。
- [ ] 腕部相机相对 ArmTip 移动时，重标 `infer/Camera_wrist/gemini305_to_rm75_armtip.json` 的 `T_armtip_camera`。
- [ ] 相机更换时同时确认序列号、内参、深度尺度和图像配置。
- [ ] 使用全局粗定位时，复核 `infer/hand_eye_calibration-main/d455_to_rm75_base.json`；其中包含旧装配时期的经验平移修正，需重新评估。
- [ ] 用独立已知点验证外参；重投影误差不能代替实际三维定位误差。

采集/求解步骤见 [标定 notebook](calibration-operations.md)。求解输出需按部署 JSON 格式直接写入上述现有外参文件，保留原路径和文件名；脚本不会自动完成部署。只有测得的新证据才能更新独立验证状态。

## 8. 统一启动入口并覆盖现有文件

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

## 9. Observe、预演及低速几何验证

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

## 10. 恢复力控、视觉与正式运行

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

## 最终记录与恢复条件

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

## 2026-10-29 角度修改执行记录（历史：下方 0° 更正记录优先）

- 源码已分离 Sensor 安装旋转和探头工作轴旋转；新字段为
  `probe_to_sensor.rotation_row_major`（R_S_T），标定 schema v2；v1 兼容为相对旋转 I。
- 示例与本机 `infer/Robot/build/rm75_force_calibration.json` 已设 Sensor +30°、
  Probe +30°，安装平移为零、TCP 为 188 mm；本机部署文件不纳入 Git。
- 原 STL 顶点、法向绕 Sensor-Z +30° 旋转一次（轴经过 Sensor 原点和 TCP）；
  当时 SHA-256：`7b7e8cf3ed29a412056fc83efc2cd742dd8757f0933e2ecef205a588b931b2b1`。实物模型匹配和偏心核对仍待完成。
- 力、力矩、TCP 姿态、接触偏移及遥测使用探头工作轴；重力补偿保持 Sensor 安装轴。
  默认入口改读现有 rm75_force_calibration.json；新角度通过上述采集/拟合参数保持。
- 本机部署 `tool_chain_verified` 已清为 false，未重新拟合重力、零偏、质心或更改身份；
  这些旧数据不能作为新传感器有效标定。真机检查及闭环验证均未执行。
- 验证结果：`main_rm75`、`robot_offline_tests`、`force_sensor_calibration_capture`、
  `force_sensor_calibrate`、`arm_probe_pose`、`arm_preset_pose` 构建成功；CTest 7/7 通过。
  合成 CSV 拟合保留探头 +30°，正确恢复 Sensor 下重力，拒绝角度元数据不匹配；
  STL 全部顶点/法向相对 Git 原模型旋转 +30° 的数值检查通过，部署摘要一致。
  Python 回环脚本语法与 `git diff --check` 通过；未运行硬件或完整视觉闭环。

## 2026-10-29 角度更正：探头相对 Sensor 恢复 0°

用户最新要求探头与传感器保持之前的相对方向。本节替代上方执行记录中的探头 +30° 和合成 +60° 配置。

- `R_S_T = I`，示例和本机部署 JSON 的 `probe_to_sensor.rotation_row_major` 均恢复单位矩阵。
- Sensor 安装仍为 +30°，因此合成工作轴为 +30°；TCP 188 mm、零安装平移保持不变。
- STL 恢复为本次角度修改前的原文件（与 Git HEAD 字节一致），未通过逆旋转引入舍入误差；
  当前 SHA-256：`03f751286e06e4e8f05ba05bc89d7d797c7defff7bcb8d9f8f43727ff9f7c82d`，已同步本机部署 JSON。
- 保留独立坐标链和 schema v2，采集/拟合命令改为 `--probe-to-sensor-rpy-deg 0,0,0`。
- 几何验证标志保持 false；新传感器重力标定、真机方向及闭环验证仍待完成。
- 本次更正验证：示例/部署角度、TCP、平移和模型摘要一致性检查通过；CTest 7/7 通过。仅修改配置、模型和文档，未重新编译或执行真机运行。

## 2026-10-30 安装平移更新为 -12.5 mm

- 用户已修改示例中的 `translation_sensor_to_tool_m` 为 `[0,0,-0.0125]` m；
  已将该几何字段同步至本机部署 `infer/Robot/build/rm75_force_calibration.json`，并更新上方拟合命令。
- Sensor → TCP 保持 188 mm；按当前轴定义，ArmTip → Sensor 为 +12.5 mm，
  ArmTip → TCP 为 +200.5 mm。此值是配置换算结果，未代表实物测量通过。
- Sensor 安装 +30°、探头相对 Sensor 0°、STL 和重力拟合参数保持原值；
  `tool_chain_verified` 保持 false，仍需现场几何验证及新传感器标定。
- 配置数值检查通过：部署与示例平移一致，换算 ArmTip → TCP 为 `[0,0,0.2005]` m；现有 CTest 7/7 通过。未重新编译或执行真机运行。

## 2026-10-30 全局相机外参已部署

全局 D455 移动后的 50 组数据已离线求解，并按用户明确指令直接覆盖现有
`infer/hand_eye_calibration-main/d455_to_rm75_base.json`。
`user_confirmed=true`、`independent_validation_recorded=false`；未叠加旧经验修正。
同批次位置一致性 RMS 2.670 mm、最大 7.487 mm；独立几何/定位及真机验证仍待完成。
部署批次与摘要可从 `d455_to_rm75_base.json` 核查；采集 / 求解步骤见 [标定操作](calibration-operations.md)。

## 2026-10-30 用户确认保留工具几何链标志

用户明确要求 `tool_chain_verified` 保留为 true，已同步本机实际运行配置
`infer/Robot/build/rm75_force_calibration.json`。几何仍为 Sensor→TCP 188 mm、
安装平移 `[0,0,-0.0125]` m、ArmTip→TCP 200.5 mm、Sensor 安装 +30°、探头相对 0°。
本条替代上文该标志保持 false 的当前状态；本次记录用户确认，未新增实物测量或真机运动验证。
重力拟合 `quality.residuals_verified` 保持 false，全局相机独立验证状态保持 false。
