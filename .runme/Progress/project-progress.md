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
| 全局定位 | 新相机快照的顶部首点，短轴 +X 沿切向、+Z 朝内，后退 50 mm；控制器 MoveJ_P，默认速度 3 |
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

装配参数、覆盖路径和执行记录见 [末端替换](../run/end-effector-replacement.md)。
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
