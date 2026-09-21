# Gemini 305 标定彩色采集

`collect_data.py` 已将原 RealSense 采集和启动代码保留为注释，使用 OpenCV/V4L2
读取 Gemini 的 MJPG 彩色节点，配置为 640×480、30 fps。此路径不依赖 Python Orbbec
SDK 绑定；需要带 V4L2 和 GUI 支持的 OpenCV、NumPy。先关闭占用同一彩色节点的预览程序。

从仓库根目录仅预览（不连接机器人、不保存样本）：

```bash
python3 infer/hand_eye_calibration-main/collect_data.py --preview-only
```

可添加 `--max-frames 30` 做有限帧检查。默认节点为 `/dev/video16`，编号可能变化，
可通过 `--camera-device` 指定 Gemini 的 MJPG 彩色节点或稳定的 `/dev/v4l/by-id/` 路径。
程序检查设备名称和输出图像格式，不会自动选择其他相机。

完成现场检查、确认 Base 下机器人位姿参考点为 ArmTip 后，在授权的标定采集现场使用：

```bash
python3 infer/hand_eye_calibration-main/collect_data.py --robot-ip ROBOT_IP
```

将 `ROBOT_IP` 替换为实际地址。省略时沿用原 `get_ip()` 探测，原辅助模块还依赖 tkinter。
普通采集模式保留原机器人初始化：连接 8080 端口并发送 `set_change_work_frame` 到 Base，
按 `s` 时读取状态；它不是纯相机预览，也不应与生产控制进程并行使用。
该初始化的原响应校验及 ArmTip/工具参考点确认尚未完善，不代表可直接开始正式标定。

机械臂完全静止后按 `s` 保存，`q` 或 Esc 退出。窗口放大显示，但保存原始 640×480 图像。
每轮自动创建 `data/gemini_时间戳/`，含 `1.jpg` 等图像及对应行的 `poses.txt`（m、rad）；
也可通过 `--output-dir` 指定不存在的新目录。读取状态失败不保存、不递增；写盘异常终止本轮，
异常或中断后的目录应检查配对完整性，不得直接用于求解。

Gemini 手眼采集和验证继续归本目录负责。

## 离线眼在手上求解

在具有 NumPy、OpenCV、SciPy、PyYAML 的 `camera` 环境中，从本目录运行：

```bash
python compute_in_hand.py --data-dir data/gemini_20260911_151012_849202
```

`--data-dir` 替换为实际采样目录。省略时优先选择本模块 `data/` 下最新的 `gemini_*`
目录，无新版目录才回退到旧 `eye_hand_data/`。选中目录为空或不完整时明确报错，
不静默改用更旧的数据。默认配置相对于脚本读取，也可使用 `--config` 指定配置。

图像编号必须从 1 连续开始，与 `poses.txt` 一行一图严格对应。角点检测失败时同步排除
该编号的机器人位姿；图像损坏、尺寸混杂、非有限位姿、有效样本不足或缺少非平行旋转轴
运动时拒绝求解。`L` 是实际格长（m），`XX/YY` 是内部角点数。

求解打印有效样本编号、相机重投影 RMS、Camera 到末端的 R/t 和 xyzw 四元数，
不连接机器人、不覆盖部署外参，也不再生成中间 `RobotToolPose.csv`。
输出名按输入位姿为 Base 下 ArmTip 的约定解释；该参考点必须由采集现场核实。
重投影 RMS 单位是像素，不代表手眼定位误差或独立验收通过。

# 数字人单点到 RM75 Probe TCP

## 一次启动完整流程

完成本轮现场检查后，在 `camera` 环境中从仓库根目录运行（默认真实运动，速度3）：

```bash
python infer/hand_eye_calibration-main/run_probe_target.py
```

先打开数字人窗口；按 `s` 保存，按 `q` 退出后，入口检查本次文件已更新，读取第一行法向量，
读取第一个采样点，即画面最下面的红点（像素 y 最大；由相机保存前排序）。用新外参计算 `p_base = R_base_camera @ p_camera + t_base_camera`，
然后调用 `arm_probe_pose --target-pose-m-rad ... --controller-movej-p`，自动传入速度3和双执行参数，
通过控制器 `movej_p` 执行单次初始位姿移动。此流程仅负责到达扫描初始位姿，不是生产扫描控制。
相机保存的是 Z≤0 的朝相机外法向，入口取反后只用外参旋转：
`n_inward_base = -R_base_camera @ normalize(n_camera)`，不加平移。
用户确认阵列长边沿 Tool-Y，初始图像要求短轴切面。直接构造目标坐标系：
`Z = normalize(n_inward_base)`，`t_surface = normalize(t - dot(t,Z)*Z)`，
`X = -t_surface`，`Y = Z × X`，得到右手旋转矩阵 `[X,Y,Z]`。
因此 Tool-YZ 成像平面垂直于血管投影切向，+Z 朝内，-X 沿采样方向。
不再生成长轴姿态或叠加固定 -90° 旋转；轴符号保留原最终朝向以避免额外翻转。
切向取第一点指向后续首个非重复点（距离大于1微米）的方向，按当前保存顺序由画面底部向后续点。
方向只用外参旋转，不加平移；完整姿态不再依赖当前探头姿态。程序将旋转矩阵转换为 ZYX
欧拉角，通过完整六维探头位姿参数交给 Robot，后者仍负责 TCP 到 ArmTip 换算。
至少需要两个不同采样点；点不足、全部重复或归一化切向投影长度小于0.001 时拒绝运动。
这是局部采样连线的切向近似，依赖深度质量及采样顺序；像素 y 排序不保证弯曲或分支血管的拓扑顺序。
入口仅保留目标 Tool -Z 后退50 mm：
`p_target = p_base - 0.050 * R_base_tool[:, 2]`。
2026-09-15 按用户要求，将原入口的 Tool +X 50 mm、+Y 100 mm 补偿，按当时
`artery_path.txt` 的参考目标姿态折算为固定 Base 平移，写入 `d455_to_rm75_base.json`。
相机旋转不变，平移修正量为 `[0.040868946102, 0.079451898080, -0.067209561344] m`。
这是经验外参修正，仅在参考姿态下等价于原 Tool X/Y 补偿，不是重新完成手眼标定。
所有读取此 JSON 的程序都会使用修正后的平移，不限于本入口。根目录 `DECISIONS.md` 保存原矩阵、参考采样内容及摘要和目标旋转，供追溯和恢复；
JSON 保持原字段结构和矩阵排版。
独立验证状态仍为 false，不应再次叠加原 X/Y 补偿。
目标 Tool +Z 朝人体内部，故负向 Z 偏置用于停在修正后的目标外侧；使用目标 Tool 轴。
此偏置施加在 Probe TCP 目标上，不是 Base-Z 固定偏移，也不修改188 mm工具长度；
Robot 随后执行已有 TCP 到 ArmTip 换算，不重复添加偏置。短轴姿态保持不变。
日志分别打印表面采样点、偏置距离和最终目标；这是终点几何距离，不保证整条 MoveJ 轨迹保持该距离。
法向来自局部深度平面，不保证解剖方向或估计精度；
新姿态需先 dry-run 并现场核对方向和短轴图像；几何对齐不代表已验证超声成像效果。
MoveJ 途中不保证直线或始终保持该朝向。

若只需读取状态并预览目标（不检查控制器逆解或可达性），运行：

```bash
python infer/hand_eye_calibration-main/run_probe_target.py --dry-run
```

无参数命令在本轮保存并退出数字人后会进入真实 MoveJ_P 流程，直接把换算后的 ArmTip 位姿交给控制器。
此模式不执行本地 ServoJ 迭代 IK、模型一致性、3° 关节裕量、奇异性、关节变化幅度和关节路径预检查，
也没有预期目标关节角可用于最终关节误差检查。控制器保护不等同于原本的这些本地检查。
仍检查标定链、启动状态、命令返回结果和最终 Probe TCP 位置/姿态误差；失败请求停止，不重试运动。
独立调用 `arm_probe_pose` 不加 `--controller-movej-p` 时仍使用原有本地规划模式。
`Ctrl+C` 中断流程；数字人异常退出、未保存或文件非法时不会调用 Robot。不自动重试运动。
数字人目前也支持 Esc 保存退出，此入口会把它视为有效的新快照。

需要构建 Robot 时：

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target arm_probe_pose -j2
```

当前外参保存在同目录 `d455_to_rm75_base.json`，定义 `T_base_camera`：把 D455 彩色光学
坐标（米）映射到 RM75 Base 坐标（米）。

先转换数字人保存的单点：

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/hand_eye_calibration-main
python transform_point.py --camera-point-m X Y Z
```

复制输出的 Base 位置，回到仓库根目录做 Robot dry-run：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
infer/Robot/build/arm_probe_pose \
  --target-position-base-m "BASE_X,BASE_Y,BASE_Z" \
  --tool-calibration infer/Robot/build/rm75_force_calibration.json \
  --velocity 1
```

上述手动命令未传入法向，仍保留原行为：Robot 读取启动时的 ArmTip 状态，通过 `CalibratedFrameChain` 取得当前 Probe TCP 姿态，
打印完整 `Base→Probe_TCP [x,y,z,rx,ry,rz]` 并运行 IK。单点模式不添加50 mm偏置。

当前新外参由用户确认，但尚未记录独立验证结果；转换输出不是运动授权。真机执行仍需
目标预览、已知点和 Base 方向现场核对、整段净空确认及明确授权。
初次验证采用悬空、速度1、小位移，并核对实际到位误差；离线检查不代表真机已验收。
# 腕部投影交接扩展（2026-09-15）

原 `run_probe_target.py` 粗定位流程保持不变。显式增加 `--wrist-handoff` 时，
仅在运动子进程成功退出后发布 TTL 60 秒的 Redis 种子；`--dry-run` 不发布。
种子包含第一表面点（未加 50 mm 后退）、初始 Tool 旋转、快照文件时间和标定摘要。
`--wrist-calibration` 可指定 Gemini 到 ArmTip 外参，`--wrist-serial` 指定相机序列号。
默认粗定位仍会执行真实运动，必须遵守现场启动要求。

当前交接只用于候选投影预览，不代表手眼外参或同一皮肤点关联已验证。
详见 [第一阶段启动和检查](../Camera_wrist/USAGE.md)、
[两端 Redis 字段](../Camera_wrist/ARCHITECTURE.md)。
