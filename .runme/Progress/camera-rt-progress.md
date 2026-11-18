# Camera_RT 项目进展

> 本文件记录进展、验证结果与待办；当前架构见
> [Camera_RT 架构](../../infer/Camera_RT/ARCHITECTURE.md)，操作见
> [全局相机手册](../run/camera-rt-operations.md)。

更新日期：2026-11-16

## 2026-11-16：共享图像窗口改为仅显示颈动脉采样点

按用户指定，共享模式显示 color stream（原始画面＋颈动脉点），首点红圈突出，
不显示 result 模型叠加窗口；无检测目标时也只刷新 color stream。
点云红点匹配及规划逻辑保持原值。仅语法与显示分支静态检查，未启动设备或 GUI。

## 2026-11-16：共享一台 D455 的颈动脉／点云双窗口

现场 errno=16 来自旧 Collision 执行模式占用 D455，未停止该进程。新增 camera_access 启动前只读提示。
全局 --collision 改为同源双窗口：C++ 独占相机、共享 RGB／对齐深度，cliff_demo --shared-frames 不再打开设备。
实时返回带源身份的识别结果，C++ 最近点匹配、红点显示和规划目标替换，沿法向后退 100 mm。
无需先保存 TXT；P 规划、E 执行、空格停止，退出窗口结束本阶段；未完成执行时不交接腕部。
两个窗口异步，识别失效发布 invalid，过期识别不显示实时红点，不回退旧文件。
旧串行模式和未启用 Collision 的 MoveJ_P 入口保留，旧记录按原执行范围解释。
构建、既有交接／深度投影／采样顺序测试、临时共享协议与 C++ 匹配合成验证通过；
未启动 GPU 模型、相机、GUI 或机械臂，实际显示和映射精度待现场验证。

## 2026-11-13：可选 Collision 粗定位执行链路

run_probe_target.py 新增 --collision：本轮采点并释放相机后进入 C++ 相机场景／IK／OMPL，
以 Robot 关节路径执行替代该分支的单次 MoveJ_P。--dry-run 只规划，原默认入口保留。
一键流程也可选用该分支；规划、执行或停止确认失败不启动腕部阶段。
终点沿外法向后退 100 mm，执行参考最高 3°/s；其他约束见
[执行说明](../../infer/Collision/EXECUTION.md)。10 项采点／交接、8 项模拟流程生命周期测试及
既有 probe_reach_test 通过，C++ 构建和仅本机回环的执行／停止模拟通过；未执行真实设备验证。

## 2026-11-13：按最新要求统一后退 10 cm

生产全局定位的 Tool-Z 偏置改为 -0.100 m，Collision 日常配置及 C++ 缺省后退值改为 0.10 m。
两条链路均沿原外法向后退 100 mm，相关提示、文档和既有测试预期同步；下方 20 cm 记录保留为历史。
Collision 监测程序编译、8 项既有离线交接／姿态测试、三处默认距离一致性和差异检查通过。
保留姿态采样、速度、可达性和碰撞检查策略，未启动相机或真实机械臂。

## 2026-11-13：生产全局定位与 Collision 统一后退 20 cm

按用户要求，`run_probe_target.py` 的初始 Tool-Z 偏置由 -0.100 m 改为 -0.200 m，
与 Collision 的 `standoff_m=0.20` 一致；输出提示、既有交接测试预期及当前文档同步。
保留短轴姿态、速度参数 6 和运动前几何臂展检查，旧的 10 cm 调整记录保留为历史。
8 项既有离线交接／姿态测试、Python 语法、两入口距离一致性及差异检查通过。
未启动相机或真实机械臂；统一目标距离不代表已接通 OMPL 路径执行。

## 2026-11-13：第一步定位速度翻倍

按用户要求，全局定位后的单次 MoveJ_P 默认速度参数由 3 调为 6，
启动脚本和 `arm_probe_pose` 的速度参数范围同步支持 6。
`arm_probe_pose` 已重编译；9 项 Robot 离线测试及 8 项模拟交接 / 姿态测试通过，
覆盖默认速度 6 透传及工具接受 6、拒绝 7。未启动真实机械臂，尚未进行真机速度验收。

## 2026-11-10 首次 MoveJ_P 后退距离调整为 10 cm

按用户要求，初始定位沿 Tool -Z 的偏置从 -0.200 m 改为 -0.100 m，
即距采样表面由 20 cm 改为 10 cm。保留姿态、速度及运动前几何臂展检查。
8 项离线交接 / 姿态测试通过；未执行真实机械臂运动。下方 20 cm 记录保留为历史。

## 2026-11-10 初始目标超出臂展的拦截

现场日志显示，20 cm 后退目标姿态需要腕心距肩部 664.472 mm，超过 RM75 两段臂长
合计 466 mm，超出约 198.472 mm；最终 J4=-0.053° 接近伸直，TCP 误差 198.328 mm，
姿态误差 0.049°。控制器虽返回 trajectory_state=true，仍未到达位置目标，最终验收正确阻止串联继续。
离线 FK 与日志起止位置一致到 0.005 mm 内，支持该臂展诊断。
`arm_probe_pose` 新增运动前几何必要条件检查，完整目标在连接控制器前即拒绝超界；
不缩短 20 cm 偏置、不放宽验收、不自动重试。此检查通过也不证明完整 IK 或碰撞可行。
已构建定位工具并通过离线回归；未连接或运动真实机械臂。

## 2026-11-10 首次 MoveJ_P 再远离 15 cm

`run_probe_target.py` 初始定位沿 Tool -Z 的偏置从 -0.050 m 改为 -0.200 m，
即距采样表面由 5 cm 增至 20 cm。姿态、速度及后续腕部跟随悬停设置保持不变。
本轮仅作离线验证，未执行真实机械臂运动。

## 2026-10-12 初始末端朝向旋转 180°

按用户反馈，`run_probe_target.py` 将 Tool +X 沿顶部首点到后续点的投影切向，
相对原 -X 对齐绕局部 Tool Z 旋转 180°。Tool +Z 朝内与后退 50 mm 的目标位置不变，
控制器欧拉角和可选腕部投影 seed 同步使用修正后的姿态。
12 项相机离线测试与 8 项模拟交接/姿态测试通过，覆盖斜向法向下的局部 Z 半圈旋转、
位置不变、欧拉角还原及 seed 朝向一致。本轮未采集真实相机或执行机械臂运动。

## 2026-10-12 初始点改为画面最上方

按用户要求，采样像素改为 y 从小到大稳定排序；新快照第二行是最上方采样点，
法向在该点邻域计算，下游切向从该点指向后续首个非重复点。
机械臂仍读取本轮新快照的首点生成初始位姿。
此前“最下方首点”的记录保留为历史。
12 项相机离线测试与 5 项模拟下游交接测试通过，含图像最上方点与三维 Y 最小点
不一致的回归场景；本轮未采集真实相机或执行机械臂运动。

## 2026-10-12 启动报错修复

- 本机 OpenCV 为 5.0.0，`findNonZero` 返回 `N×2`，旧代码的三维索引导致退出。
  深度补点改用 NumPy 行列坐标，补齐上下界裁剪，全零深度明确拒绝生成目标。
- OpenCV 普通窗口配合停用输入法 / XCB GL 插件，避免混入 Conda Qt 5.15.2；
  缺失的 wheel 字体目录改用系统字体。宿主两个空白窗口测试通过，只加载一套 Qt，
  `QObject::moveToThread` 和字体警告均为零。
- 渲染分桶容量设为单个网格的全部面数。CPU 与宿主 GPU 的 10,001 面合成网格
  渲染检查通过；路径格式、采样顺序、深度反投影共 11 项离线测试通过。

本轮未采集真实相机画面、未连接或运动机械臂，完整现场流程仍待复验。

## 当前状态

Camera_RT 负责 RealSense、Faster R-CNN、CLIFF / SMPL 解剖叠加及相机坐标快照；旧 YOLOv3 不在活动链路。
当前使用左颈动脉 6 个模型顶点，像素 y 升序、顶部首点；法向随首点计算。旧记录中的左侧 8 点和底部优先属于历史。

下游 `run_probe_target.py` 读取本轮新快照，用 D455 外参转换首点并构造短轴姿态，沿 Tool -Z 后退 100 mm，
通过 `arm_probe_pose` 执行控制器 MoveJ_P，默认速度 6；只读预览支持几何臂展上界检查，尚不验证完整控制器 IK / 可达性。
历史 `infer/Calibration` 目录已不存在，accepted Base 路径不是当前串联入口。

2026-10-30 已按授权部署新 D455 50 组标定结果，`user_confirmed=true`、独立验证 false，未叠加旧经验修正；
同批次位置一致性 RMS 2.670 mm、max 7.487 mm。标定入口见[说明](../../infer/hand_eye_calibration-main/README.md)，部署记录见[本文件外参部署记录](#2026-10-30-全局相机外参已部署)。

## 历史基础验证

- 2026-09-02 静态预检 blocked=0，路径格式 2/2；七个资产摘要固定，旧机路径移入 fixtures。
- 2026-09-03 用户确认当前机器 / 位置可运行；流配置提前到 start 前，退出释放设备 / 窗口。
- 2026-09-07 曾改为底部首点；09-15 曾切左侧 8 点。现行顶部排序与朝向更正见上方 10-12 记录。
- 以下环境表记录当时宿主检查，不能视为今天重新验收。10-09 内核 / NVIDIA 更新见[腕部进展](camera-wrist-progress.md)。

## 当前环境事实

| 项目 | 当前状态 |
| --- | --- |
| Conda | `camera`，Python 3.8 |
| Torch | `1.13.1+cu117` |
| torchvision | `0.14.1+cu117` |
| PyTorch3D | `0.7.5` |
| RealSense Python | `2.54.2.5684` 已安装 |
| NVIDIA | 宿主预检通过：RTX 4080 Laptop、驱动 535.129.03、显存 12282 MiB |
| CUDA runtime | 宿主预检通过：Torch 1.13.1+cu117、CUDA 11.7、RTX 4080 Laptop |
| RealSense device | 宿主 SDK 枚举通过：Intel RealSense D455，SDK serial `318122303626` |
| RealSense USB | sysfs 为 D455、5 Gb/s，USB descriptor serial `318523150496` |
| 检测器缓存 | Faster R-CNN checkpoint 已存在于 Torch hub cache，大小 167502836 bytes |
| PyTorch3D 来源 | 0.7.5 原生扩展由 `/home/cair-jacen/Pytorch/pytorch3d` editable checkout 提供 |

2026-09-02 在普通宿主终端重新运行宿主预检，结果为
`blocked=0, warnings=2`：NVIDIA driver、Torch CUDA、RealSense kernel 和 RealSense SDK
device 全部通过。当时的两个警告分别是 PyTorch3D 使用外部 editable checkout，以及
Faster R-CNN checkpoint 尚未缓存；后者现已缓存，只剩 editable 安装可移植性警告。

此前 Codex 沙箱中的 3 项 `BLOCKED` 均由沙箱看不到 `/dev/nvidia*`、USBFS 和
systemd/udev bus 导致，不能视为宿主硬件故障。后续设备验收均应在普通宿主终端执行。
sysfs USB descriptor 与 librealsense SDK 返回了两个不同序列号；后续流选择和标定元数据以
SDK `camera_info.serial_number` 为运行身份，同时保留 USB descriptor serial 供物理排障，
不得静默将二者当作同一字段。

## 模型资产

活动资产位于被 Git 忽略的 `data/`：CLIFF HR48 checkpoint、SMPL mean、SMPL neutral、
骨骼 params/shapedir、血管 params/shapedir。它们来源于
`/home/cair-jacen/CGL/Demo/AutoScan/CarotidArtery/Camera_RT/data`，摘要记录在
`tools/assets.sha256`。未复制活动入口不使用的 YOLOv3 权重。

模型可用只表示文件身份匹配，不表示模型许可允许再分发，也不表示当前人群或新相机精度有效。

## 已知功能问题

- 入口计算了 `device`，但多个模型和张量仍硬编码 `.cuda()`。
- 未检测到人体时没有可靠的 `bbox is None` 分支。
- 深度全部为零时明确报错退出，尚未实现无效深度帧的在线恢复。
- 路径输出没有时间戳、设备身份、内参摘要、置信度或坐标 schema 版本。
- `artery_path_rbt.txt` 没有当前生成器，历史含义不可信。

## 已有验证结果

静态预检、格式测试、宿主运行预检与入口启动均已通过。该结论证明依赖、资产、设备访问和
实时程序可用，但尚不代表叠加精度、深度精度或输出路径空间准确性已经验收。

## 下一步

1. 在现场重新保存本轮相机快照，核对顶部首点、法向和血管对应。
2. 使用部署外参做独立已知点验证，复核新装配 TCP 与深度误差。
3. 按[初始定位手册](../run/camera-rt-operations.md)先预览，再在现场条件满足时执行单次定位。
4. 补无人体检测、无效深度恢复及快照元数据；现有预检 / 格式测试不能替代空间精度验收。

## 2026-10-30 全局相机外参已部署

全局 D455 移动后的 50 组数据已离线求解，并按用户明确指令直接覆盖现有
`infer/hand_eye_calibration-main/d455_to_rm75_base.json`。
`user_confirmed=true`、`independent_validation_recorded=false`；未叠加旧经验修正。
同批次位置一致性 RMS 2.670 mm、最大 7.487 mm；独立几何/定位及真机验证仍待完成。
部署批次与摘要可从 `d455_to_rm75_base.json` 核查；采集 / 求解步骤见 [标定操作](../run/calibration_camera.md)。
