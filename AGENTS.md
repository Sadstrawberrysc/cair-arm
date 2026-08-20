# USPilot Control 项目指南

## 项目概览与目的

本仓库用于超声扫描机器人的视觉—力觉闭环控制。当前主链路由超声视觉程序生成
Tool-Y、RZ、phase 和 action 命令，经本机 Redis 传给 RM75 控制进程；控制进程融合
RM75 状态和 Haptron 六维力传感器数据，在 10 ms 周期内执行接触估计、力控状态机、
七轴数值 IK 和 ServoJ。

当前系统是使用临时标定的工程调试系统，不是已完成全量计量验收的生产系统。

主要目录：

- `infer/Robot/`：RM75 七轴通信、运动学、力传感器、接触估计、控制状态机和生产入口。
- `intergrate_infer/`：在线超声视觉推理及 Redis 命令发布，当前入口为
  `main_redis_seg_newphase_recovery_mode.py`。
- `infer/Camera_RT/`、`infer/ContactPointShow/`、`infer/SensorMonitor/`：相机、接触点和
  传感器辅助程序。
- `scan_pilot/`、`pose_pilot/`、`segmentation/`、`encoder_pretrain/`：模型训练与实验代码。
- `SonoScape_api/`、`py-xiaokai/`：超声设备接口和语音/UI 辅助模块。
- `docs/`：RM75 当前状态、启动、安全边界和学习资料。

## 技术栈与版本

机器人端的仓库级约束：

- Linux（当前开发机为 Ubuntu 22.04 系列）与 Bash。
- CMake `>=3.16`；当前开发机为 `3.22.1`。
- C++17；当前编译器为 GCC/G++ `11.4.0`。
- Eigen `>=3.3`、OpenSSL、POSIX Threads 和 hiredis。
- Redis 位于 `127.0.0.1:7777`；当前服务版本为 `6.0.16`。

视觉主链路使用 Conda 环境 `carotid`。当前开发机已验证的基线为：

- Python `3.10.18`
- PyTorch `2.0.1+cu117`
- torchvision `0.15.2+cu117`
- OpenCV `4.11.0`
- NumPy `1.26.4`
- redis-py `6.4.0`

其他子项目有各自的依赖文件和 Conda 环境，不应把 `carotid` 的版本强行套用到整个仓库；
例如 `infer/Camera_RT/requirements.txt`、`scan_pilot/requirements.txt` 和
`py-xiaokai/requirements.txt` 分别描述其局部依赖。

## 首次构建与运行

在仓库根目录先构建机器人生产入口：

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release
cmake --build infer/Robot/build --target main_rm75
```

确认 Redis；仅在没有返回 `PONG` 时启动一个本机实例：

```bash
redis-cli -h 127.0.0.1 -p 7777 ping
redis-server --bind 127.0.0.1 --port 7777 --daemonize yes
```

以下命令连接真实硬件并可能驱动机械臂。只有完成下节的现场检查且获得明确运行授权后，
才可启动：

```bash
cd infer/Robot/build
./main_rm75
```

另开终端启动视觉程序：

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate carotid
cd /home/cair-jacen/uspilot_ctrl-main/intergrate_infer
python main_redis_seg_newphase_recovery_mode.py
```

视觉窗口中，`b` 开始接近和 Tool-Y，`m` 开始 Tool-X 扫描，`t` 结束本轮并回到
idle，`q` 发布 terminate 后退出。停止顺序为先在视觉端按 `t` 或 `q`，再在
`main_rm75` 终端按 `Ctrl+C`。

维护工具默认不构建；只有明确需要诊断或标定时才使用
`-DBUILD_MAINTENANCE_TOOLS=ON`。项目目前没有注册 CTest 自动测试，也没有完整的自动
回放 harness；不要把“编译成功”表述为“控制逻辑或真机测试通过”。

## 不可违反的硬约束

1. 未经用户明确要求和现场确认，不得启动 `main_rm75`，不得运行带 `--execute` 的
   诊断/精度脚本，也不得发送 ServoJ、预置位姿或其他真实运动命令。构建和只读源码检查
   不属于真机运行。
2. 每次真机启动前必须确认：探头完全悬空、线缆不受力、运动空间无人且无障碍、方向已核对、
   物理急停在手边且可用、Redis 和当前 RM75 标定文件有效。无参数 `main_rm75` 就是真机模式，
   不得把它当作 dry-run。
3. 不得绕过或放宽原始/补偿 wrench、关节限位、IK/奇异、通信陈旧、跟踪误差、控制周期、
   tare 或 emergency stop 等安全门，除非用户明确要求该项变更并说明了验证方案。
4. 当前 `rm75_force_calibration_v9_provisional.json` 是未通过正式 `0.6 N` 残差门的临时标定。
   启动 tare 不能替代多姿态重力、质量、质心及 R/t/TCP 标定；不得宣称其已正式验收。
5. 控制、规划和运行安全参数以 `infer/Robot/include/rm75_control.hpp` 中的
   `Rm75ControlConfig`、`Rm75ServoPlannerConfig` 和 `Rm75RuntimeSafetyConfig` 为准。
   不要在 `main_rm75.cpp` 重复覆盖常规算法参数；该文件负责启动编排、硬件 I/O、坐标链注入
   和日志。
6. 保持生产模块边界：传感器协议/标定补偿属于 `force_sensor`，接触点算法属于
   `contact_sensing`，运动学与控制属于 `realman_kinematics`/`rm75_control`，Redis I/O 属于
   `redis_bridge`，`main_rm75` 只做编排。
7. `infer/Robot/tests/legacy/six_axis/` 仅作旧六轴参考，不得重新接入 RM75 七轴生产入口。
   `infer/Robot/tests/tools/` 是会接触硬件的维护工具，不等同于自动化单元测试。
8. 不得在 `main_rm75` 运行期间删除或清理 `infer/Robot/build/logs/`。修改控制参数时每次只改
   一个参数或门控条件，并保留对应 CSV 与 summary 供追溯。
9. 不要提交生成的构建产物、运行日志、模型权重或本机设备路径产生的临时文件。不要覆盖用户
   已有的未提交改动；修改前后都应检查 `git status --short`。
10. 涉及真实硬件的失败应先进入安全停止并报告证据，不得通过自动重试运动、扩大阈值或忽略
    返回码来“让流程继续”。

## 详细文档

- [系统架构与 API 决策](ARCHITECTURE.md)：服务边界、依赖方向、Redis 协议和各服务模块文档索引。
- [根目录 README](README.md)：各主要程序的简要入口。
- [RM75 构建与启动命令](docs/rm75_build_and_start_commands.md)：完整启动、按键与停止顺序。
- [RM75 当前进度与运行边界](PROGRESS.md)：当前配置、标定状态、安全门、状态机、日志和待验收项。
- [RM75 学习路径](docs/rm75_learning_path.md)：源码阅读顺序、模块职责和真机验证方法。
- [远程相机监控](docs/remote_camera_monitoring.md)：Daheng Galaxy SDK、远程视频流和 SSH 隧道。
- [机器人构建定义](infer/Robot/CMakeLists.txt)：实际 C++ 标准、依赖、目标和维护工具开关。
