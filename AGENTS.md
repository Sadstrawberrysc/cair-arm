# USPilot Control

## 项目上下文

本仓库用于超声扫描机器人的视觉—力觉闭环控制，包含 RM75 七轴控制、在线超声推理、
相机定位与追踪、标定，以及模型训练实验。超声视觉主链路通过本机 Redis
（`127.0.0.1:7777`）向机器人控制进程传递命令。

按任务定位代码，无需先通读整个仓库：

- `infer/Robot/`：机器人控制与通信；入口为 `src/main_rm75.cpp`。
- `intergrate_infer/`：在线超声推理；主入口为 `main_redis_seg_newphase_recovery_mode.py`。
- `infer/Camera_RT/`：全局相机与粗定位；`infer/Camera_wrist/`：腕部相机与追踪。
- `infer/hand_eye_calibration-main/`：手眼标定。
- `infer/ContactPointShow/`、`infer/SensorMonitor/`：接触点与传感器辅助程序。
- `scan_pilot/`、`pose_pilot/`、`segmentation/`、`encoder_pretrain/`：训练与实验。
- `SonoScape_api/`、`py-xiaokai/`：超声设备接口及语音/UI 辅助模块。

## 协作与完成标准

根据用户当前请求和会话上下文确定交付目标，自主完成必要的实现、相关验证和问题修复。
用户要求落地时，持续推进到可交付结果，不停在方案或第一版实现等待例行确认。
实现方式由任务决定，不预设逐功能串行验收、文件数量或模块拆分方式。

常规实现细节自行判断；只有缺失信息会实质改变结果且无法从上下文推断时才提问，
并继续推进不依赖该答案的工作。用户中途补充要求时，保留已完成且仍适用的工作。

用户明确指令优先于本文件及技能中的项目工作建议。在读取技能或其他指引时，
只采用与当前任务相关的内容；如果某条指引导致暂停、请求确认或偏离目标，
指出具体文件和条文，并说明实际阻碍。

改动应覆盖请求所需的完整范围，避免夹带无关清理。修改前后检查 `git status --short`，
保留用户已有的未提交改动；不把构建产物、运行日志、模型权重或本机临时文件纳入提交。

## 环境与验证入口

Robot 使用 Linux/Bash、C++17、CMake >=3.16，依赖 Eigen >=3.3、OpenSSL、Threads 和 hiredis。
视觉主链路使用 Conda 环境 `carotid`；其他子项目按各自依赖文件选择环境。
具体版本和可用设备以当前环境及配置为准，不把历史版本记录视为全仓库锁定要求。

Robot 构建与离线测试入口（在仓库根目录执行，按改动选择所需命令）：

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_MAINTENANCE_TOOLS=OFF
cmake --build infer/Robot/build --target main_rm75 robot_offline_tests
ctest --test-dir infer/Robot/build --output-on-failure
```

本地构建、离线测试及本次改动引起的修复可连续执行，无需逐步确认。
验证应与改动范围相称：文档改动检查内容、链接与差异；代码改动运行相关构建或测试。
相关检查通过后，仅在新增改动、失败或未解决疑点需要时扩大或重复验证。

交付时说明实际改动、验证结果和仍未完成的事项。区分源码检查、编译、离线测试与实际运行，
不把未执行的验证描述为通过；环境阻塞时说明具体原因和剩余步骤。

## 按需查阅

- 理解系统关系或接口时，查 [ARCHITECTURE.md](ARCHITECTURE.md) 及其模块文档索引。
- 确认当前进度、配置或待办时，查 [PROGRESS.md](PROGRESS.md) 或相关子目录的进度文档。
- 查找启动方式时，查 [README.md](README.md) 和
  [RM75 构建与启动命令](docs/rm75_build_and_start_commands.md)。
- 处理 Robot 构建与测试时，以 [CMakeLists.txt](infer/Robot/CMakeLists.txt) 为准。
- 处理远程相机时，查 [远程相机监控](docs/remote_camera_monitoring.md)。
- 追溯设计原因时，查 [DECISIONS.md](DECISIONS.md)。新的跨模块、公共 API 或长期维护
  决策在该文件追加日期、决策及原因，保留旧记录；普通小修无需新增决策条目。

## 沟通

默认使用简洁中文，先说明结果，再补充必要证据和文件位置。
较长任务只更新关键进展、发现和阻碍；最终回复让用户无需翻阅过程也能理解交付状态。
