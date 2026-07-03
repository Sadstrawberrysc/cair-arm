# Realman RM75 机械臂控制进度记录

## 当前目标

- 先跑通纯机械臂控制链。
- 当前阶段只关注 Realman 机械臂本体控制：连接控制器、读取关节/末端状态、执行基础低速运动。
- 暂不纳入视觉推理、超声 API、训练代码、语音 UI、力控扫查闭环。

## 已确认事实

- `infer/Robot/src/main.cpp` 当前整文件被注释，不是实际可运行入口。
- `infer/Robot/CMakeLists.txt` 当前默认构建目标是七轴只读工具 `arm_read_state` 和 `arm_read_state_raw`。
- 机械臂 TCP 封装主要在 `infer/Robot/src/realman_command.cpp` 和 `infer/Robot/include/realman_command.hpp`。
- 当前代码默认 Realman IP 为 `192.168.50.254`，端口为 `8080`。
- 当前 `realman_command` 命令封装已按 RM75 七轴改造。
- `main_nomove` 仍基于旧 6 轴运动学/力控链路，不作为当前默认可用目标。

## 已完成内容

- 梳理了项目中与机械臂控制相关的核心文件。
- 明确当前阶段只做机械臂控制，不做视觉、超声、训练、语音 UI 或力控扫查闭环。
- 确认后续应优先采用独立测试入口验证机械臂连接和只读状态。
- 新增本进度记录文件，便于持续跟踪 RM75 机械臂控制开发进展。
- 已确认本机 C++ 基础工具链可用：CMake 3.22.1、g++ 11.4.0。
- 已确认系统存在 OpenCV 4.7.0、hiredis 和 Eigen 头文件目录。
- 已用 `/tmp/uspilot_robot_build` 做过非侵入式 CMake 配置测试，配置可通过。
- 首次编译暴露当前 CMake 未包含 Eigen include 路径，导致 `#include <Eigen/Dense>` 找不到。
- 已用 `/tmp/uspilot_robot_build_eigen` 临时加入 `-I/usr/include/eigen3` 验证，`main_nomove` 可完整编译通过。
- 已确认机械臂真实 IP 为 `192.168.50.254`，TCP JSON 端口 `8080` 可连接。
- 已新增 raw 只读工具 `infer/Robot/src/arm_read_state_raw.cpp` 和构建脚本 `infer/Robot/tools/build_arm_read_state_raw.sh`。
- 已成功执行 raw 只读测试：`get_joint_degree` 返回 7 个关节，`get_current_arm_state` 返回 7 个关节和 6 维末端位姿。
- 已确认当前机械臂状态返回 `arm_err=0`、`sys_err=0`。
- 已确认旧 `RMCommand::ReadJ()` 的 6 轴 Eigen 缓存会在 RM75 七轴返回时越界，因此旧封装不能直接用于 RM75 七轴读取。
- 已将 `realman_command.hpp/.cpp` 改为 RM75 七轴默认接口：`ReadJ()`、`ReadArmState()`、`MoveJ()`、`ServoJ()` 均使用 7 轴关节向量。
- 已移除 `realman_command.hpp` 中的 6 轴关节接口和过渡接口：不再保留 `ReadJ7()`、`MoveJ7()`、`ServoJ7()`。
- 已将 `arm_read_state.cpp` 改为使用七轴 `ReadArmState()`，默认 IP 更新为 `192.168.50.254`。
- 已修改 `infer/Robot/CMakeLists.txt`：显式列出源文件，加入 Eigen include 路径，默认只构建 `arm_read_state` 和 `arm_read_state_raw`。
- 已将旧 6 轴 `main_nomove` 放到 `BUILD_LEGACY_MAIN_NOMOVE` 选项下，默认不构建，避免误用未适配七轴运动学的闭环主控。
- 已在 `/tmp/uspilot_robot_rm75_7axis_build` 验证 CMake 默认目标构建通过：`arm_read_state`、`arm_read_state_raw` 均可编译。
- 已在真机上执行七轴只读封装测试：`arm_read_state --ip 192.168.50.254 --port 8080 --repeat 3 --interval-ms 500` 成功读取 3 次。
- 真机只读测试结果稳定：`arm_err=0`、`sys_err=0`，七轴关节和 6 维末端位姿均正常返回。
- 已新增低速小幅关节运动测试入口 `infer/Robot/src/arm_movej_delta.cpp` 和构建脚本 `infer/Robot/tools/build_arm_movej_delta.sh`。
- `arm_movej_delta` 默认 dry-run，不发送运动命令；只有显式添加 `--execute` 才会执行第 7 轴小角度往返 `MoveJ()`。
- 已验证 `infer/Robot/tools/build_arm_movej_delta.sh /tmp/arm_movej_delta` 可编译通过。
- 已验证 CMake 默认目标完整构建通过：`arm_read_state`、`arm_read_state_raw`、`arm_movej_delta` 均可生成。
- 首次执行 `arm_movej_delta --joint-index 7 --delta-deg 1 --velocity 5 --execute` 时，`movej` 命令已发送，但旧解析逻辑强制读取 `trajectory_state` 为 bool；控制器实际返回 `null` 或缺失该字段，导致 `nlohmann::json type_error.302` 崩溃。
- 已将 `MoveJ()`、`MoveL()`、`MoveJP()` 的运动返回解析改为先打印原始 JSON，再仅在明确收到 `trajectory_state=false` 或非零 `arm_err` 时判错；避免返回字段为 `null` 时程序崩溃。
- 第二次 `arm_movej_delta` 证明 RM75 的 `movej` 返回是异步两阶段：先返回 `{"command":"movej","receive_state":true}`，之后才返回 `{"state":"current_trajectory_state","trajectory_state":true}`。
- 已将 `MoveJ()`、`MoveL()`、`MoveJP()` 改为等待轨迹完成包后再返回，避免连续两条运动命令导致 TCP 返回包错位。
- 已将 `ReadArmState()` 改为可跳过残留的运动 ack，继续等待真正的 `arm_state` 返回。
- 已在真机上成功执行修复后的第 7 轴 `MoveJ()` 小幅往返测试：从约 `1.005°` 运动到约 `2.005°`，再回到约 `1.023°`。
- 修复后的运动测试中，两条 `MoveJ()` 均正常收到 `receive_state=true` 和 `trajectory_state=true`，最终状态读取正常，`arm_err=0`、`sys_err=0`。
- 已继续完成 J5、J4、J3、J2、J1 的低速小幅 `MoveJ()` 往返测试，均成功。
- 当前已确认 RM75 七轴中 J1、J2、J3、J4、J5、J7 均可通过七轴 `MoveJ()` 接口安全执行小幅往返；J6 需单独确认是否已执行或补测。
- 已将 `arm_movej_delta` 的单次测试角度上限从 `3°` 放宽到 `10°`，用于下一阶段中等角度关节往返测试；默认角度仍保持 `1°`。
- 已完成 `10°` 单关节往返测试，当前七轴关节空间基础运动链路验证通过。
- 远程相机监控、Daheng SDK 安装和抓帧验证记录已迁移到 `docs/remote_camera_monitoring.md`。

## 下一步任务

- 已重新编译 `/tmp/arm_movej_delta` 和 `/tmp/arm_read_state`。
- 下一阶段建议新增“预设关节位姿测试”工具：读取当前关节、保存 home/ready 位姿、执行低速关节空间移动。
- 在预设位姿工具稳定后，再考虑更大的角度测试、末端位姿 `MoveJP/MoveL` 测试，或开始七轴运动学适配。

## 风险/注意事项

- 真机测试前必须确认急停、拖动示教、保护停止等安全机制可用。
- 未确认坐标系和运动方向前，不进行大幅度运动。
- `main_nomove` 依赖 Redis、力传感器、接触点估计、标定文件和旧 6 轴运动学，不适合作为 RM75 七轴控制入口。
- `main_nomove` 仍然依赖 6 轴运动学和 6x6 Jacobian，不能视为已完成 RM75 七轴闭环控制适配。
- 七轴 `MoveJ()` 和 `ServoJ()` 已加入接口，但尚未做真机运动测试；首次测试必须低速、小角度、空载、安全范围内执行。
- 新增的 `arm_movej_delta` 只有在添加 `--execute` 时才会发运动命令；不要在未确认安全环境时使用该参数。

## 待确认问题

- 后续低速运动测试使用 `movej` 还是 `movej_canfd` 作为第一步？
- 是否已有官方 SDK、协议文档或上一任开发者留下的最小控制示例？
- RM75 七轴运动学参数、关节限位和逆解策略后续如何确定？
