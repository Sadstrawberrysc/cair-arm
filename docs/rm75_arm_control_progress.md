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
- 已新增预设关节位姿工具 `infer/Robot/src/arm_preset_pose.cpp` 和构建脚本 `infer/Robot/tools/build_arm_preset_pose.sh`。
- `arm_preset_pose` 默认只读当前状态；支持 `--list-presets`、`--preset ready_verified`、`--target-deg d1,d2,d3,d4,d5,d6,d7`，并默认 dry-run。
- `arm_preset_pose` 增加 `--max-joint-delta-deg` 限制，防止从当前姿态直接跳到距离过大的目标关节位姿。
- 已验证 `infer/Robot/tools/build_arm_preset_pose.sh /tmp/arm_preset_pose` 可编译通过，CMake 默认目标也可完整构建。
- 已将机械臂测试工具脚本的默认输出从 `/tmp` 改到项目内 `infer/Robot/build/`，便于直接在项目目录中执行。
- 已将 `home_current` 改为动态预设：每次运行 `arm_preset_pose` 时读取当前七轴关节角，并将当前姿态作为 `home_current` 目标。
- 已为 `arm_preset_pose` 新增 `--allow-multistep`，允许将距离较远的目标关节位姿按 `--max-joint-delta-deg` 自动拆成多段 `MoveJ()`；未显式启用时仍拒绝大跳变。
- 已明确 `arm_preset_pose` 的主逻辑：先读取当前七轴关节位姿作为初始状态，再通过 `--preset` 或 `--target-deg` 输入目标关节位姿；默认只 dry-run，确认后才加 `--execute` 发运动命令。
- `arm_preset_pose` 默认连接 `192.168.50.254:8080`，日常使用不需要额外输入 `--ip` 和 `--port`；这两个参数仅作为临时覆盖保留。
- 已增强 `arm_preset_pose` 的 dry-run 输出：如果目标被拆成多段，会先打印每一个 waypoint 的七轴目标角，便于执行前逐段确认。
- 已增强 `arm_preset_pose` 的状态读取输出：每次读取当前姿态后都会打印 `current_as_target_deg` 和一行可复制的 dry-run 命令，方便把当前姿态快速作为后续目标位姿。
- 已为 `arm_preset_pose` 增加执行后目标误差检查：默认要求最大关节误差不超过 `1°`，可通过 `--max-final-error-deg` 调整；超过阈值时程序返回失败。
- 已新增末端位姿增量测试工具 `infer/Robot/src/arm_pose_delta.cpp` 和构建脚本 `infer/Robot/tools/build_arm_pose_delta.sh`。
- `arm_pose_delta` 默认读取当前 6D 末端位姿作为初始状态，生成小增量目标；默认命令为 `MoveJP`，可通过 `--command movel` 切换到 `MoveL`。
- `arm_pose_delta` 默认 dry-run，不发送运动命令；支持 `--axis x|y|z|rx|ry|rz`、`--delta-cm`、`--delta-deg`、`--return-start` 和执行后误差检查。
- 已调整 `arm_pose_delta` 的位姿显示：位置单独输出为 `position_cm`，姿态单独输出为 `rotation_deg`，避免把位置和角度混在同一个标签里。
- 已精简 `arm_pose_delta` 默认输出：正常运行只显示 `current_pose` 和 `target_pose`；底层 JSON、轨迹 ack、最终误差等仅用于内部检查或错误提示。
- 已进一步精简 `arm_pose_delta --execute` 输出：dry-run 时显示 `current_pose` 和 `target_pose`，执行后正常情况下只显示执行完成后的 `current_pose`。
- 已执行 `MoveJP` 末端 X 方向位移测试：`./arm_pose_delta --command movejp --axis x --delta-cm 10 --execute`，程序完成后正常返回当前末端位姿。
- 已完成 `MoveL` 末端 X 方向小位移测试：dry-run 目标 X 从约 `36.393 cm` 到 `36.893 cm`，执行后 X 到约 `36.8673 cm`，末端直线位姿控制链路验证通过。
- 已确认 `realman_command.hpp/.cpp` 中关节相关接口均为 RM75 七轴：`ReadJ()`、`ReadArmState()`、`MoveJ()`、`ServoJ()` 使用 `Eigen::Matrix<double,7,1>`；`MoveJP/MoveL/ReadL` 保持 6D 末端位姿是正确设计。
- 已开始 7 轴运动学适配前置工作：新增 `arm_estimate_jacobian`，通过真机小关节扰动估计实际 `6x7` Jacobian，避免在缺少 RM75 官方 DH/URDF 参数时硬改旧 6 轴解析公式。
- 已根据睿尔曼官方 RM75 参数页提取 RM75 为 7 自由度、关节范围以及 MDH 参数；官方页面说明 `模型角度 = 关节角度 + offset`，当前 offset 均为 0。
- 已将 `realman_kinematics.hpp/.cpp` 从旧 6 轴手写解析式改为 RM75 官方 MDH 参数驱动的 7 轴 FK、数值 `6x7` Jacobian 和阻尼最小二乘 `GetNextJoints()`。
- 已新增 `arm_check_kinematics`，用于读取真机当前 7 轴状态后打印官方 MDH FK 和 `6x7` Jacobian，后续需与控制器返回的末端位姿对比验证坐标系一致性。
- 已执行一次只读 `arm_check_kinematics` 真机验证：当前官方 MDH 直接 FK 位置与控制器返回末端位置误差约 `0.144 m`，说明还需要继续确认 MDH 坐标约定、基座/工具坐标偏置或控制器 TCP 定义，暂不能直接把该 FK 用进闭环控制。
- 已根据睿尔曼 JSON 协议确认存在当前工具坐标系/工作坐标系查询命令：`get_current_tool_frame`、`get_current_work_frame`。
- 已新增只读坐标系查询工具 `infer/Robot/src/arm_read_frames.cpp` 和构建脚本 `infer/Robot/tools/build_arm_read_frames.sh`，用于读取当前工具坐标系、工作坐标系、工具列表、工作坐标系列表和当前机械臂状态。
- 已通过 CMake 和独立脚本编译 `infer/Robot/build/arm_read_frames`；由于当前会话外部网络执行审批/额度受限，尚未由 Codex 直接完成真机坐标系查询，需要在终端手动运行。
- 已通过 `arm_read_frames` 真机确认：当前工具坐标系为 `Arm_Tip`，工具 `pose=[0,0,0,0,0,0]`；当前工作坐标系为 `Base`，工作坐标 `pose=[0,0,0,0,0,0]`。因此此前 FK 误差不是用户配置的 TCP 或工作坐标偏置造成。
- 已根据当前姿态反推 RM75 控制器返回的 `Arm_Tip` 相对官方 MDH 第 7 轴末端坐标系存在固定几何偏移，约为末端局部 Z 方向 `0.143998 m`。
- 已将 `realman_kinematics` 的 `tool_offset` 设置为 `[0, 0, 0.143998] m`，并重新运行 `arm_check_kinematics` 真机验证：FK 位置误差从约 `0.144 m` 降至约 `5.3e-06 m`。
- 已在多个不同姿态下重复验证 `arm_check_kinematics`，目前最大 `fk_position_error_norm_m` 约为 `1.13495e-05 m`，说明 RM75 位置 FK 和控制器返回末端位置已稳定对齐到约 `0.012 mm` 量级。
- 已增强 `arm_estimate_jacobian`：真机扰动估计 `6x7` Jacobian 后，会同时打印当前 `realman_kinematics` 模型 Jacobian、两者差值，以及位置/姿态 Jacobian 误差范数。
- 首次 Jacobian 对比显示位置 Jacobian 误差范数约 `0.00937`，但姿态 Jacobian 误差范数约 `0.915`；原因是 `RMKinematics::GetJacobian()` 输出的是几何角速度 Jacobian，而控制器 `pose[3:6]` 使用 `R = Rz(rz) * Ry(ry) * Rx(rx)` 的欧拉角参数。
- 已按当前策略保留 `realman_kinematics` 的原始几何 Jacobian 输出，不把控制器欧拉角口径写入核心运动学文件；`arm_estimate_jacobian` 里单独计算并输出 `model_controller_pose_jacobian_6x7`，用于和真机 `pose` 差分结果对比。
- 第二次 Jacobian 对比中姿态误差范数降至约 `0.288`，但仍受控制器姿态回读 `0.001 rad` 分辨率影响；`0.5°` 差分时姿态估计会被量化成约 `0.114592` 的倍数。
- 已将 `arm_estimate_jacobian` 改为中心差分：每个关节依次运动到 `+delta` 和 `-delta`，用 `(pose_plus - pose_minus)/(2*delta)` 估计 Jacobian；同时将 `--delta-deg` 上限从 `1°` 放宽到 `3°`，便于降低姿态量化误差。
- 执行 `arm_estimate_jacobian --delta-deg 2 --execute` 前读取到 `arm_err=4099`，程序已拒绝运动；当时 J4 约为 `135°`，正好贴近 RM75 官方 J4 正限位，不能进行 `+2°/-2°` 中心差分测试。
- 已增强 `arm_estimate_jacobian` 的安全检查：启动后打印 `arm_err_hex` 和每个关节到上下限的余量；若当前姿态无法满足 `±delta` 中心差分，则直接拒绝执行。
- 已在安全姿态下完成 `arm_estimate_jacobian --delta-deg 2 --execute` 中心差分验证：位置 Jacobian 误差范数约 `0.00144`，控制器 pose 口径姿态 Jacobian 误差范数约 `0.02836`，模型与真机差分结果已基本一致。
- 已新增第一版基于 Jacobian 的位置小步控制工具 `infer/Robot/src/arm_jacobian_position_delta.cpp` 和构建脚本 `infer/Robot/tools/build_arm_jacobian_position_delta.sh`。
- `arm_jacobian_position_delta` 只做 XYZ 位置小步，不控制姿态；默认 dry-run，使用 RM75 位置 Jacobian 计算一个低风险 MoveJ 目标，执行前会检查机器人错误状态、关节限位和单次最大关节步长。
- 已执行第一次 Jacobian 位置小步控制：Z 方向目标 `+0.2 cm`，实际 Z 从约 `59.7939 cm` 到 `59.9662 cm`，约完成 `+0.1723 cm`；X/Y 漂移约为毫米以下量级。
- 已将 `arm_jacobian_position_delta` 升级为可迭代位置闭环：新增 `--iterations N` 和 `--tolerance-cm CM`，每轮执行后重新读取当前位姿、重新计算 Jacobian 和下一步 MoveJ 目标；默认仍为 1 轮。
- 已完成 Z/X/Y 三个单轴正方向位置闭环小步测试，均在 2 轮内进入 `0.03 cm` 容差：Z 剩余误差约 `0.0203 cm`，X 剩余误差约 `0.0207 cm`，Y 剩余误差约 `0.0188 cm`。
- 已将 `arm_jacobian_position_delta --delta-cm` 扩展为兼容两种格式：单轴标量 `--axis z --delta-cm 0.2`，或三维组合位移 `--delta-cm "0.1,0.1,0.1"`；旧命令保持可用。
- 已完成三维组合位置闭环小步测试，剩余误差 `remaining_error_cm=0.029464`，约 `0.29464 mm`，进入 `0.03 cm` 容差。
- 已完成负方向/混合方向位置闭环小步测试，剩余误差约 `0.0239094 cm` 和 `0.0122821 cm`，均进入 `0.03 cm` 容差。
- 已开始大距离位置闭环测试，当前三次结果剩余误差约 `0.0360258 cm`、`0.0401115 cm`、`0.0256461 cm`，均小于 `0.05 cm`，说明 1-2 cm 级别位置闭环已进入约 `0.5 mm` 容差。
- 已将 `arm_jacobian_position_delta` 升级为自动分段大距离移动：`--max-segment-delta-cm` 控制单段最大位移，`--max-total-delta-cm` 控制总位移上限；例如 5 cm 目标会自动拆成多个 1-2 cm 级别小段，每段闭环收敛后再进入下一段。
- 已完成 Z 方向 5 cm 自动分段大距离测试：正方向最终误差约 `0.0375225 cm`，负方向最终误差约 `0.0342013 cm`，均进入 `0.05 cm` 容差。
- 三维组合大距离测试中出现中间段未严格收敛的情况：例如 `segment_error_cm=0.321202`、`0.558694`，说明组合方向比单轴更容易在中间段留下毫米级残差。
- 已将 `arm_jacobian_position_delta` 的分段策略改为“进展式分段”：中间段新增 `--max-segment-error-cm` 和 `--min-progress-cm`，只要段误差不过大且整体剩余距离明显下降，就允许继续下一段；最终段仍使用 `--max-final-error-cm` 严格检查。
- 直接三维组合路径继续出现进展不足：`segment_error_cm=0.703713`、`remaining_error_cm=4.33086`、`progress_cm=0.046493`，说明 direct 三维路径在当前姿态下效率较低。
- 已为 `arm_jacobian_position_delta` 新增 `--path-mode sequential`：当输入三维组合位移时，按 X、Y、Z 三个轴向子目标依次执行，每个轴向目标仍使用已验证的自动分段闭环；默认 `direct` 路径保持不变。
- sequential 三维大距离仍出现最终段收敛不足：例如 `segment_error_cm=1.03687`、`progress_cm=0.344246`，说明不是发散，而是每段内部迭代推进不足。
- 已为 `arm_jacobian_position_delta` 新增可调控制参数：`--iterations` 上限放宽到 12，新增 `--damping` 和 `--gain`，并输出 `planned_joint_delta_max_abs_deg` 方便判断是否被单步关节角限制截断。
- 进一步排查发现 sequential 路径存在漂移累积问题：旧逻辑在执行 Y/Z 子目标时会保留前面轴的实际漂移，导致最终误差可能累计到 1 cm 量级。
- 已修正 sequential 路径：执行第 N 个轴时，会把已完成轴 `0..N` 都锁定到最终目标坐标；三轴执行后如仍超过 `tolerance-cm`，会自动执行一次 `sequential_final_correction` 到最终目标。
- 修正 sequential 后，`"3,3,2"` 大组合仍在最后段出现约 `1.5 cm` 级误差，说明当前姿态/目标下三维大组合 Jacobian 位置路径仍可能走偏，不宜继续盲目加大增益或放宽最终阈值。
- 已为 `arm_jacobian_position_delta` 新增段回滚保护：每段开始记录起点关节和起点误差；若某次 MoveJ 后 `segment_error_cm` 比段起始误差更大超过 `--regression-margin-cm`，自动 MoveJ 回段起点并中止。
- 对较小三维组合 `"1,1,0.5"` 的测试仍出现最后段约 `0.5 cm` 级剩余误差，说明需要诊断每次 MoveJ 的实际位移方向，而不是继续盲目调 gain/damping。
- 已为 `arm_jacobian_position_delta` 增加逐步诊断输出：`actual_step_cm`、`desired_remaining_step_cm`、`step_projection_cm`、`step_lateral_cm`、`step_cos_angle`，用于判断每步是沿目标方向推进、横向漂移，还是方向错误。
- 三维组合 `"1,1,0.5"` 诊断显示最后阶段规划出的关节步长已经极小：`planned_joint_delta_max_abs_deg≈0.0086°`，实际关节最大变化约 `0.005°`，因此末端实际位移只有约 `0.006 cm` 以内，无法继续有效收敛。
- 该失败姿态下 J4/J6 均接近 `0°`，与 RM75 官方奇异位形提示中的 `q4=0`、`q6=0` 相关情况一致；当前判断为接近奇异位形导致位置 Jacobian 反解变软或方向不可靠，而不是 MoveJ 命令未执行。
- 当前结论：Jacobian 位置闭环在普通姿态下已验证有效，但在 J4/J6 接近 `0°` 的奇异附近会出现收敛停滞；后续闭环控制必须增加奇异位形检测/规避策略。
- 已在远离 J4/J6 近零奇异区域的新姿态下重新测试三维组合位置闭环：目标关节附近为 `[54.8111, 10.9464, 71.2744, 14.8921, -57.0659, -6.29209, 7.639] deg`。
- 新姿态下 `"1,1,0.5"` 组合测试恢复正常：`planned_joint_delta_max_abs_deg=0.43914`，`actual_joint_delta_max_abs_deg=0.444`，`target_joint_error_max_abs_deg≈0.00486`，说明关节目标能被控制器准确执行。
- 新姿态下末端步进方向也明显改善：`step_cos_angle≈0.8383`，最终 `final_remaining_error_cm=0.0344404`，约 `0.344 mm`，说明远离奇异区域后 Jacobian 位置闭环重新可用。
- 已按睿尔曼官方 RM75 参数页的 4 类运动奇异点更新 `arm_jacobian_position_delta` 的风险提示：`q2≈0 && q6≈0`、`q4≈0`、`q2≈0 && q3≈±90`、`q6≈0 && q5≈±90`。
- 奇异提示默认阈值为 `5°`，可通过 `--singularity-warning-deg` 调整；触发时会打印 `rm75_singularity_warning`、奇异类型和当前 J2-J6 角度，但不会自动改变运动命令。
- 后续 `"1,1,0.5"` sequential 执行中再次出现停滞：最后规划目标为 `[83.5333, 15.3263, 72.7074, 0.366198, -56.906, -0.0776461, 7.631] deg`，其中 J4≈`0.37°`、J6≈`-0.08°`，目标本身已经贴近官方奇异类型 2（`q4≈0`）。
- 该次停滞的实际步进方向很差：`step_cos_angle≈0.063`、`progress_cm≈0.0038`，说明末端几乎没有沿目标方向推进；根因判断为“迭代过程中规划目标进入奇异附近”，而不是 MoveJ 执行失败。
- 已升级 `arm_jacobian_position_delta`：不仅检测当前姿态，也检测每一步规划出的 `target_joints`；在 `--execute` 模式下，若规划目标接近官方 RM75 奇异类型，默认拒绝发送该步 MoveJ。只有显式添加 `--allow-near-singularity` 才允许继续。
- 远程相机监控、Daheng SDK 安装和抓帧验证记录已迁移到 `docs/remote_camera_monitoring.md`。

## 下一步任务

- 补测负方向位置闭环：`--axis x/y/z --delta-cm -0.2 --iterations 2 --tolerance-cm 0.03 --execute`。
- 在三维组合位移上继续做小范围正负混合测试，例如 `--delta-cm "0.1,-0.1,0.1"` 和 `--delta-cm "-0.1,0.1,-0.1"`。
- 先 dry-run 自动分段 5 cm：`arm_jacobian_position_delta --axis z --delta-cm 5 --max-segment-delta-cm 2 --iterations 3 --tolerance-cm 0.05`，确认 `segment_count` 和首段关节增量后再执行。
- 继续测试 X/Y 方向 5 cm 自动分段，或三维组合大距离如 `--delta-cm "3,3,2"`，仍保持 `--max-segment-delta-cm 2` 和 `--tolerance-cm 0.05`。
- 三维大距离建议优先使用 sequential 路径：`--delta-cm "3,3,2" --path-mode sequential --max-segment-delta-cm 2 --iterations 3 --tolerance-cm 0.05 --max-final-error-cm 0.3`。
- 修正 sequential 漂移后，重新测试：`--delta-cm "3,3,2" --path-mode sequential --max-segment-delta-cm 0.5 --iterations 12 --tolerance-cm 0.05 --max-final-error-cm 0.3 --gain 0.7 --damping 0.002 --execute`。
- 暂停直接测试 `"3,3,2"` 大组合；先退回到较小组合如 `"1,1,0.5"` 或 `"2,1,1"`，使用回滚保护确认不会走偏，再决定是否继续优化大组合路径策略。
- 下一步用诊断版先跑较小组合 `"1,1,0.5"`，重点查看 `step_projection_cm`、`step_lateral_cm` 和 `step_cos_angle`，确认每步实际运动方向。
- 继续在远离 J4/J6 近零区域的姿态下测试 `"2,1,1"`、`"2,2,1"` 等中等三维组合位移，先 dry-run，再 `--execute`。
- 后续将奇异检测从简单 J4/J6 阈值升级为基于位置 Jacobian 条件数或最小奇异值的数值检测。
- 为三维组合位置控制增加“奇异规避项”或“零空间偏置”，避免迭代过程中 J4/J6 被解算推向 `0°`。
- 若位置闭环继续稳定，再新增固定目标位置工具：输入绝对 `target_position_cm`，而不是只输入相对 `delta-cm`。
- 位置闭环稳定后，再开始姿态闭环适配；姿态部分需要保留 `realman_kinematics` 的几何 Jacobian，同时在控制工具里单独处理控制器欧拉角口径。
- 根据后续任务需求补充更多经真机验证的 `home/ready/work_start` 位姿。

## 风险/注意事项

- 真机测试前必须确认急停、拖动示教、保护停止等安全机制可用。
- 未确认坐标系和运动方向前，不进行大幅度运动。
- `main_nomove` 依赖 Redis、力传感器、接触点估计、标定文件和旧 6 轴运动学，不适合作为 RM75 七轴控制入口。
- `main_nomove` 仍然依赖 6 轴运动学和 6x6 Jacobian，不能视为已完成 RM75 七轴闭环控制适配。
- 七轴 `MoveJ()` 已完成真机小角度和 10 度关节测试；`ServoJ()` 尚未作为当前阶段入口，首次使用仍需低速、小角度、空载、安全范围内执行。
- 新增的 `arm_movej_delta` 只有在添加 `--execute` 时才会发运动命令；不要在未确认安全环境时使用该参数。
- `realman_kinematics` 当前已使用官方 RM75 MDH 数值，并通过 `[0,0,0.143998] m` 末端几何偏移与控制器末端位置稳定对齐；姿态闭环仍需注意几何角速度 Jacobian 与控制器欧拉角 `pose[3:6]` 的口径差异。

## 待确认问题

- 后续低速运动测试使用 `movej` 还是 `movej_canfd` 作为第一步？
- 是否已有官方 SDK、协议文档或上一任开发者留下的最小控制示例？
- RM75 七轴运动学参数、关节限位和逆解策略后续如何确定？
