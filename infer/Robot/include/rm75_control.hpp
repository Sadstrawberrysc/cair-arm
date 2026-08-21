#pragma once

#include <cstdint>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include <realman_kinematics.hpp>

// 控制状态机：由 Rm75ControlLaw::Step 在每个 10 ms 周期更新。
enum class Rm75SupervisorState {
    // 初始化/仅观测/已握手但静止。
    kInitializing,
    kObserve,
    kArmed,
    // 悬空接近、已检测到接触、将轴向力稳定到目标附近。
    kApproach,
    kContact,
    kForceSettle,
    // Tool-X 扫描、Trigger 横向对齐、锁存后的 RZ 旋转对齐。
    kScan,
    kTriggerAlign,
    kRotateAlign,
    // Redis 命令失效时保持、固定阈值回退、不可恢复故障。
    kHold,
    kRetreat,
    kFault,
};

const char* ToString(Rm75SupervisorState state);

struct ControlIntent {
    // 视觉端预测的横向误差（m）和绕 Tool-Z 的角误差（deg）。
    double model_y_m = 0.0;
    double model_rz_deg = 0.0;
    // 本周期目标轴向力，压向组织的约定符号为负。
    double desired_force_n = -1.0;
    // 视觉阶段：-1 未指定，0 Scan，1 Trigger，2 Action。
    int phase_index = -1;
    // true 允许接近/力控；terminate 仅结束本轮，回到 Armed，不退出进程。
    bool action_enabled = false;
    bool terminate = false;
    // phase=2 中视觉端确认血管丢失后请求恢复：保持 Tool-X/RZ 为零，
    // 按丢失前锁存的分割左右方向驱动 Tool-Y 恢复。
    bool recovery_mode = false;
    // 分割掩膜最近窗口的左右多数，仅保留用于诊断：-1/0 未知，1 左，2 右。
    int mask_lr_majority = 0;
    // Redis 的单调命令序号，用于避免重复累计 RZ 命令。
    std::uint64_t sequence = 0;
};

struct Rm75ControlConfig {
    // RM75 生产闭环的控制参数集中在本结构中。main_rm75 只注入本次
    // 标定得到的 Tool/TCP 坐标关系，不再逐项覆盖这些算法参数。
    // ===== 1. 控制周期、接触门与 wrench 安全门 =====
    double cycle_s = 0.010;                 // 主控制周期（s）
    // 目标 Tool-Z 力（N）；负号表示压向组织。
    double desired_force_n = -2.0;
    // |Fz| 达到该值后锁存“已接触”（N）。
    double contact_threshold_n = 0.99;
    // 补偿后 Tool-X/Y 与 Tool-Z 的独立力门（N）。
    double force_limit_xy_n = 50.0;
    // 旧式固定阈值回退。当前生产力控使用下方“相对目标力卸载”，默认关闭。
    bool force_retract_enabled = false;     // 是否启用旧式固定阈值回退
    // 轴向力达到此值时进入固定阈值卸载；独立硬门仍由 force_limit_z_n 控制。
    double force_retract_threshold_z_n = 20.0; // 固定阈值回退进入力（N）
    // 卸载至该值以下回到普通导纳；应高于目标力且低于进入阈值，形成滞回。
    double force_retract_release_z_n = 1.2; // 固定阈值回退释放力（N）
    double retract_direction_tool_z = 0.0; // 旧式回退方向；默认关闭时不使用
    double retract_distance_m = 0.005;      // 旧式回退最大距离（m）
    double retract_speed_m_s = 0.002;       // 旧式回退速度（m/s）
    double force_limit_z_n = 50.0;          // 补偿后 Tool-Z 硬门（N）
    double torque_limit_nm = 5.0;           // 补偿后 Tx/Ty/Tz 硬门（N·m）

    // ===== 2. Tool-Z 导纳、超力卸载与重新接触 =====
    // 导纳模型 M*a + D*v = F_target - F_measured 的虚拟质量/阻尼。
    double force_virtual_mass = 3.0;
    double force_virtual_damping = 20.0;
    // 相对目标力的超力卸载：额外压入超过该裕量时，暂停横向/角运动并执行 -Tool-Z。
    bool target_force_unload_enabled = true; // 是否启用相对目标力的卸载层
    // Fz 比目标力额外压入该值时启动 -Tool-Z 卸载（N）。当前 -2 N
    // 目标下，须低于 -3 N 才进入卸载，避免正常力控波动频繁触发。
    double target_force_unload_margin_n = 1.0;
    // 力连续超过卸载门的最短时间（s）。短暂噪声或接触瞬态不会触发卸载。
    double target_force_unload_entry_duration_s = 0.50;
    // 超力误差达到减速力带时的最大卸载速度；回到目标力时线性降至零。
    double target_force_unload_speed_m_s = 0.01; // 最大卸载速度（m/s）
    double target_force_unload_deceleration_band_n = 1.0; // 卸载减速力带（N）
    // 力回到目标后，先将残余 -Tool-Z 速度制动至零，再恢复普通导纳。
    double target_force_unload_stop_acceleration_m_s2 = 0.05; // 卸载制动加速度（m/s²）
    // 制动后仅在 action 仍开启时，以纯 +Tool-Z 重接触；idle 时取消恢复并保持 Z 零速。
    double target_force_reacquire_speed_m_s = 0.005; // 失去接触后的 +Tool-Z 重接近速度
    double target_force_recovery_stable_duration_s = 0.5; // 恢复普通导纳前稳定时间
    // 限制普通导纳的速度变化，避免恢复时在单周期内以最大速度直接反向。
    double max_force_axis_acceleration_m_s2 = 0.01; // 普通导纳 Tool-Z 速度变化上限

    // ===== 3. wrench/contact Kalman 与接触点驱动姿态 =====
    // 旧六轴滤波器。硬 wrench 门始终先检查未滤波输入，再将结果用于控制。
    bool legacy_wrench_filter_enabled = true; // 对补偿 wrench 做 Kalman 平滑
    double legacy_wrench_filter_measurement_noise = 0.0002; // wrench Kalman 测量噪声 R
    double legacy_wrench_filter_process_noise = 0.000001;   // wrench Kalman 过程噪声 Q
    bool legacy_contact_filter_enabled = true; // 对 STL 接触点做 Kalman 平滑
    double legacy_contact_filter_measurement_noise = 0.0002; // contact Kalman 测量噪声 R
    double legacy_contact_filter_process_noise = 0.0000005;  // contact Kalman 过程噪声 Q

    // 旧变量 delta_ry 实际合成为 Tool-X/Roll 旋转，沿用其有效物理含义。
    bool legacy_contact_roll_enabled = true; // 接触点偏移驱动 Tool-X/Roll 姿态导纳
    double legacy_contact_roll_virtual_mass = 2.0; // 接触 Roll 导纳虚拟质量
    double legacy_contact_roll_virtual_damping = 10.0; // 接触 Roll 导纳阻尼
    double legacy_contact_roll_scale = 1.5; // 旧 delta_ry 到 Tool-X/Roll 的比例
    double legacy_contact_roll_max_velocity_rad_s = 1.0; // Roll 导纳速度上限
    bool legacy_contact_roll_limits_enabled = true; // 是否再受公共角速度限制

    // ===== 4. 悬空接近、接触后 Tool-Z 与角速度 =====
    // 悬空接近使用 +Tool-Z 2 cm/s；接触后切换到下方的导纳速度上限。
    double approach_speed_m_s = 0.020;
    double approach_direction_tool_z = 1.0;
    // 接触导纳的 Tool-Z 速度上限。
    double max_force_axis_speed_m_s = 0.002; // 普通接触导纳 Tool-Z 最大速度
    // 视觉 rotate-align 的 Tool-RZ 使用独立速度上限，不与接触点驱动的
    // Roll/Pitch 共用公共角速度包络。
    double max_model_rz_speed_rad_s = 5.0 * M_PI / 180.0; // 视觉 Tool-RZ 最大速度
    // Trigger 锁存 rotate-align 后，按每周期实际执行的 |delta_rz| 累计。
    // 达到该额度后只停止 Tool-RZ，其他平移和力控维度继续运行。
    double maximum_total_model_rz_rotation_rad = 120.0 * M_PI / 180.0; // Trigger 后累计 RZ 额度
    // 旋转中丢失血管后的固定 Tool-Y 恢复速度：图像左侧对应 -Tool-Y，
    // 图像右侧对应 +Tool-Y。此 Tool-Y 指进入 RZ 旋转前锁存的方向，
    // 不随恢复时的当前 RZ 姿态一起旋转。
    double recovery_tool_y_speed_m_s = 0.0002;
    // 每次进入 recovery 后的 Tool-Y 累计搜索距离上限。到达上限
    // 后只停止 Tool-Y，保持 recovery 和 Tool-Z 力控，不恢复 RZ。
    double maximum_recovery_tool_y_distance_m = 0.020;
    double max_angular_speed_rad_s = 2.0 * M_PI / 180.0; // 接触 Roll/Pitch 公共角速度上限

    // ===== 5. 视觉 Tool-Y 居中 =====
    // 当前 RM75 Tool-Y 与视觉模型训练时的 Y 方向相反；用此符号显式转换。
    double model_y_direction = -1.0; // 视觉 y 到实际 Tool-Y 的符号映射
    // 比例速度律：v_y = direction × gain × y；每周期位移为 v_y × cycle_s。
    // 比例律在误差较大时移动较快，并在接近中心时自然减速。
    double model_y_velocity_gain_per_s = 1.0;
    // 独立限制 Tool-Y 速度：10 ms 周期下 0.5 mm/周期等效 5 cm/s，
    // 避免视觉异常值在提高增益后形成过大的单周期横移。
    double maximum_model_y_step_m = 0.0005; // 单周期 Tool-Y 最大步长（m）
    // 按 m 进入 Scan phase 后的 Tool-Y 速度缩放。接近与 force_settle 阶段
    // 保持完整居中速度；扫描时降速，避免横向修正抢占 Tool-X 扫描轨迹。
    double scan_y_scale = 0.8;
    // 实际探头 TCP 沿 Tool-Y 落后于最新 ServoJ 模型目标时，仅暂停 Y 参考积分；
    // 暂停/恢复使用滞回，给机械臂追赶时间并避免阈值附近反复切换。
    double visual_y_tracking_pause_error_m = 0.020; // 实际 TCP 落后该 Tool-Y 误差时暂停积分
    double visual_y_tracking_resume_error_m = 0.010; // 落后回到该值以下时恢复积分
    // 若跟踪丢失主要来自 Tool-Y，可在达到全局故障前按实际反馈重置笛卡尔/关节参考；
    // 其他方向的跟踪故障仍会终止运行。
    double visual_y_tracking_rebase_error_m = 0.005; // Tool-Y 主导误差达到该值时重置参考
    double visual_y_tracking_rebase_dominance_ratio = 0.8; // Tool-Y 在总误差中的最小占比
    // 旋转阶段仍降低横向速度，但保留足够的实时居中能力。
    double rotate_align_y_scale = 0.3; // rotate-align 阶段的 Tool-Y 速度缩放
    double model_rz_gain = 0.05; // 视觉 rz（deg）到本周期 RZ 修正的比例
    double contact_pitch_gain_rad_per_m = 0.0; // 保留的接触点 Pitch 增益，0 为关闭
    // 视觉 Tool-Y 使用旧六轴的瞬时力门：|Fz| 必须超过设定值。
    // 当前目标力为 -2 N，门限必须留有裕量，否则目标力附近会反复关闭 Y。
    double visual_y_enable_force_n = 1.2; // |Fz| 超过该值后允许视觉 Tool-Y
    // 为配置/日志兼容保留；当前瞬时 Tool-Y 力门不累计该时长。
    double visual_y_force_stable_duration_s = 0.5; // 仅保留给日志/兼容，不参与当前门控

    // ===== 6. Tool-X 扫描与 Trigger/RZ 对齐状态机 =====
    // 扫描准备计时可启动的最低接触力。
    double scan_start_force_n = 2.0; // 扫描稳定计时的最低 |Fz|（N）
    // 轴向力在目标力附近的该误差带内连续保持设定时间后，才开始 Tool-X 扫描。
    double scan_force_tolerance_n = 0.8; // 相对目标力的扫描允许误差带（N）
    double scan_force_stable_duration_s = 0.5; // 力需连续稳定的时间（s）
    double scan_speed_m_s = 0.010; // Tool-X 扫描速度（m/s）
    // 0 表示 Tool-X 扫描连续运行，终止由视觉 terminate、Ctrl+C 或故障决定。
    double maximum_scan_distance_m = 0.0;
    double scan_alignment_tolerance_m = 0.0008; // 允许扫描的 |视觉 Y| 阈值（m）
    double scan_direction_tool_x = -1.0; // 扫描方向，通常为 -Tool-X
    double trigger_alignment_tolerance_m = 0.008; // Trigger 阶段允许的 |视觉 Y| 阈值
    double trigger_alignment_stable_duration_s = 0.4; // Trigger 对齐锁存前稳定时间

    // ===== 7. Base → Arm_Tip → Tool/Sensor → Probe TCP 坐标链 =====
    // 控制器位姿表示 Base → Arm_Tip；控制增量定义在当前与传感器重合的 Tool 坐标。
    // 该固定旋转把 Tool 向量映射到 Arm_Tip 后，再合成控制器目标位姿。
    Eigen::Matrix3d rotation_pose_from_tool = Eigen::Matrix3d::Identity();
    // 探头 TCP 在物理 Tool 坐标中的位置。当前假定 Arm_Tip 与 Tool 原点重合，
    // 坐标轴差异由 rotation_pose_from_tool 表示。
    Eigen::Vector3d probe_tcp_tool_m = Eigen::Vector3d::Zero();
};

struct Rm75ControlInput {
    // 当前“理想笛卡尔参考”位姿，格式 [x,y,z,rx,ry,rz]，位置 m、角度 rad。
    Eigen::Matrix<double, 6, 1> current_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    // 已完成偏置、重力、质心和 tare 补偿，并表达在 Tool/Sensor 的 wrench。
    Eigen::Matrix<double, 6, 1> compensated_wrench_tool =
        Eigen::Matrix<double, 6, 1>::Zero();
    // STL 估计的接触点，相对 Probe TCP 的 Tool/Sensor 坐标（m）。
    Eigen::Vector3d contact_point_probe_m = Eigen::Vector3d::Zero();
    double robot_model_position_error_m = 0.0;
    // 实际 TCP 对模型目标的 Tool-Y 投影误差；X/Z 误差不会阻塞 Y 对齐。
    double robot_model_tool_y_error_m = 0.0;
    bool robot_valid = false;   // 最新机器人状态是否可用
    bool wrench_valid = false;  // 最新补偿 wrench 是否可用
    bool contact_valid = false; // 本周期接触点估计是否有效
};

struct Rm75ControlOutput {
    // `desired_pose` 是送往 IK 的 Base→Arm_Tip 绝对目标；`requested_delta`
    // 是本周期在 Tool 坐标下请求的 [dx,dy,dz,droll,dpitch,drz]。
    Eigen::Matrix<double, 6, 1> desired_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> requested_delta =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> control_wrench_tool =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Vector3d filtered_contact_point_probe_m = Eigen::Vector3d::Zero();
    Rm75SupervisorState state = Rm75SupervisorState::kInitializing;
    bool command_motion = false; // 是否需要规划并提交新的 ServoJ 目标
    bool request_retract = false; // 请求旧式固定阈值回退
    bool target_force_unloading = false; // 当前是否在超力卸载
    bool target_force_recovering = false; // 当前是否在卸载后的制动/重新接触
    bool visual_y_tracking_paused = false; // Tool-Y 是否因实际落后而暂停
    bool recovery_search_active = false; // 是否处于血管丢失恢复状态
    int recovery_locked_mask_side = 0; // 锁存的丢失前侧别：1 左，2 右
    double recovery_tool_y_velocity_m_s = 0.0; // 恢复期间固定 Tool-Y 速度
    double recovery_search_distance_m = 0.0; // 当次 recovery 已积分的 Tool-Y 距离
    bool recovery_distance_limit_reached = false; // 是否已到达搜索距离上限
    double accumulated_model_rz_rotation_rad = 0.0; // Trigger 后已执行的累计 RZ
    bool completed = false; // 本轮是否正常结束（terminate 或扫描里程结束）
    std::string completion_reason;
    std::string fault;
};

// 确定性的笛卡尔控制律：不直接访问硬件，只输出状态机与笛卡尔目标；输出仍须
// 经过 Rm75ServoPlanner 的关节限位、速度和奇异性检查后才能发送 ServoJ。
class Rm75ControlLaw {
public:
    explicit Rm75ControlLaw(Rm75ControlConfig config = {});

    void Reset();
    const Rm75ControlConfig& Config() const { return config_; }
    Rm75ControlOutput Step(const Rm75ControlInput& input,
                           const ControlIntent& intent,
                           bool motion_armed);

private:
    void ResetMotionState(bool clear_command_memory);
    Eigen::Matrix<double, 6, 1> FilterWrench(
        const Eigen::Matrix<double, 6, 1>& wrench);
    Eigen::Vector3d FilterContactPoint(const Eigen::Vector3d& point);

    Rm75ControlConfig config_;
    bool config_valid_ = false; // 构造时检查参数范围后的结果
    // 一旦检测到力接触，保持导纳状态直至显式 idle/reset；力暂时低于接触门时，
    // 不得重新以固定速度撞向同一表面。
    bool contact_latched_ = false;
    bool target_force_unloading_ = false; // 相对目标力卸载状态
    bool target_force_release_pending_ = false; // 达到释放力后等待制动完成
    bool target_force_recovering_ = false; // 制动/重新接触阶段
    bool target_force_contact_reacquired_ = false; // 重新满足接触门
    double target_force_unload_overload_time_s_ = 0.0; // 连续超力计时
    bool wrench_filter_initialized_ = false;
    Eigen::Matrix<double, 6, 1> filtered_wrench_ =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> wrench_filter_covariance_ =
        Eigen::Matrix<double, 6, 1>::Zero();
    bool contact_filter_initialized_ = false;
    Eigen::Vector3d filtered_contact_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d contact_filter_covariance_ = Eigen::Vector3d::Zero();
    double force_axis_velocity_m_s_ = 0.0; // Tool-Z 导纳当前内部速度
    double target_force_recovery_stable_time_s_ = 0.0;
    double contact_roll_velocity_rad_s_ = 0.0;
    double scan_distance_m_ = 0.0; // 已累计的 Tool-X 扫描距离
    double visual_y_force_stable_time_s_ = 0.0;
    double scan_force_stable_time_s_ = 0.0;
    double trigger_alignment_stable_time_s_ = 0.0;
    bool force_settled_ = false; // 是否已满足扫描前力稳定带
    bool visual_y_enabled_ = false; // 当前是否允许视觉 Tool-Y
    bool visual_y_tracking_paused_ = false; // Tool-Y 跟踪滞后暂停锁存
    bool scan_latched_ = false; // 扫描阶段锁存
    bool rotation_latched_ = false; // Trigger 后 rotate-align 锁存
    int active_scan_phase_ = -1;
    std::uint64_t active_rz_sequence_ =
        std::numeric_limits<std::uint64_t>::max();
    double remaining_model_rz_rad_ = 0.0;
    double accumulated_model_rz_rotation_rad_ = 0.0;
    bool recovery_search_active_ = false;
    int last_valid_mask_side_ = 0;
    int recovery_locked_mask_side_ = 0;
    double recovery_search_distance_m_ = 0.0;
    // Base 坐标中的“旋转前 Tool-Y”单位向量；在 rotate-align
    // 从未锁存切换为锁存时记录，专供丢失恢复平移使用。
    bool pre_rotation_tool_y_valid_ = false;
    Eigen::Vector3d pre_rotation_tool_y_base_ = Eigen::Vector3d::UnitY();
};

// ===== 七轴数值 IK 与 ServoJ 规划模块 =====
// 七轴笛卡尔到关节规划器：控制律产生笛卡尔目标，规划器在提交 ServoJ 前执行
// RM75 关节限位、速度/加速度与奇异性约束。
enum class Rm75PlanError {
    kNone,
    kNonFiniteInput,
    kPreviousStepOutOfBounds,
    kJacobianFailure,
    kJointLimit,
    kJointLimitMargin,
    kSingularity,
};

const char* Rm75PlanErrorString(Rm75PlanError error);

struct Rm75ServoPlannerConfig {
    // 关节规划参数属于 ServoJ/IK 模块；main_rm75 不再覆盖这些值。
    int period_ms = 10; // ServoJ 规划周期（ms）
    double damping = 0.001; // 阻尼最小二乘 IK 的 λ
    double joint_limit_warning_deg = 10.0; // 距关节限位该角度内记录预警
    double joint_limit_stop_deg = 3.0; // 距关节限位该角度内拒绝规划
    double singularity_warning_deg = 5.0; // 奇异性预警角度裕量
    double minimum_task_singular_value = 1e-5; // Jacobian 最小奇异值门
    // 每个 10 ms 周期允许使用官方最大关节速度的 100%。
    // 速度比例乘周期不得超过最小下发间隔。
    double minimum_dispatch_gap_ms = 10.0; // 相邻 ServoJ 下发的最小时间间隔
    double joint_speed_scale = 1.0; // 官方最大关节速度比例
    double max_joint_accel_deg_s2 = 90.0; // 相邻周期关节速度变化上限
    bool allow_near_singularity = false; // true 仅用于诊断，生产入口保持 false
};

// 运行循环使用的硬件量程与跟踪故障门。它们不属于控制律公式，也不属于
// main_rm75 的启动职责，因此与控制/规划配置一起集中在本模块定义。
struct Rm75RuntimeSafetyConfig {
    // 未补偿传感器读数的量程门；在 Kalman/标定补偿前检查。
    double raw_force_limit_n = 50.0;
    double raw_torque_limit_nm = 5.0;
    // 实际反馈相对 ServoJ 模型目标的跟踪故障门。
    double max_tracking_joint_error_deg = 20.0;
    double max_tracking_position_error_mm = 25.0;
    // 0 表示不单独以笛卡尔姿态残差终止；关节与位置误差仍受监督。
    double max_tracking_orientation_error_deg = 0.0;
    // 悬空接近相对起点的 Probe-TCP 行程和姿态包络。
    double maximum_no_contact_approach_distance_m = 0.005;
    // 0 表示关闭独立姿态 excursion 门；关节/IK 门仍保持有效。
    double maximum_orientation_excursion_deg = 5.0;
};

struct Rm75ServoPlan {
    // 规划成功后提交给 ServoJ 的关节目标、其 FK 模型位姿和本周期关节增量。
    Eigen::Matrix<double, 7, 1> target_joints =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 1> model_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 7, 1> joint_delta =
        Eigen::Matrix<double, 7, 1>::Zero();
    Rm75PlanError error = Rm75PlanError::kNone; // 失败类型
    bool valid = false; // 是否可安全下发
    bool near_joint_limit = false; // 是否进入关节限位预警区
    bool near_singularity = false; // 是否进入奇异预警区
    double minimum_joint_margin_deg = 0.0; // 至最近关节限位的裕量
    std::string detail; // 面向日志的补充诊断
};

class Rm75ServoPlanner {
public:
    explicit Rm75ServoPlanner(Rm75ServoPlannerConfig config = {});

    Rm75ServoPlan Plan(
        const Eigen::Matrix<double, 7, 1>& current_joints,
        const Eigen::Matrix<double, 6, 1>& current_pose,
        const Eigen::Matrix<double, 6, 1>& desired_pose,
        const Eigen::Matrix<double, 7, 1>& previous_joint_delta =
            Eigen::Matrix<double, 7, 1>::Zero());

    Eigen::Matrix<double, 6, 1> PoseFromJoints(
        const Eigen::Matrix<double, 7, 1>& joints);
    bool NearSingularity(const Eigen::Matrix<double, 7, 1>& joints) const;
    const Rm75ServoPlannerConfig& Config() const { return config_; }
    const RMKinematics& Kinematics() const { return kinematics_; }

private:
    Eigen::Matrix<double, 6, 7> GeometricJacobian(
        const Eigen::Matrix<double, 7, 1>& joints);

    Rm75ServoPlannerConfig config_;
    RMKinematics kinematics_;
};
