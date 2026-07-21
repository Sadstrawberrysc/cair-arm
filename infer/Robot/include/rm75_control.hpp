#pragma once

#include <cstdint>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include <realman_kinematics.hpp>

enum class Rm75SupervisorState {
    kInitializing,
    kObserve,
    kArmed,
    kApproach,
    kContact,
    kScan,
    kHold,
    kRetreat,
    kFault,
};

const char* ToString(Rm75SupervisorState state);

struct ControlIntent {
    double model_y_m = 0.0;
    double model_rz_deg = 0.0;
    double desired_force_n = -1.0;
    int phase_index = -1;
    bool action_enabled = false;
    bool terminate = false;
    std::uint64_t sequence = 0;
};

struct Rm75ControlConfig {
    double cycle_s = 0.020;
    double desired_force_n = -1.0;
    double contact_threshold_n = 0.8;
    double force_limit_xy_n = 20.0;
    // Optional RM75 commissioning retreat. Disabled for the legacy six-axis
    // axial-admittance path, where overload unloading comes from the same
    // force-error equation as normal contact motion.
    bool force_retract_enabled = false;
    // Enter a controlled tool-Z unloading retreat at this axial magnitude.
    // force_limit_z_n remains the independent hard-stop threshold.
    double force_retract_threshold_z_n = 20.0;
    // Return from the dedicated retreat to normal admittance control after
    // unloading below this magnitude. It must remain above the force target
    // and below force_retract_threshold_z_n to provide hysteresis.
    double force_retract_release_z_n = 1.2;
    double force_limit_z_n = 20.0;
    double torque_limit_nm = 4.0;

    double force_virtual_mass = 3.0;
    double force_virtual_damping = 20.0;
    // Optional legacy six-axis filters. Hard wrench limits are always checked
    // against the unfiltered input before these values are used for control.
    bool legacy_wrench_filter_enabled = false;
    double legacy_wrench_filter_measurement_noise = 0.0002;
    double legacy_wrench_filter_process_noise = 0.000001;
    bool legacy_contact_filter_enabled = false;
    double legacy_contact_filter_measurement_noise = 0.0002;
    double legacy_contact_filter_process_noise = 0.0000005;

    // The legacy variable named delta_ry was composed as a tool-X rotation.
    // Preserve that effective behavior rather than the misleading old name.
    bool legacy_contact_roll_enabled = false;
    double legacy_contact_roll_virtual_mass = 2.0;
    double legacy_contact_roll_virtual_damping = 10.0;
    double legacy_contact_roll_scale = 1.5;
    double legacy_contact_roll_max_velocity_rad_s = 1.0;
    bool legacy_contact_roll_limits_enabled = true;
    double approach_speed_m_s = 0.0;
    double approach_direction_tool_z = 0.0;
    // Contact-admittance Tool-Z speed limit, applied before the final XYZ
    // vector safety envelope.
    double max_force_axis_speed_m_s = 0.005;
    // Combined XYZ translation-speed limit. Simultaneous X/Z motion shares
    // this vector envelope.
    double max_linear_speed_m_s = 0.005;
    double max_angular_speed_rad_s = 2.0 * M_PI / 180.0;

    double model_y_gain = 0.5;
    double model_rz_gain = 0.05;
    double contact_pitch_gain_rad_per_m = 0.0;
    // Coarse contact-force floor for allowing the scan-ready timer to run.
    double scan_start_force_n = 2.0;
    // Tool-X scanning starts only after the signed axial force remains within
    // this band around desired_force_n for scan_force_stable_duration_s.
    double scan_force_tolerance_n = 0.3;
    double scan_force_stable_duration_s = 0.5;
    double scan_speed_m_s = 0.001;
    double maximum_scan_distance_m = 0.005;
    double scan_alignment_tolerance_m = 0.0008;
    double scan_direction_tool_x = -1.0;

    // The controller pose describes Base -> Arm_Tip. Control increments use
    // the physical Tool frame, which is coincident with the force-sensor axes
    // on the current installation. This fixed rotation maps Tool vectors into
    // Arm_Tip coordinates before composing a controller pose target.
    Eigen::Matrix3d rotation_pose_from_tool = Eigen::Matrix3d::Identity();
    // Position of the configured probe TCP expressed in physical Tool axes.
    // The current installation assumes coincident Arm_Tip/Tool origins; their
    // axes may differ by rotation_pose_from_tool.
    Eigen::Vector3d probe_tcp_tool_m = Eigen::Vector3d::Zero();
};

struct Rm75ControlInput {
    Eigen::Matrix<double, 6, 1> current_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> compensated_wrench_tool =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Vector3d contact_point_probe_m = Eigen::Vector3d::Zero();
    bool robot_valid = false;
    bool wrench_valid = false;
    bool contact_valid = false;
};

struct Rm75ControlOutput {
    Eigen::Matrix<double, 6, 1> desired_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> requested_delta =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> control_wrench_tool =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Vector3d filtered_contact_point_probe_m = Eigen::Vector3d::Zero();
    Rm75SupervisorState state = Rm75SupervisorState::kInitializing;
    bool command_motion = false;
    bool request_retract = false;
    bool completed = false;
    std::string completion_reason;
    std::string fault;
};

// Deterministic Cartesian control law. It never communicates with hardware;
// its output must still pass through the RM75 joint/singularity safety layer.
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
    bool config_valid_ = false;
    // Once force contact is detected, remain in admittance control until the
    // action is explicitly disarmed/reset. Dropping below the entry threshold
    // must not restart fixed-speed approach into the same surface.
    bool contact_latched_ = false;
    bool wrench_filter_initialized_ = false;
    Eigen::Matrix<double, 6, 1> filtered_wrench_ =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> wrench_filter_covariance_ =
        Eigen::Matrix<double, 6, 1>::Zero();
    bool contact_filter_initialized_ = false;
    Eigen::Vector3d filtered_contact_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d contact_filter_covariance_ = Eigen::Vector3d::Zero();
    double force_axis_velocity_m_s_ = 0.0;
    double contact_roll_velocity_rad_s_ = 0.0;
    double scan_distance_m_ = 0.0;
    double scan_force_stable_time_s_ = 0.0;
    bool scan_latched_ = false;
    int active_scan_phase_ = -1;
    std::uint64_t active_correction_sequence_ =
        std::numeric_limits<std::uint64_t>::max();
    double remaining_model_y_m_ = 0.0;
    double remaining_model_rz_rad_ = 0.0;
};

// Seven-axis Cartesian-to-joint planner. It shares this module with the
// deterministic control law, matching the original robot_control boundary:
// control produces a Cartesian target and the planner applies RM75 joint,
// acceleration and singularity constraints before any ServoJ submission.
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
    int period_ms = 20;
    double damping = 0.001;
    double joint_limit_warning_deg = 10.0;
    double joint_limit_stop_deg = 3.0;
    double singularity_warning_deg = 5.0;
    double minimum_task_singular_value = 1e-5;
    // The default 20 ms loop reserves half of the official speed. In general,
    // joint_speed_scale * period_ms must not exceed minimum_dispatch_gap_ms.
    // The square of the scale is also applied to acceleration changes.
    double minimum_dispatch_gap_ms = 10.0;
    double joint_speed_scale = 0.5;
    double max_joint_accel_deg_s2 = 90.0;
    bool allow_near_singularity = false;
};

struct Rm75ServoPlan {
    Eigen::Matrix<double, 7, 1> target_joints =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 1> model_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 7, 1> joint_delta =
        Eigen::Matrix<double, 7, 1>::Zero();
    Rm75PlanError error = Rm75PlanError::kNone;
    bool valid = false;
    bool near_joint_limit = false;
    bool near_singularity = false;
    double minimum_joint_margin_deg = 0.0;
    std::string detail;
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
