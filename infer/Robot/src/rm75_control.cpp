#include <rm75_control.hpp>

#include <algorithm>
#include <cmath>

namespace {

double Clamp(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(maximum, value));
}

bool Finite(const Eigen::Matrix<double, 6, 1>& value) {
    return value.array().isFinite().all();
}

bool ValidConfig(const Rm75ControlConfig& config) {
    const double values[] = {
        config.cycle_s,
        config.desired_force_n,
        config.contact_threshold_n,
        config.force_limit_xy_n,
        config.force_retract_threshold_z_n,
        config.force_retract_release_z_n,
        config.force_limit_z_n,
        config.torque_limit_nm,
        config.force_virtual_mass,
        config.force_virtual_damping,
        config.legacy_wrench_filter_measurement_noise,
        config.legacy_wrench_filter_process_noise,
        config.legacy_contact_filter_measurement_noise,
        config.legacy_contact_filter_process_noise,
        config.legacy_contact_roll_virtual_mass,
        config.legacy_contact_roll_virtual_damping,
        config.legacy_contact_roll_scale,
        config.legacy_contact_roll_max_velocity_rad_s,
        config.approach_speed_m_s,
        config.approach_direction_tool_z,
        config.max_linear_speed_m_s,
        config.max_angular_speed_rad_s,
        config.model_y_gain,
        config.model_rz_gain,
        config.contact_pitch_gain_rad_per_m,
        config.scan_speed_m_s,
        config.maximum_scan_distance_m,
        config.scan_alignment_tolerance_m,
        config.scan_direction_tool_x};
    for (double value : values) {
        if (!std::isfinite(value)) return false;
    }
    if (!config.probe_tcp_tool_m.array().isFinite().all()) return false;
    return config.cycle_s > 0.0
        && config.desired_force_n < 0.0
        && std::abs(config.desired_force_n) <= config.force_limit_z_n
        && config.contact_threshold_n >= 0.0
        && config.force_limit_xy_n > 0.0
        // Retraction is optional. When enabled it is an overload response,
        // not the normal contact-entry transition, so it must not pre-empt
        // the configured force target.
        && (!config.force_retract_enabled
            || (config.force_retract_threshold_z_n
                    > std::abs(config.desired_force_n)
                && config.force_retract_release_z_n
                    > std::abs(config.desired_force_n)
                && config.force_retract_release_z_n
                    < config.force_retract_threshold_z_n
                && config.force_retract_threshold_z_n
                    <= config.force_limit_z_n))
        && config.force_limit_z_n > 0.0
        && config.torque_limit_nm > 0.0
        && config.force_virtual_mass > 0.0
        && config.force_virtual_damping >= 0.0
        && config.legacy_wrench_filter_measurement_noise > 0.0
        && config.legacy_wrench_filter_process_noise > 0.0
        && config.legacy_contact_filter_measurement_noise > 0.0
        && config.legacy_contact_filter_process_noise > 0.0
        && config.legacy_contact_roll_virtual_mass > 0.0
        && config.legacy_contact_roll_virtual_damping >= 0.0
        && config.legacy_contact_roll_scale >= 0.0
        && config.legacy_contact_roll_max_velocity_rad_s > 0.0
        && config.approach_speed_m_s >= 0.0
        && std::abs(config.approach_direction_tool_z) <= 1.0
        && config.max_linear_speed_m_s > 0.0
        && config.max_angular_speed_rad_s > 0.0
        && config.scan_speed_m_s >= 0.0
        && config.maximum_scan_distance_m >= 0.0
        && config.scan_alignment_tolerance_m >= 0.0
        && std::abs(config.scan_direction_tool_x) <= 1.0;
}

Eigen::Matrix3d RotationFromEuler(const Eigen::Vector3d& euler) {
    return (Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

Eigen::Vector3d EulerFromRotation(const Eigen::Matrix3d& rotation) {
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    euler.y() = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
    const double cosine_pitch = std::cos(euler.y());
    if (std::abs(cosine_pitch) > 1e-9) {
        euler.x() = std::atan2(rotation(2, 1), rotation(2, 2));
        euler.z() = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        euler.z() = std::atan2(-rotation(0, 1), rotation(1, 1));
    }
    return euler;
}

}  // namespace

const char* ToString(Rm75SupervisorState state) {
    switch (state) {
        case Rm75SupervisorState::kInitializing: return "initializing";
        case Rm75SupervisorState::kObserve: return "observe";
        case Rm75SupervisorState::kArmed: return "armed";
        case Rm75SupervisorState::kApproach: return "approach";
        case Rm75SupervisorState::kContact: return "contact";
        case Rm75SupervisorState::kScan: return "scan";
        case Rm75SupervisorState::kHold: return "hold";
        case Rm75SupervisorState::kRetreat: return "retreat";
        case Rm75SupervisorState::kFault: return "fault";
    }
    return "unknown";
}

Rm75ControlLaw::Rm75ControlLaw(Rm75ControlConfig config)
    : config_(config), config_valid_(ValidConfig(config)) {}

void Rm75ControlLaw::Reset() {
    contact_latched_ = false;
    wrench_filter_initialized_ = false;
    filtered_wrench_.setZero();
    wrench_filter_covariance_.setZero();
    contact_filter_initialized_ = false;
    filtered_contact_point_.setZero();
    contact_filter_covariance_.setZero();
    ResetMotionState(true);
}

void Rm75ControlLaw::ResetMotionState(bool clear_command_memory) {
    force_axis_velocity_m_s_ = 0.0;
    contact_roll_velocity_rad_s_ = 0.0;
    scan_distance_m_ = 0.0;
    active_scan_phase_ = -1;
    remaining_model_y_m_ = 0.0;
    remaining_model_rz_rad_ = 0.0;
    if (clear_command_memory) {
        completed_scan_phase_ = -1;
        active_correction_sequence_ =
            std::numeric_limits<std::uint64_t>::max();
    }
}

Eigen::Matrix<double, 6, 1> Rm75ControlLaw::FilterWrench(
    const Eigen::Matrix<double, 6, 1>& wrench) {
    if (!config_.legacy_wrench_filter_enabled) return wrench;
    const double measurement_noise =
        config_.legacy_wrench_filter_measurement_noise;
    const double process_noise = config_.legacy_wrench_filter_process_noise;
    if (!wrench_filter_initialized_) {
        filtered_wrench_ = wrench;
        wrench_filter_covariance_.setConstant(process_noise);
        wrench_filter_initialized_ = true;
    }
    for (int axis = 0; axis < 6; ++axis) {
        const double predicted_covariance =
            wrench_filter_covariance_[axis] + process_noise;
        const double gain = predicted_covariance
            / (predicted_covariance + measurement_noise);
        filtered_wrench_[axis] +=
            gain * (wrench[axis] - filtered_wrench_[axis]);
        wrench_filter_covariance_[axis] =
            predicted_covariance - gain * predicted_covariance;
    }
    return filtered_wrench_;
}

Eigen::Vector3d Rm75ControlLaw::FilterContactPoint(
    const Eigen::Vector3d& point) {
    if (!config_.legacy_contact_filter_enabled) return point;
    const double measurement_noise =
        config_.legacy_contact_filter_measurement_noise;
    const double process_noise = config_.legacy_contact_filter_process_noise;
    if (!contact_filter_initialized_) {
        filtered_contact_point_ = point;
        contact_filter_covariance_.setConstant(process_noise);
        contact_filter_initialized_ = true;
    }
    for (int axis = 0; axis < 3; ++axis) {
        const double predicted_covariance =
            contact_filter_covariance_[axis] + process_noise;
        const double gain = predicted_covariance
            / (predicted_covariance + measurement_noise);
        filtered_contact_point_[axis] +=
            gain * (point[axis] - filtered_contact_point_[axis]);
        contact_filter_covariance_[axis] =
            predicted_covariance - gain * predicted_covariance;
    }
    return filtered_contact_point_;
}

Rm75ControlOutput Rm75ControlLaw::Step(const Rm75ControlInput& input,
                                       const ControlIntent& intent,
                                       bool motion_armed) {
    Rm75ControlOutput output;
    output.desired_pose = input.current_pose;

    if (!config_valid_) {
        output.state = Rm75SupervisorState::kFault;
        output.fault = "control_configuration_invalid";
        ResetMotionState(false);
        return output;
    }
    if (!std::isfinite(intent.model_y_m)
        || !std::isfinite(intent.model_rz_deg)
        || !std::isfinite(intent.desired_force_n)
        || intent.desired_force_n >= 0.0
        || std::abs(intent.desired_force_n) > config_.force_limit_z_n
        || intent.phase_index < -1 || intent.phase_index > 2) {
        output.state = Rm75SupervisorState::kFault;
        output.fault = "control_intent_invalid";
        ResetMotionState(false);
        return output;
    }

    if (!input.robot_valid || !Finite(input.current_pose)) {
        output.state = Rm75SupervisorState::kFault;
        output.fault = "robot_state_invalid_or_stale";
        ResetMotionState(false);
        return output;
    }
    if (!input.wrench_valid || !Finite(input.compensated_wrench_tool)) {
        output.state = Rm75SupervisorState::kHold;
        output.fault = "wrench_invalid_or_stale";
        ResetMotionState(false);
        return output;
    }

    const Eigen::Vector3d measured_force =
        input.compensated_wrench_tool.head<3>();
    const Eigen::Vector3d measured_torque =
        input.compensated_wrench_tool.tail<3>();
    const bool axial_force_over_limit =
        std::abs(measured_force.z()) > config_.force_limit_z_n;
    const bool non_axial_over_limit =
        std::abs(measured_force.x()) > config_.force_limit_xy_n
        || std::abs(measured_force.y()) > config_.force_limit_xy_n
        || measured_torque.cwiseAbs().maxCoeff() > config_.torque_limit_nm;
    if (axial_force_over_limit || non_axial_over_limit) {
        output.state = Rm75SupervisorState::kFault;
        output.fault = "wrench_safety_limit_exceeded";
        ResetMotionState(false);
        return output;
    }

    output.control_wrench_tool = FilterWrench(input.compensated_wrench_tool);
    const Eigen::Vector3d force = output.control_wrench_tool.head<3>();
    const Eigen::Vector3d contact_measurement = input.contact_valid
        ? input.contact_point_probe_m
        : Eigen::Vector3d::Zero();
    output.filtered_contact_point_probe_m =
        FilterContactPoint(contact_measurement);

    if (!motion_armed) {
        output.state = Rm75SupervisorState::kObserve;
        contact_latched_ = false;
        ResetMotionState(false);
        return output;
    }
    if (intent.terminate || !intent.action_enabled) {
        output.state = Rm75SupervisorState::kHold;
        output.completed = intent.terminate;
        if (intent.terminate) output.completion_reason = "terminate_requested";
        contact_latched_ = false;
        ResetMotionState(false);
        return output;
    }

    // Dedicated unloading must not wait for the legacy low-pass filter. The
    // filter is part of normal force behavior, not the overload response.
    if (config_.force_retract_enabled
        && std::abs(measured_force.z())
        >= config_.force_retract_threshold_z_n) {
        output.state = Rm75SupervisorState::kRetreat;
        output.request_retract = true;
        output.fault = "wrench_unload_requested";
        force_axis_velocity_m_s_ = 0.0;
        return output;
    }

    const double dt = config_.cycle_s;
    const double max_linear_step = config_.max_linear_speed_m_s * dt;
    const double max_angular_step = config_.max_angular_speed_rad_s * dt;
    if (std::abs(force.z()) >= config_.contact_threshold_n) {
        contact_latched_ = true;
    }
    const bool in_contact = contact_latched_;
    if (intent.sequence != active_correction_sequence_) {
        active_correction_sequence_ = intent.sequence;
        remaining_model_y_m_ = config_.model_y_gain * intent.model_y_m;
        remaining_model_rz_rad_ =
            config_.model_rz_gain * intent.model_rz_deg * M_PI / 180.0;
    }
    if ((intent.phase_index == 0 || intent.phase_index == 2)
        && intent.phase_index == completed_scan_phase_) {
        output.state = Rm75SupervisorState::kHold;
        output.completed = true;
        output.completion_reason = "scan_distance_completed";
        force_axis_velocity_m_s_ = 0.0;
        return output;
    }

    double delta_z = 0.0;
    if (in_contact) {
        const double requested_force = std::isfinite(intent.desired_force_n)
            ? intent.desired_force_n
            : config_.desired_force_n;
        const double acceleration =
            (force.z() - requested_force
             - config_.force_virtual_damping * force_axis_velocity_m_s_)
            / config_.force_virtual_mass;
        force_axis_velocity_m_s_ = Clamp(
            force_axis_velocity_m_s_ + acceleration * dt,
            -config_.max_linear_speed_m_s,
            config_.max_linear_speed_m_s);
        delta_z = force_axis_velocity_m_s_ * dt;
    } else {
        force_axis_velocity_m_s_ = 0.0;
        delta_z = config_.approach_direction_tool_z
            * config_.approach_speed_m_s * dt;
    }

    double delta_x = 0.0;
    const bool scan_phase = intent.phase_index == 0 || intent.phase_index == 2;
    if (scan_phase && in_contact
        && std::abs(intent.model_y_m) <= config_.scan_alignment_tolerance_m) {
        if (active_scan_phase_ != intent.phase_index) {
            active_scan_phase_ = intent.phase_index;
            scan_distance_m_ = 0.0;
        }
        const double remaining =
            std::max(0.0, config_.maximum_scan_distance_m - scan_distance_m_);
        const double scan_step = std::min(config_.scan_speed_m_s * dt, remaining);
        delta_x = config_.scan_direction_tool_x * scan_step;
        if (remaining <= 0.0) {
            output.state = Rm75SupervisorState::kHold;
            output.completed = true;
            output.completion_reason = "scan_distance_completed";
            completed_scan_phase_ = intent.phase_index;
            force_axis_velocity_m_s_ = 0.0;
            return output;
        }
    } else if (!scan_phase) {
        scan_distance_m_ = 0.0;
        active_scan_phase_ = -1;
        completed_scan_phase_ = -1;
    }

    const double model_y_step = Clamp(remaining_model_y_m_,
                                      -max_linear_step,
                                      max_linear_step);
    Eigen::Vector3d translation_delta;
    translation_delta << delta_x,
        model_y_step,
        Clamp(delta_z, -max_linear_step, max_linear_step);
    if (translation_delta.norm() > max_linear_step) {
        translation_delta *= max_linear_step / translation_delta.norm();
    }
    if (in_contact) {
        // Keep the admittance damping state consistent with the XYZ step that
        // survives the combined Cartesian speed limit.
        force_axis_velocity_m_s_ = translation_delta.z() / dt;
    }
    remaining_model_y_m_ -= translation_delta.y();
    if (scan_phase && in_contact
        && std::abs(intent.model_y_m) <= config_.scan_alignment_tolerance_m) {
        scan_distance_m_ += std::abs(translation_delta.x());
    }

    double delta_roll = 0.0;
    if (config_.legacy_contact_roll_enabled) {
        const double contact_y_m = output.filtered_contact_point_probe_m.y();
        if (contact_y_m == 0.0) {
            contact_roll_velocity_rad_s_ = 0.0;
        } else {
            const double acceleration =
                (1000.0 * contact_y_m
                 - config_.legacy_contact_roll_virtual_damping
                     * contact_roll_velocity_rad_s_)
                / config_.legacy_contact_roll_virtual_mass;
            delta_roll = contact_roll_velocity_rad_s_ * dt
                + 0.5 * acceleration * dt * dt;
            const double next_velocity =
                contact_roll_velocity_rad_s_ + acceleration * dt;
            contact_roll_velocity_rad_s_ =
                config_.legacy_contact_roll_limits_enabled
                ? Clamp(next_velocity,
                        -config_.legacy_contact_roll_max_velocity_rad_s,
                        config_.legacy_contact_roll_max_velocity_rad_s)
                : next_velocity;
            delta_roll *= config_.legacy_contact_roll_scale;
        }
    } else {
        contact_roll_velocity_rad_s_ = 0.0;
    }
    double delta_pitch = 0.0;
    if (input.contact_valid && std::isfinite(input.contact_point_probe_m.y())) {
        delta_pitch = Clamp(
            -config_.contact_pitch_gain_rad_per_m * input.contact_point_probe_m.y(),
            -max_angular_step,
            max_angular_step);
    }
    double delta_rz = Clamp(remaining_model_rz_rad_,
                            -max_angular_step,
                            max_angular_step);
    // Keep the ordinary pitch/RZ envelope available even when the explicitly
    // selected legacy contact-roll path is unbounded.
    const double combined_angular_step = std::sqrt(
        (config_.legacy_contact_roll_limits_enabled
             ? delta_roll * delta_roll : 0.0)
        + delta_pitch * delta_pitch + delta_rz * delta_rz);
    if (combined_angular_step > max_angular_step) {
        const double scale = max_angular_step / combined_angular_step;
        if (config_.legacy_contact_roll_limits_enabled) delta_roll *= scale;
        delta_pitch *= scale;
        delta_rz *= scale;
    }
    remaining_model_rz_rad_ -= delta_rz;

    output.requested_delta << translation_delta.x(),
        translation_delta.y(), translation_delta.z(),
        delta_roll, delta_pitch, delta_rz;

    // All control deltas are expressed in the current tool frame. Compose the
    // rigid transform and keep the calibrated probe TCP fixed during pure
    // orientation corrections. Directly adding these values to base XYZ and
    // Euler angles would move along the wrong axis whenever the tool is tilted.
    const Eigen::Matrix3d rotation_base_from_tool =
        RotationFromEuler(input.current_pose.tail<3>());
    const Eigen::Matrix3d incremental_rotation =
        (Eigen::AngleAxisd(delta_rz, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(delta_pitch, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(delta_roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Matrix3d desired_rotation =
        rotation_base_from_tool * incremental_rotation;
    const Eigen::Vector3d current_probe_tcp_base =
        input.current_pose.head<3>()
        + rotation_base_from_tool * config_.probe_tcp_tool_m;
    const Eigen::Vector3d desired_probe_tcp_base =
        current_probe_tcp_base + rotation_base_from_tool * translation_delta;
    output.desired_pose.head<3>() = desired_probe_tcp_base
        - desired_rotation * config_.probe_tcp_tool_m;
    output.desired_pose.tail<3>() = EulerFromRotation(desired_rotation);
    output.command_motion = output.requested_delta.cwiseAbs().maxCoeff() > 0.0;
    output.state = in_contact
        ? (intent.phase_index >= 0 ? Rm75SupervisorState::kScan
                                   : Rm75SupervisorState::kContact)
        : Rm75SupervisorState::kApproach;
    return output;
}

// Seven-axis ServoJ planning implementation. It shares the RM75 control
// translation unit while retaining its public planner interface.
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kEulerSingularityEpsilon = 1e-9;

const Eigen::Matrix<double, 7, 1> kJointMaximumSpeedDegS =
    (Eigen::Matrix<double, 7, 1>()
         << 180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0)
        .finished();

Eigen::Vector3d ControllerEuler(const Eigen::Matrix3d& rotation) {
    const double sy = std::clamp(-rotation(2, 0), -1.0, 1.0);
    const double ry = std::asin(sy);
    const double cy = std::cos(ry);
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    euler.y() = ry;
    if (std::abs(cy) > kEulerSingularityEpsilon) {
        euler.x() = std::atan2(rotation(2, 1), rotation(2, 2));
        euler.z() = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        euler.z() = std::atan2(-rotation(0, 1), rotation(1, 1));
    }
    return euler;
}

Eigen::Matrix<double, 6, 1> PoseError(
    const Eigen::Matrix<double, 6, 1>& target,
    const Eigen::Matrix<double, 6, 1>& current) {
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target.head<3>() - current.head<3>();
    const auto rotation = [](const Eigen::Vector3d& euler) {
        return (Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    };
    const Eigen::Matrix3d rotation_error =
        rotation(target.tail<3>()) * rotation(current.tail<3>()).transpose();
    const Eigen::AngleAxisd angle_axis(rotation_error);
    error.tail<3>() = angle_axis.axis() * angle_axis.angle();
    return error;
}

bool Near(double value, double target, double threshold) {
    return std::abs(value - target) < threshold;
}

}  // namespace

const char* Rm75PlanErrorString(Rm75PlanError error) {
    switch (error) {
        case Rm75PlanError::kNone: return "none";
        case Rm75PlanError::kNonFiniteInput: return "non_finite_input";
        case Rm75PlanError::kPreviousStepOutOfBounds:
            return "previous_step_out_of_bounds";
        case Rm75PlanError::kJacobianFailure: return "jacobian_failure";
        case Rm75PlanError::kJointLimit: return "joint_limit";
        case Rm75PlanError::kJointLimitMargin: return "joint_limit_margin";
        case Rm75PlanError::kSingularity: return "singularity";
    }
    return "unknown";
}

Rm75ServoPlanner::Rm75ServoPlanner(Rm75ServoPlannerConfig config)
    : config_(config) {}

Eigen::Matrix<double, 6, 1> Rm75ServoPlanner::PoseFromJoints(
    const Eigen::Matrix<double, 7, 1>& joints) {
    Eigen::Matrix<double, 7, 1> mutable_joints = joints;
    Eigen::Matrix4d transform;
    kinematics_.GetKinematics(transform, mutable_joints);
    Eigen::Matrix<double, 6, 1> pose;
    pose.head<3>() = transform.block<3, 1>(0, 3);
    pose.tail<3>() = ControllerEuler(transform.block<3, 3>(0, 0));
    return pose;
}

Eigen::Matrix<double, 6, 7> Rm75ServoPlanner::GeometricJacobian(
    const Eigen::Matrix<double, 7, 1>& joints) {
    Eigen::Matrix<double, 6, 7> jacobian;
    Eigen::Matrix<double, 7, 1> mutable_joints = joints;
    kinematics_.GetJacobian(jacobian, mutable_joints);
    return jacobian;
}

bool Rm75ServoPlanner::NearSingularity(
    const Eigen::Matrix<double, 7, 1>& joints) const {
    const double threshold = config_.singularity_warning_deg;
    if (threshold <= 0.0) return false;
    const double j2 = joints[1] * kRadToDeg;
    const double j3 = joints[2] * kRadToDeg;
    const double j4 = joints[3] * kRadToDeg;
    const double j5 = joints[4] * kRadToDeg;
    const double j6 = joints[5] * kRadToDeg;
    const bool type1 = Near(j2, 0.0, threshold) && Near(j6, 0.0, threshold);
    const bool type2 = Near(j4, 0.0, threshold);
    const bool type3 = Near(j2, 0.0, threshold)
        && (Near(j3, 90.0, threshold) || Near(j3, -90.0, threshold));
    const bool type4 = Near(j6, 0.0, threshold)
        && (Near(j5, 90.0, threshold) || Near(j5, -90.0, threshold));
    return type1 || type2 || type3 || type4;
}

Rm75ServoPlan Rm75ServoPlanner::Plan(
    const Eigen::Matrix<double, 7, 1>& current_joints,
    const Eigen::Matrix<double, 6, 1>& current_pose,
    const Eigen::Matrix<double, 6, 1>& desired_pose,
    const Eigen::Matrix<double, 7, 1>& previous_joint_delta) {
    Rm75ServoPlan result;
    if (config_.period_ms <= 0
        || !std::isfinite(config_.minimum_dispatch_gap_ms)
        || config_.minimum_dispatch_gap_ms <= 0.0
        || !std::isfinite(config_.joint_speed_scale)
        || config_.joint_speed_scale <= 0.0
        || config_.joint_speed_scale > 1.0
        || config_.joint_speed_scale * config_.period_ms
               > config_.minimum_dispatch_gap_ms + 1e-12
        || !std::isfinite(config_.max_joint_accel_deg_s2)
        || config_.max_joint_accel_deg_s2 < 0.0) {
        result.error = Rm75PlanError::kNonFiniteInput;
        result.detail =
            "invalid planner period, dispatch gap, speed scale or acceleration";
        return result;
    }
    if (!current_joints.array().isFinite().all()
        || !current_pose.array().isFinite().all()
        || !desired_pose.array().isFinite().all()
        || !previous_joint_delta.array().isFinite().all()) {
        result.error = Rm75PlanError::kNonFiniteInput;
        result.detail = "joint, pose or previous delta contains non-finite data";
        return result;
    }

    const Eigen::Matrix<double, 6, 7> jacobian =
        GeometricJacobian(current_joints);
    if (!jacobian.array().isFinite().all()) {
        result.error = Rm75PlanError::kJacobianFailure;
        result.detail = "controller-pose Jacobian contains non-finite data";
        return result;
    }
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> jacobian_svd(jacobian);
    const double minimum_singular_value =
        jacobian_svd.singularValues().minCoeff();
    const bool task_space_singularity =
        !std::isfinite(minimum_singular_value)
        || minimum_singular_value < config_.minimum_task_singular_value;
    const Eigen::Matrix<double, 6, 6> normal =
        jacobian * jacobian.transpose()
        + config_.damping * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(normal);
    if (solver.info() != Eigen::Success) {
        result.error = Rm75PlanError::kJacobianFailure;
        result.detail = "damped Jacobian factorization failed";
        return result;
    }
    Eigen::Matrix<double, 7, 1> delta =
        jacobian.transpose() * solver.solve(PoseError(desired_pose, current_pose));
    if (!delta.array().isFinite().all()) {
        result.error = Rm75PlanError::kJacobianFailure;
        result.detail = "damped Jacobian solve produced non-finite delta";
        return result;
    }

    const double period_s = config_.period_ms / 1000.0;
    const Eigen::Matrix<double, 7, 1> maximum_step =
        config_.joint_speed_scale
        * kJointMaximumSpeedDegS * period_s * kDegToRad;
    if ((previous_joint_delta.cwiseAbs().array()
         > maximum_step.array() + 1e-12).any()) {
        result.error = Rm75PlanError::kPreviousStepOutOfBounds;
        result.detail =
            "previous joint delta exceeds the configured speed envelope";
        return result;
    }
    double scale = 1.0;
    for (int joint = 0; joint < 7; ++joint) {
        if (std::abs(delta[joint]) > maximum_step[joint]) {
            scale = std::min(scale, maximum_step[joint] / std::abs(delta[joint]));
        }
    }
    delta *= scale;

    if (config_.max_joint_accel_deg_s2 > 0.0) {
        const double maximum_change =
            config_.max_joint_accel_deg_s2
            * config_.joint_speed_scale * config_.joint_speed_scale
            * kDegToRad * period_s * period_s;
        for (int joint = 0; joint < 7; ++joint) {
            delta[joint] = previous_joint_delta[joint]
                + std::clamp(delta[joint] - previous_joint_delta[joint],
                             -maximum_change,
                             maximum_change);
        }
    }
    // Preserve the public speed invariant after acceleration limiting too.
    delta = delta.cwiseMax(-maximum_step).cwiseMin(maximum_step);

    result.target_joints = current_joints + delta;
    result.joint_delta = delta;
    result.minimum_joint_margin_deg = std::numeric_limits<double>::infinity();
    for (int joint = 0; joint < 7; ++joint) {
        if (result.target_joints[joint] < kinematics_.joint_min[joint]
            || result.target_joints[joint] > kinematics_.joint_max[joint]) {
            result.error = Rm75PlanError::kJointLimit;
            result.detail = "planned target exceeds J" + std::to_string(joint + 1)
                + " joint limit";
            return result;
        }
        const double margin = std::min(
            result.target_joints[joint] - kinematics_.joint_min[joint],
            kinematics_.joint_max[joint] - result.target_joints[joint])
            * kRadToDeg;
        result.minimum_joint_margin_deg =
            std::min(result.minimum_joint_margin_deg, margin);
    }
    result.near_joint_limit =
        result.minimum_joint_margin_deg < config_.joint_limit_warning_deg;
    if (result.minimum_joint_margin_deg < config_.joint_limit_stop_deg) {
        result.error = Rm75PlanError::kJointLimitMargin;
        std::ostringstream detail;
        detail << "minimum joint margin " << result.minimum_joint_margin_deg
               << " deg is below stop threshold " << config_.joint_limit_stop_deg;
        result.detail = detail.str();
        return result;
    }

    result.near_singularity =
        task_space_singularity || NearSingularity(result.target_joints);
    if (result.near_singularity && !config_.allow_near_singularity) {
        result.error = Rm75PlanError::kSingularity;
        std::ostringstream detail;
        detail << "planned target is near a singularity; task sigma_min="
               << minimum_singular_value;
        result.detail = detail.str();
        return result;
    }
    result.model_pose = PoseFromJoints(result.target_joints);
    result.valid = true;
    return result;
}
