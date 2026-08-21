#include <rm75_control.hpp>

#include <algorithm>
#include <cmath>

namespace {

double Clamp(double value, double minimum, double maximum) {
    return std::max(minimum, std::min(maximum, value));
}

double MoveToward(double value, double target, double maximum_delta) {
    if (value < target) return std::min(value + maximum_delta, target);
    if (value > target) return std::max(value - maximum_delta, target);
    return target;
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
        config.retract_direction_tool_z,
        config.retract_distance_m,
        config.retract_speed_m_s,
        config.force_limit_z_n,
        config.torque_limit_nm,
        config.force_virtual_mass,
        config.force_virtual_damping,
        config.target_force_unload_margin_n,
        config.target_force_unload_entry_duration_s,
        config.target_force_unload_speed_m_s,
        config.target_force_unload_deceleration_band_n,
        config.target_force_unload_stop_acceleration_m_s2,
        config.target_force_reacquire_speed_m_s,
        config.target_force_recovery_stable_duration_s,
        config.max_force_axis_acceleration_m_s2,
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
        config.max_force_axis_speed_m_s,
        config.max_model_rz_speed_rad_s,
        config.maximum_total_model_rz_rotation_rad,
        config.recovery_tool_y_speed_m_s,
        config.maximum_recovery_tool_y_distance_m,
        config.max_angular_speed_rad_s,
        config.model_y_direction,
        config.model_y_velocity_gain_per_s,
        config.maximum_model_y_step_m,
        config.scan_y_scale,
        config.visual_y_tracking_pause_error_m,
        config.visual_y_tracking_resume_error_m,
        config.visual_y_tracking_rebase_error_m,
        config.visual_y_tracking_rebase_dominance_ratio,
        config.rotate_align_y_scale,
        config.model_rz_gain,
        config.contact_pitch_gain_rad_per_m,
        config.visual_y_enable_force_n,
        config.visual_y_force_stable_duration_s,
        config.scan_start_force_n,
        config.scan_force_tolerance_n,
        config.scan_force_stable_duration_s,
        config.scan_speed_m_s,
        config.maximum_scan_distance_m,
        config.scan_alignment_tolerance_m,
        config.scan_direction_tool_x,
        config.trigger_alignment_tolerance_m,
        config.trigger_alignment_stable_duration_s};
    for (double value : values) {
        if (!std::isfinite(value)) return false;
    }
    if (!config.rotation_pose_from_tool.array().isFinite().all()
        || !config.probe_tcp_tool_m.array().isFinite().all()) return false;
    const Eigen::Matrix3d rotation_identity_error =
        config.rotation_pose_from_tool.transpose()
            * config.rotation_pose_from_tool
        - Eigen::Matrix3d::Identity();
    if (rotation_identity_error.norm() > 1e-6
        || std::abs(config.rotation_pose_from_tool.determinant() - 1.0)
               > 1e-6) {
        return false;
    }
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
                    <= config.force_limit_z_n
                && std::abs(std::abs(config.retract_direction_tool_z) - 1.0)
                    <= 1e-12
                && config.retract_distance_m > 0.0
                && config.retract_speed_m_s > 0.0))
        && config.force_limit_z_n > 0.0
        && config.torque_limit_nm > 0.0
        && config.force_virtual_mass > 0.0
        && config.force_virtual_damping >= 0.0
        && config.target_force_unload_margin_n >= 0.0
        && config.target_force_unload_margin_n < config.force_limit_z_n
        && config.target_force_unload_entry_duration_s >= 0.0
        && config.target_force_unload_speed_m_s > 0.0
        && config.target_force_unload_deceleration_band_n
               >= config.target_force_unload_margin_n
        && config.target_force_unload_stop_acceleration_m_s2 > 0.0
        && config.target_force_reacquire_speed_m_s > 0.0
        && config.target_force_recovery_stable_duration_s > 0.0
        && config.max_force_axis_acceleration_m_s2 > 0.0
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
        && config.max_force_axis_speed_m_s > 0.0
        && config.max_model_rz_speed_rad_s > 0.0
        && config.maximum_total_model_rz_rotation_rad > 0.0
        && config.recovery_tool_y_speed_m_s > 0.0
        && config.recovery_tool_y_speed_m_s * config.cycle_s
               <= config.maximum_model_y_step_m
        && config.maximum_recovery_tool_y_distance_m > 0.0
        && config.max_angular_speed_rad_s > 0.0
        && std::abs(std::abs(config.model_y_direction) - 1.0) <= 1e-12
        && config.maximum_model_y_step_m > 0.0
        && config.scan_y_scale >= 0.0
        && config.scan_y_scale <= 1.0
        && config.visual_y_tracking_pause_error_m > 0.0
        && config.visual_y_tracking_resume_error_m >= 0.0
        && config.visual_y_tracking_resume_error_m
               < config.visual_y_tracking_pause_error_m
        && config.visual_y_tracking_rebase_error_m > 0.0
        && config.visual_y_tracking_rebase_dominance_ratio > 0.0
        && config.visual_y_tracking_rebase_dominance_ratio <= 1.0
        && config.rotate_align_y_scale >= 0.0
        && config.rotate_align_y_scale <= 1.0
        && config.visual_y_enable_force_n >= config.contact_threshold_n
        && config.visual_y_enable_force_n <= config.force_limit_z_n
        && config.visual_y_force_stable_duration_s > 0.0
        && config.scan_start_force_n >= config.contact_threshold_n
        && config.scan_start_force_n <= config.force_limit_z_n
        && config.scan_force_tolerance_n > 0.0
        && config.scan_force_stable_duration_s > 0.0
        && config.scan_speed_m_s >= 0.0
        && config.maximum_scan_distance_m >= 0.0
        && config.scan_alignment_tolerance_m >= 0.0
        && std::abs(config.scan_direction_tool_x) <= 1.0
        && config.trigger_alignment_tolerance_m >= 0.0
        && config.trigger_alignment_stable_duration_s > 0.0;
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
        case Rm75SupervisorState::kForceSettle: return "force_settle";
        case Rm75SupervisorState::kScan: return "scan";
        case Rm75SupervisorState::kTriggerAlign: return "trigger_align";
        case Rm75SupervisorState::kRotateAlign: return "rotate_align";
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
    target_force_unloading_ = false;
    target_force_release_pending_ = false;
    target_force_recovering_ = false;
    target_force_contact_reacquired_ = false;
    target_force_unload_overload_time_s_ = 0.0;
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
    target_force_unloading_ = false;
    target_force_release_pending_ = false;
    target_force_recovering_ = false;
    target_force_contact_reacquired_ = false;
    target_force_recovery_stable_time_s_ = 0.0;
    target_force_unload_overload_time_s_ = 0.0;
    contact_roll_velocity_rad_s_ = 0.0;
    scan_distance_m_ = 0.0;
    visual_y_force_stable_time_s_ = 0.0;
    scan_force_stable_time_s_ = 0.0;
    trigger_alignment_stable_time_s_ = 0.0;
    force_settled_ = false;
    visual_y_enabled_ = false;
    visual_y_tracking_paused_ = false;
    scan_latched_ = false;
    rotation_latched_ = false;
    active_scan_phase_ = -1;
    remaining_model_rz_rad_ = 0.0;
    accumulated_model_rz_rotation_rad_ = 0.0;
    recovery_search_active_ = false;
    last_valid_mask_side_ = 0;
    recovery_locked_mask_side_ = 0;
    recovery_search_distance_m_ = 0.0;
    pre_rotation_tool_y_valid_ = false;
    pre_rotation_tool_y_base_ = Eigen::Vector3d::UnitY();
    if (clear_command_memory) {
        active_rz_sequence_ =
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
        || intent.phase_index < -1 || intent.phase_index > 2
        || intent.mask_lr_majority < -1
        || intent.mask_lr_majority > 2) {
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
    if (!std::isfinite(input.robot_model_position_error_m)
        || input.robot_model_position_error_m < 0.0
        || !std::isfinite(input.robot_model_tool_y_error_m)
        || input.robot_model_tool_y_error_m < 0.0) {
        output.state = Rm75SupervisorState::kFault;
        output.fault = "robot_tracking_error_invalid";
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
    if (input.contact_valid) {
        // 仅将真实有效的 STL 接触点送入 Kalman。新接触的第一帧会在
        // FilterContactPoint 中直接初始化状态，避免继承上一接触面的坐标。
        output.filtered_contact_point_probe_m =
            FilterContactPoint(input.contact_point_probe_m);
    } else {
        // 无接触不是“接触点位于原点”。若把 [0,0,0] 作为测量值连续更新，
        // 滤波状态会被错误拉向零，下一次接触时会产生很长的恢复滞后。
        // 因此无有效接触时清除滤波器；输出零只表示“当前没有有效接触点”。
        contact_filter_initialized_ = false;
        filtered_contact_point_.setZero();
        contact_filter_covariance_.setZero();
        output.filtered_contact_point_probe_m.setZero();
    }

    if (!motion_armed) {
        output.state = Rm75SupervisorState::kObserve;
        contact_latched_ = false;
        target_force_unloading_ = false;
        target_force_release_pending_ = false;
        target_force_recovering_ = false;
        target_force_contact_reacquired_ = false;
        ResetMotionState(false);
        return output;
    }

    const double dt = config_.cycle_s;
    const double requested_force = std::isfinite(intent.desired_force_n)
        ? intent.desired_force_n
        : config_.desired_force_n;
    if (std::abs(force.z()) >= config_.contact_threshold_n) {
        contact_latched_ = true;
    }
    const bool in_contact = contact_latched_;

    // Target-relative unloading is an independent axial safety layer. It is
    // evaluated before visual action/terminate gates so an idle or stale Redis
    // command cannot prevent unloading a valid excessive contact force.
    const bool target_force_overloaded = in_contact
        && config_.target_force_unload_enabled
        && force.z() <= requested_force - config_.target_force_unload_margin_n;
    if (!target_force_unloading_) {
        target_force_unload_overload_time_s_ = target_force_overloaded
            ? target_force_unload_overload_time_s_ + dt : 0.0;
    } else {
        target_force_unload_overload_time_s_ = 0.0;
    }
    if (!target_force_unloading_
        && target_force_unload_overload_time_s_ + 1e-12
            >= config_.target_force_unload_entry_duration_s) {
        target_force_unloading_ = true;
        target_force_unload_overload_time_s_ = 0.0;
        target_force_release_pending_ = false;
        target_force_recovering_ = false;
        target_force_contact_reacquired_ = false;
        target_force_recovery_stable_time_s_ = 0.0;
        force_settled_ = false;
        // Unloading invalidates the previous visual-Y force qualification.
        // The probe must establish contact and pass the complete force gate
        // again after braking before any lateral correction can resume.
        visual_y_enabled_ = false;
        visual_y_force_stable_time_s_ = 0.0;
        visual_y_tracking_paused_ = true;
        scan_force_stable_time_s_ = 0.0;
    } else if (target_force_unloading_) {
        if (target_force_overloaded) {
            // A renewed overload during braking returns directly to the
            // force-dependent unloading profile.
            target_force_release_pending_ = false;
        } else if (force.z() >= requested_force) {
            target_force_release_pending_ = true;
        }
    }

    // Keep all lateral/angular gates closed throughout unloading, braking,
    // contact reacquisition and force restoration.
    const bool target_force_transition_active = target_force_unloading_
        || target_force_recovering_;

    // Idle/terminate may not interrupt an active -Tool-Z unloading or its
    // zero-speed braking. It does, however, have priority over the subsequent
    // +Tool-Z contact reacquisition: recovery is allowed only while move stays
    // enabled. This prevents an idle Redis command from driving into contact.
    if ((intent.terminate || !intent.action_enabled)
        && !target_force_unloading_) {
        output.state = Rm75SupervisorState::kArmed;
        output.completed = intent.terminate;
        if (intent.terminate) output.completion_reason = "terminate_requested";
        contact_latched_ = false;
        target_force_unloading_ = false;
        target_force_release_pending_ = false;
        target_force_recovering_ = false;
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

    const double max_angular_step = config_.max_angular_speed_rad_s * dt;
    const double max_model_rz_step =
        config_.max_model_rz_speed_rad_s * dt;
    if (intent.sequence != active_rz_sequence_) {
        active_rz_sequence_ = intent.sequence;
        // RZ remains a finite correction: a new command replaces any
        // unfinished rotation and a repeated sequence only drains it.
        remaining_model_rz_rad_ =
            config_.model_rz_gain * intent.model_rz_deg * M_PI / 180.0;
    }
    double delta_z = 0.0;
    if (in_contact) {
        if (target_force_unloading_) {
            if (target_force_release_pending_) {
                // Do not let a sensor sample crossing the target flip directly
                // from retreat to positive admittance velocity. Brake to zero
                // first; ordinary admittance starts on a later cycle.
                force_axis_velocity_m_s_ = MoveToward(
                    force_axis_velocity_m_s_, 0.0,
                    config_.target_force_unload_stop_acceleration_m_s2 * dt);
                if (std::abs(force_axis_velocity_m_s_) <= 1e-12) {
                    force_axis_velocity_m_s_ = 0.0;
                    target_force_unloading_ = false;
                    target_force_release_pending_ = false;
                    if (intent.action_enabled && !intent.terminate) {
                        target_force_recovering_ = true;
                        target_force_contact_reacquired_ = false;
                        target_force_recovery_stable_time_s_ = 0.0;
                    } else {
                        // The overload has been released and braking reached
                        // zero. Idle now takes effect immediately: cancel all
                        // recovery memory and never issue a +Tool-Z step.
                        output.state = Rm75SupervisorState::kArmed;
                        output.completed = intent.terminate;
                        if (intent.terminate) {
                            output.completion_reason = "terminate_requested";
                        }
                        contact_latched_ = false;
                        ResetMotionState(false);
                        return output;
                    }
                }
            } else {
                // Approach is +Tool-Z on this installation, therefore
                // unloading is -Tool-Z. Scale the maximum 0.5 cm/s retreat
                // linearly down as the force approaches the requested target.
                const double compressive_error_n =
                    std::max(0.0, requested_force - force.z());
                const double speed_scale = Clamp(
                    compressive_error_n
                        / config_.target_force_unload_deceleration_band_n,
                    0.0, 1.0);
                force_axis_velocity_m_s_ =
                    -config_.target_force_unload_speed_m_s * speed_scale;
            }
        } else if (target_force_recovering_) {
            if (std::abs(force.z()) < config_.contact_threshold_n) {
                // The retreat detached the probe. Reacquire contact quickly
                // with pure +Tool-Z; X/Y/RZ remain suppressed.
                force_axis_velocity_m_s_ =
                    config_.target_force_reacquire_speed_m_s;
                target_force_contact_reacquired_ = false;
                target_force_recovery_stable_time_s_ = 0.0;
            } else {
                // Contact is present again. Restore the configured target with
                // low-speed admittance and its acceleration limit.
                if (!target_force_contact_reacquired_) {
                    force_axis_velocity_m_s_ = 0.0;
                    target_force_contact_reacquired_ = true;
                }
                const double acceleration =
                    (force.z() - requested_force
                     - config_.force_virtual_damping
                           * force_axis_velocity_m_s_)
                    / config_.force_virtual_mass;
                const double desired_admittance_velocity_m_s = Clamp(
                    force_axis_velocity_m_s_ + acceleration * dt,
                    -config_.max_force_axis_speed_m_s,
                    config_.max_force_axis_speed_m_s);
                force_axis_velocity_m_s_ = MoveToward(
                    force_axis_velocity_m_s_,
                    desired_admittance_velocity_m_s,
                    config_.max_force_axis_acceleration_m_s2 * dt);

                const bool force_recovered =
                    std::abs(force.z() - requested_force)
                        <= config_.scan_force_tolerance_n;
                target_force_recovery_stable_time_s_ = force_recovered
                    ? target_force_recovery_stable_time_s_ + dt : 0.0;
                if (target_force_recovery_stable_time_s_ + 1e-12
                    >= config_.target_force_recovery_stable_duration_s) {
                    target_force_recovering_ = false;
                    target_force_contact_reacquired_ = false;
                    target_force_recovery_stable_time_s_ = 0.0;
                }
            }
        } else {
            const double acceleration =
                (force.z() - requested_force
                 - config_.force_virtual_damping * force_axis_velocity_m_s_)
                / config_.force_virtual_mass;
            const double desired_admittance_velocity_m_s = Clamp(
                force_axis_velocity_m_s_ + acceleration * dt,
                -config_.max_force_axis_speed_m_s,
                config_.max_force_axis_speed_m_s);
            force_axis_velocity_m_s_ = MoveToward(
                force_axis_velocity_m_s_, desired_admittance_velocity_m_s,
                config_.max_force_axis_acceleration_m_s2 * dt);
        }
        delta_z = force_axis_velocity_m_s_ * dt;
    } else {
        target_force_unloading_ = false;
        target_force_release_pending_ = false;
        target_force_recovering_ = false;
        target_force_contact_reacquired_ = false;
        target_force_recovery_stable_time_s_ = 0.0;
        force_axis_velocity_m_s_ = 0.0;
        delta_z = config_.approach_direction_tool_z
            * config_.approach_speed_m_s * dt;
    }

    const bool scan_phase = intent.phase_index == 0 || intent.phase_index == 2;
    const bool scan_alignment_ready =
        std::abs(intent.model_y_m) <= config_.scan_alignment_tolerance_m;
    if (target_force_transition_active) {
        // 超力卸载、制动和重新接触期间不允许横向视觉修正。
        visual_y_enabled_ = false;
        visual_y_force_stable_time_s_ = 0.0;
    } else if (in_contact) {
        // 复现旧六轴的即时 Tool-Y 门：只有 |Fz|>2 N 才允许视觉Y。
        // 不增加连续稳定计时；力降到门限及以下时立即关闭Y。
        visual_y_enabled_ =
            std::abs(force.z()) > config_.visual_y_enable_force_n;
        visual_y_force_stable_time_s_ = 0.0;
    }
    const bool force_in_settle_band = in_contact
        && !target_force_transition_active
        && std::abs(force.z()) >= config_.scan_start_force_n
        && std::abs(force.z() - requested_force)
               <= config_.scan_force_tolerance_n;
    if (in_contact && !force_settled_) {
        scan_force_stable_time_s_ = force_in_settle_band
            ? scan_force_stable_time_s_ + dt : 0.0;
        if (scan_force_stable_time_s_ + 1e-12
            >= config_.scan_force_stable_duration_s) {
            force_settled_ = true;
        }
    }

    Rm75SupervisorState control_state = Rm75SupervisorState::kApproach;
    if (in_contact) {
        control_state = Rm75SupervisorState::kForceSettle;
        if (force_settled_ && !target_force_transition_active) {
            if (rotation_latched_) {
                control_state = Rm75SupervisorState::kRotateAlign;
            } else if (intent.phase_index == 1) {
                trigger_alignment_stable_time_s_ =
                    std::abs(intent.model_y_m)
                            < config_.trigger_alignment_tolerance_m
                    ? trigger_alignment_stable_time_s_ + dt : 0.0;
                if (trigger_alignment_stable_time_s_ + 1e-12
                    >= config_.trigger_alignment_stable_duration_s) {
                    // Capture Tool-Y before applying the first RZ increment.
                    // Recovery must search along this fixed Base-frame axis,
                    // rather than along Tool-Y after it has rotated with RZ.
                    pre_rotation_tool_y_base_ =
                        RotationFromEuler(input.current_pose.tail<3>())
                        * config_.rotation_pose_from_tool.col(1);
                    const double axis_norm = pre_rotation_tool_y_base_.norm();
                    pre_rotation_tool_y_valid_ =
                        std::isfinite(axis_norm) && axis_norm > 1e-12;
                    if (pre_rotation_tool_y_valid_) {
                        pre_rotation_tool_y_base_ /= axis_norm;
                    }
                    rotation_latched_ = true;
                    accumulated_model_rz_rotation_rad_ = 0.0;
                    control_state = Rm75SupervisorState::kRotateAlign;
                } else {
                    control_state = Rm75SupervisorState::kTriggerAlign;
                }
            } else {
                trigger_alignment_stable_time_s_ = 0.0;
                if (scan_phase && scan_alignment_ready) {
                    control_state = Rm75SupervisorState::kScan;
                }
            }
        }
    } else {
        visual_y_force_stable_time_s_ = 0.0;
        scan_force_stable_time_s_ = 0.0;
        trigger_alignment_stable_time_s_ = 0.0;
        force_settled_ = false;
        visual_y_enabled_ = false;
        scan_latched_ = false;
        rotation_latched_ = false;
        pre_rotation_tool_y_valid_ = false;
        active_scan_phase_ = -1;
        accumulated_model_rz_rotation_rad_ = 0.0;
    }

    scan_latched_ = control_state == Rm75SupervisorState::kScan;
    active_scan_phase_ = scan_latched_ ? intent.phase_index : -1;
    const bool recovery_search_requested =
        control_state == Rm75SupervisorState::kRotateAlign
        && intent.phase_index == 2 && intent.recovery_mode;
    if (!recovery_search_requested
        && (intent.mask_lr_majority == 1 || intent.mask_lr_majority == 2)) {
        // Continuously remember the last valid pre-loss side. The visual
        // history may contain many zero votes by the time recovery begins.
        last_valid_mask_side_ = intent.mask_lr_majority;
    }
    if (!recovery_search_requested) {
        recovery_search_active_ = false;
        recovery_locked_mask_side_ = 0;
        recovery_search_distance_m_ = 0.0;
    } else if (!recovery_search_active_) {
        recovery_search_active_ = true;
        recovery_search_distance_m_ = 0.0;
        recovery_locked_mask_side_ = last_valid_mask_side_;
        if (recovery_locked_mask_side_ == 0
            && (intent.mask_lr_majority == 1
                || intent.mask_lr_majority == 2)) {
            recovery_locked_mask_side_ = intent.mask_lr_majority;
        }
    }
    output.recovery_search_active = recovery_search_active_;
    output.recovery_locked_mask_side = recovery_locked_mask_side_;
    if (recovery_search_requested) {
        // A lost-target model angle is not a trustworthy correction. Hold
        // Tool-X/RZ and contact-driven Roll/Pitch until vision confirms the
        // vessel again. The sign uses the mask side latched at recovery entry;
        // the physical axis is Tool-Y captured before RZ rotation started.
        remaining_model_rz_rad_ = 0.0;
    }
    double delta_x = 0.0;
    if (scan_latched_) {
        double scan_step = config_.scan_speed_m_s * dt;
        if (config_.maximum_scan_distance_m > 0.0) {
            const double remaining = std::max(
                0.0, config_.maximum_scan_distance_m - scan_distance_m_);
            constexpr double kScanCompletionToleranceM = 1e-12;
            scan_step = remaining > kScanCompletionToleranceM
                ? std::min(scan_step, remaining) : 0.0;
        }
        delta_x = config_.scan_direction_tool_x * scan_step;
    }

    // Treat the visual Y value as the current alignment error, not as a
    // displacement to accumulate once per Redis sequence. The proportional
    // controller produces a velocity and integrates it over this cycle.
    // 按 m 后 phase=0/2；在扫描阶段降低 Tool-Y 速度，保证 Tool-X 是主运动。
    // Trigger 锁存旋转模式时，继续沿用独立的旧六轴 0.3 缩放。
    const double model_y_scale = rotation_latched_
        ? config_.rotate_align_y_scale
        : (scan_phase ? config_.scan_y_scale : 1.0);
    const double model_y_velocity_m_s = config_.model_y_direction
        * config_.model_y_velocity_gain_per_s
        * model_y_scale * intent.model_y_m;
    double commanded_y_velocity_m_s = model_y_velocity_m_s;
    if (recovery_search_requested) {
        if (recovery_locked_mask_side_ == 1) {
            commanded_y_velocity_m_s = -config_.recovery_tool_y_speed_m_s;
        } else if (recovery_locked_mask_side_ == 2) {
            commanded_y_velocity_m_s = config_.recovery_tool_y_speed_m_s;
        } else {
            commanded_y_velocity_m_s = 0.0;
        }
    }
    if (input.robot_model_tool_y_error_m
            >= config_.visual_y_tracking_pause_error_m) {
        visual_y_tracking_paused_ = true;
    } else if (input.robot_model_tool_y_error_m
                   <= config_.visual_y_tracking_resume_error_m) {
        visual_y_tracking_paused_ = false;
    }
    double model_y_step = 0.0;
    if (visual_y_enabled_
        && !target_force_transition_active
        && !visual_y_tracking_paused_) {
        double requested_model_y_step = Clamp(
            commanded_y_velocity_m_s * dt,
            -config_.maximum_model_y_step_m,
            config_.maximum_model_y_step_m);
        if (recovery_search_requested) {
            const double remaining_recovery_distance_m = std::max(
                0.0,
                config_.maximum_recovery_tool_y_distance_m
                    - recovery_search_distance_m_);
            if (std::abs(requested_model_y_step)
                > remaining_recovery_distance_m) {
                requested_model_y_step = std::copysign(
                    remaining_recovery_distance_m,
                    requested_model_y_step);
            }
        }
        // 预测式 Tool-Y 跟踪门控：不能只在上一周期的误差已经越界后
        // 才暂停。若本周期候选步长会让实际 TCP 与 ServoJ 模型目标的
        // Tool-Y 误差超过暂停门，只走完剩余裕量并立即锁存暂停。这样
        // 持久参考轨迹不会因一个较大的视觉误差在单周期内跨过门限。
        const double remaining_tracking_margin_m = std::max(
            0.0,
            config_.visual_y_tracking_pause_error_m
                - input.robot_model_tool_y_error_m);
        if (std::abs(requested_model_y_step) > remaining_tracking_margin_m) {
            model_y_step = std::copysign(
                remaining_tracking_margin_m, requested_model_y_step);
            visual_y_tracking_paused_ = true;
        } else {
            model_y_step = requested_model_y_step;
        }
    }
    if (recovery_search_requested) {
        recovery_search_distance_m_ += std::abs(model_y_step);
    }
    output.recovery_search_distance_m = recovery_search_distance_m_;
    output.recovery_distance_limit_reached = recovery_search_requested
        && recovery_search_distance_m_ + 1e-12
               >= config_.maximum_recovery_tool_y_distance_m;
    output.recovery_tool_y_velocity_m_s =
        recovery_search_requested
            && !output.recovery_distance_limit_reached
        ? commanded_y_velocity_m_s : 0.0;
    output.visual_y_tracking_paused = visual_y_tracking_paused_;
    Eigen::Vector3d translation_delta;
    translation_delta << delta_x,
        model_y_step,
        delta_z;
    if (in_contact) {
        // Keep the admittance damping state consistent with the applied
        // Tool-Z step.
        force_axis_velocity_m_s_ = translation_delta.z() / dt;
    }
    if (scan_latched_) {
        scan_distance_m_ += std::abs(translation_delta.x());
    }

    double delta_roll = 0.0;
    if (config_.legacy_contact_roll_enabled
        && !target_force_transition_active
        && !recovery_search_requested) {
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
    if (!target_force_transition_active
        && !recovery_search_requested
        && input.contact_valid
        && std::isfinite(output.filtered_contact_point_probe_m.y())) {
        // Roll 与 Pitch 必须使用同一份 Kalman 后接触点，避免原始接触点
        // 的瞬时噪声绕过滤波器而直接形成 Pitch 姿态指令。
        delta_pitch = Clamp(
            -config_.contact_pitch_gain_rad_per_m
                * output.filtered_contact_point_probe_m.y(),
            -max_angular_step,
            max_angular_step);
    }
    double delta_rz = 0.0;
    if (control_state == Rm75SupervisorState::kRotateAlign
               && !recovery_search_requested) {
        delta_rz = Clamp(
            remaining_model_rz_rad_, -max_model_rz_step, max_model_rz_step);
    }
    if (control_state == Rm75SupervisorState::kRotateAlign) {
        const double remaining_rotation_budget_rad = std::max(
            0.0,
            config_.maximum_total_model_rz_rotation_rad
                - accumulated_model_rz_rotation_rad_);
        if (std::abs(delta_rz) > remaining_rotation_budget_rad) {
            delta_rz = std::copysign(remaining_rotation_budget_rad, delta_rz);
        }
        accumulated_model_rz_rotation_rad_ += std::abs(delta_rz);
    }
    // 公共包络只约束接触点 Roll/Pitch；视觉 RZ 使用独立限速，避免较慢的
    // 接触姿态修正把 rotate-align 的绕 Tool-Z 对齐一并拖慢。
    const double combined_angular_step = std::sqrt(
        (config_.legacy_contact_roll_limits_enabled
             ? delta_roll * delta_roll : 0.0)
        + delta_pitch * delta_pitch);
    if (combined_angular_step > max_angular_step) {
        const double scale = max_angular_step / combined_angular_step;
        if (config_.legacy_contact_roll_limits_enabled) delta_roll *= scale;
        delta_pitch *= scale;
    }
    if (!recovery_search_requested) {
        remaining_model_rz_rad_ -= delta_rz;
    }
    output.accumulated_model_rz_rotation_rad =
        accumulated_model_rz_rotation_rad_;

    output.requested_delta << translation_delta.x(),
        translation_delta.y(), translation_delta.z(),
        delta_roll, delta_pitch, delta_rz;

    // The controller pose is Base -> Arm_Tip, while all control deltas are
    // expressed in the physical Tool/Sensor frame. Keep the fixed mount
    // rotation explicit so -Tool-X does not accidentally mean -ArmTip-X.
    const Eigen::Matrix3d rotation_base_from_pose =
        RotationFromEuler(input.current_pose.tail<3>());
    const Eigen::Matrix3d rotation_base_from_tool =
        rotation_base_from_pose * config_.rotation_pose_from_tool;
    const Eigen::Matrix3d incremental_rotation_tool =
        (Eigen::AngleAxisd(delta_rz, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(delta_pitch, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(delta_roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const Eigen::Matrix3d desired_rotation_base_from_tool =
        rotation_base_from_tool * incremental_rotation_tool;
    const Eigen::Matrix3d desired_rotation_base_from_pose =
        desired_rotation_base_from_tool
        * config_.rotation_pose_from_tool.transpose();
    const Eigen::Vector3d current_probe_tcp_base =
        input.current_pose.head<3>()
        + rotation_base_from_tool * config_.probe_tcp_tool_m;
    Eigen::Vector3d translation_delta_base =
        rotation_base_from_tool * translation_delta;
    if (recovery_search_requested && pre_rotation_tool_y_valid_) {
        // X is zero during recovery, while Z force control remains expressed
        // in the current Tool frame. Replace only the Y contribution with the
        // Tool-Y direction captured before rotate-align began.
        translation_delta_base =
            rotation_base_from_tool
                * Eigen::Vector3d(translation_delta.x(), 0.0,
                                  translation_delta.z())
            + pre_rotation_tool_y_base_ * translation_delta.y();
    }
    const Eigen::Vector3d desired_probe_tcp_base =
        current_probe_tcp_base + translation_delta_base;
    output.desired_pose.head<3>() = desired_probe_tcp_base
        - desired_rotation_base_from_tool * config_.probe_tcp_tool_m;
    output.desired_pose.tail<3>() =
        EulerFromRotation(desired_rotation_base_from_pose);
    output.command_motion = output.requested_delta.cwiseAbs().maxCoeff() > 0.0;
    output.target_force_unloading = target_force_unloading_;
    output.target_force_recovering = target_force_recovering_
        || (target_force_transition_active && !target_force_unloading_);
    output.state = control_state;
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
        if (result.target_joints[joint]
                < kinematics_.JointMinimums()[joint]
            || result.target_joints[joint]
                > kinematics_.JointMaximums()[joint]) {
            result.error = Rm75PlanError::kJointLimit;
            result.detail = "planned target exceeds J" + std::to_string(joint + 1)
                + " joint limit";
            return result;
        }
        const double margin = std::min(
            result.target_joints[joint]
                - kinematics_.JointMinimums()[joint],
            kinematics_.JointMaximums()[joint]
                - result.target_joints[joint])
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
