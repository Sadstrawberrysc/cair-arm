#include <rm75_runtime_config.hpp>

#include <cmath>
#include <filesystem>

namespace {

void SetError(std::string* error, const char* message) {
    if (error != nullptr) *error = message;
}

bool DirectionValid(double value, bool allow_zero) {
    return (allow_zero && value == 0.0) || value == -1.0 || value == 1.0;
}

}  // namespace

RobotRuntimeConfig MakeImplicitRm75ProductionConfig(
    const std::string& executable_directory,
    const std::string& timestamp_for_filename) {
    RobotRuntimeConfig config;
    const std::filesystem::path directory(executable_directory);
    config.mode = ControllerMode::kExecute;
    config.redis_enabled = true;
    config.robot_ip = "192.168.50.254";
    config.robot_port = 8080;
    config.sensor_device =
        "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0DU5LC-if00-port0";
    config.sensor_baud = 115200;
    config.calibration_path =
        (directory / "rm75_force_calibration.json").string();
    config.expected_sensor_id = "DU0DU5LC";
    config.probe_model_path =
        (directory / "../model/Lprobe-IFS.STL").lexically_normal().string();
    config.duration_s = 0;
    config.tare_no_contact_s = 3;
    config.manual_phase = -1;
    config.manual_action = false;
    config.allow_provisional_force_control = true;
    config.implicit_commissioning_profile = true;
    config.safety.maximum_no_contact_approach_distance_m = 0.250;
    config.safety.maximum_orientation_excursion_deg = 0.0;
    config.runtime_log_path =
        (directory / "logs"
         / (std::string(kImplicitRm75ProfileName) + "_"
            + timestamp_for_filename + ".csv"))
            .string();
    return config;
}

bool ValidateRobotRuntimeConfig(const RobotRuntimeConfig& config,
                                std::string* error) {
    const auto& control = config.control;
    const auto& planner = config.planner;
    const auto& safety = config.safety;
    const double maximum_approach_m =
        config.implicit_commissioning_profile ? 0.250 : 0.005;
    if (config.calibration_path.empty()
        || config.robot_port <= 0 || config.robot_port > 65535
        || config.redis_port <= 0 || config.redis_port > 65535
        || config.period_ms < 10 || config.period_ms > 100
        || config.sensor_stale_ms < config.period_ms
        || config.sensor_stale_ms > 1000
        || config.robot_stale_ms < config.period_ms
        || config.robot_stale_ms > 2000
        || config.command_stale_ms < config.period_ms
        || config.command_stale_ms > 10000
        || config.duration_s < 0 || config.publish_every <= 0
        || config.execute_warmup_s < 2 || config.execute_warmup_s > 10
        || config.tare_no_contact_s < 0 || config.tare_no_contact_s > 10
        || config.manual_y_m < -0.2 || config.manual_y_m > 0.2
        || config.manual_rz_deg < -180.0 || config.manual_rz_deg > 180.0
        || config.manual_phase < -1 || config.manual_phase > 2) {
        SetError(error, "invalid entry runtime configuration");
        return false;
    }
    if (control.desired_force_n < -3.0 || control.desired_force_n > -0.1
        || control.approach_speed_m_s < 0.0
        || control.approach_speed_m_s > 0.020
        || !DirectionValid(control.approach_direction_tool_z, true)
        || control.max_force_axis_speed_m_s <= 0.0
        || control.max_force_axis_speed_m_s > 0.005
        || control.scan_start_force_n < control.contact_threshold_n
        || control.scan_start_force_n > control.force_limit_z_n
        || control.scan_force_tolerance_n <= 0.0
        || control.scan_force_tolerance_n > 3.0
        || control.scan_force_stable_duration_s < 0.02
        || control.scan_force_stable_duration_s > 10.0
        || control.scan_speed_m_s <= 0.0 || control.scan_speed_m_s > 0.010
        || control.maximum_scan_distance_m < 0.0
        || !DirectionValid(control.scan_direction_tool_x, false)) {
        SetError(error, "invalid control configuration");
        return false;
    }
    if (safety.raw_force_limit_n <= 0.0 || safety.raw_force_limit_n > 50.0
        || safety.raw_torque_limit_nm <= 0.0
        || safety.raw_torque_limit_nm > 5.0
        || safety.max_tracking_joint_error_deg <= 0.0
        || safety.max_tracking_joint_error_deg > 20.0
        || safety.max_tracking_position_error_mm <= 0.0
        || safety.max_tracking_position_error_mm > 25.0
        || safety.max_tracking_orientation_error_deg < 0.0
        || safety.max_tracking_orientation_error_deg > 10.0
        || safety.maximum_no_contact_approach_distance_m <= 0.0
        || safety.maximum_no_contact_approach_distance_m > maximum_approach_m
        || safety.maximum_orientation_excursion_deg < 0.0
        || safety.maximum_orientation_excursion_deg > 15.0) {
        SetError(error, "invalid runtime safety configuration");
        return false;
    }
    if (planner.period_ms <= 0 || planner.minimum_dispatch_gap_ms <= 0.0
        || planner.joint_speed_scale <= 0.0
        || planner.joint_speed_scale > 1.0
        || planner.max_joint_accel_deg_s2 < 0.0) {
        SetError(error, "invalid planner configuration");
        return false;
    }
    if (config.mode == ControllerMode::kExecute
        && (config.simulate || config.period_ms != 10
            || planner.allow_near_singularity)) {
        SetError(error, "invalid execute-mode configuration");
        return false;
    }
    if (error != nullptr) error->clear();
    return true;
}
