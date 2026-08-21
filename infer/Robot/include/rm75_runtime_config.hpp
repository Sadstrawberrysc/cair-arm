#pragma once

#include <string>

#include <rm75_control.hpp>

enum class ControllerMode {
    kObserve,
    kDryRun,
    kExecute,
};

inline const char* ModeName(ControllerMode mode) noexcept {
    switch (mode) {
        case ControllerMode::kObserve: return "observe";
        case ControllerMode::kDryRun: return "dry_run";
        case ControllerMode::kExecute: return "execute";
    }
    return "unknown";
}

struct RobotRuntimeConfig {
    ControllerMode mode = ControllerMode::kObserve;
    bool simulate = false;
    bool redis_enabled = true;
    std::string robot_ip = "192.168.50.254";
    int robot_port = 8080;
    std::string sensor_device = "/dev/ttyUSB0";
    unsigned int sensor_baud = 115200;
    int sensor_stale_ms = 50;
    int robot_stale_ms = 200;
    int command_stale_ms = 500;
    std::string calibration_path;
    std::string expected_sensor_id;
    std::string probe_model_path = "../model/Lprobe-IFS.STL";
    std::string redis_host = "127.0.0.1";
    int redis_port = 7777;
    int period_ms = 10;
    int duration_s = 0;
    int publish_every = 5;
    int execute_warmup_s = 2;
    int tare_no_contact_s = 0;
    std::string runtime_log_path;

    double manual_y_m = 0.0;
    double manual_rz_deg = 0.0;
    int manual_phase = -1;
    bool manual_action = false;
    bool manual_terminate = false;
    bool allow_provisional_force_control = false;
    bool implicit_commissioning_profile = false;

    Rm75ControlConfig control;
    Rm75ServoPlannerConfig planner;
    Rm75RuntimeSafetyConfig safety;
};

inline constexpr const char* kImplicitRm75ProfileName =
    "rm75_redis_visual_closed_loop_tcp188";

RobotRuntimeConfig MakeImplicitRm75ProductionConfig(
    const std::string& executable_directory,
    const std::string& timestamp_for_filename);

bool ValidateRobotRuntimeConfig(const RobotRuntimeConfig& config,
                                std::string* error = nullptr);
