#include <force_calibration_capture.hpp>
#include <force_calibration_path.hpp>
#ifdef CALIBRATION_HAS_SELF_COLLISION
#include <force_calibration_self_collision.hpp>
#endif
#include <realman_kinematics.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <string>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRawForceLimitN = 50.0;
constexpr double kRawTorqueLimitNm = 5.0;
std::atomic<bool> g_stop_requested{false};

void HandleSignal(int) {
    g_stop_requested.store(true);
}

struct Options {
    std::string robot_ip;
    int robot_port = 8080;
    std::string sensor_device;
    unsigned int sensor_baud = 460800;
    ForceSensorProtocol sensor_protocol = ForceSensorProtocol::kKwr57Rs422;
    std::string output_path;
    Eigen::Vector3d sensor_to_tool_rpy_deg = Eigen::Vector3d::Zero();
    bool sensor_to_tool_rpy_set = false;
    Eigen::Vector3d probe_to_sensor_rpy_deg = Eigen::Vector3d::Zero();
    int pose_count = 8;
    int samples_per_pose = 20;
    int capture_timeout_s = 30;
    int sensor_stale_ms = 50;
    int robot_stale_ms = 100;
    int maximum_pair_skew_ms = 50;
    int minimum_window_duration_ms = 500;
    bool overwrite = false;
    bool automatic = false;
    bool plan_only = false;
    bool pose_count_set = false;
    bool start_joints_set = false;
    Eigen::Matrix<double, 7, 1> start_joints_rad = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> joint_half_ranges_deg =
        (Eigen::Matrix<double, 7, 1>() << 20, 20, 20, 20, 45, 45, 60).finished();
    std::string path_output;
    int velocity = 5;
    int move_timeout_s = 30;
    std::string resume_csv;
    std::string path_mode = "workspace";
    Eigen::Vector3d tcp_armtip_m{0.0, 0.0, 0.2005};
    bool resume_workspace = false;
    bool check_resume = false;
    std::string workspace_config;
    ForceCalibrationPathOptions height_constraints;
    std::function<bool(const Eigen::Matrix<double,7,1>&,const Eigen::Matrix<double,7,1>&)> self_collision_check;
    nlohmann::json self_collision_config;
};

void Usage(const char* program) {
    std::cout
        << "Usage: " << program << " --robot-ip IP --sensor-device PATH\n"
        << "       --sensor-to-tool-rpy-deg RX,RY,RZ --output FILE [options]\n\n"
        << "Interactively capture synchronized RM75 pose + raw wrench rows for\n"
        << "force_sensor_calibrate. This program is read-only and never sends\n"
        << "MoveJ, ServoJ, Hold or Stop commands in manual mode.\n\n"
        << "Automatic mode sends MoveJ and controlled Stop commands.\n"
        << "KWR57 uses stream start/stop commands.\n\n"
        << "Required:\n"
        << "  --robot-ip IP\n"
        << "  --sensor-device PATH\n"
        << "  --sensor-to-tool-rpy-deg RX,RY,RZ\n"
        << "      Independently measured R_ArmTip_Sensor = Rz*Ry*Rx\n"
        << "  --probe-to-sensor-rpy-deg RX,RY,RZ  probe axes in Sensor, default 0,0,0\n"
        << "  --output FILE\n\n"
        << "Options:\n"
        << "  --robot-port PORT          default 8080\n"
        << "  --sensor-protocol NAME     kwr57-rs422 (default), haptron-modbus, legacy-aa55\n"
        << "  --sensor-baud BAUD         default 460800 for KWR57, 115200 otherwise\n"
        << "  --automatic                MoveJ through selected poses; no manual teaching\n"
        << "  --path-mode workspace|local  default workspace; local uses joint half-ranges\n"
        << "  --workspace-config FILE    Base-frame allowed regions and forbidden volumes\n"
        << "  --probe-tcp-armtip-m X,Y,Z  TCP in ArmTip frame; workspace FK offset\n"
        << "  --plan-only                export automatic path without moving or opening the sensor\n"
        << "  --start-joints-deg Q1,..,Q7 offline plan center (only with --plan-only)\n"
        << "  --path-output FILE         default OUTPUT.path.json\n"
        << "  --wrist-half-ranges-deg A,B,C  J5/J6/J7 half ranges; default 45,45,60\n"
        << "  --joint-half-ranges-deg A,..,G all seven half ranges; default 20,20,20,20,45,45,60\n"
        << "  --velocity V               MoveJ speed percentage, default 5\n"
        << "  --resume FILE              continue partial CSV using its original path JSON\n"
        << "  --check-resume             validate resume files offline; no device/network access\n"
        << "  --resume-workspace         preserve accepted prefix; replan remaining spatial targets\n"
        << "  --move-timeout-sec SEC     default 30\n"
        << "  --poses N                  automatic default 1000; local requires a cubic count\n"
        << "  --samples-per-pose N       default 20, minimum 5\n"
        << "  --capture-timeout-sec SEC  default 30\n"
        << "  --sensor-stale-ms MS       default 50\n"
        << "  --robot-stale-ms MS        default 100\n"
        << "  --max-pair-skew-ms MS      default 50 (host receive time)\n"
        << "  --min-window-duration-ms MS default 500\n"
        << "  --overwrite                atomically replace an existing FILE\n"
        << "  --help\n\n"
        << "The probe must be completely free of contact at every pose. Absolute\n"
        << "raw force cannot prove no-contact before gravity calibration.\n";
}

bool ParseInteger(const std::string& text, long long& value) {
    if (text.empty()) return false;
    errno = 0;
    char* end = nullptr;
    const long long parsed = std::strtoll(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0') return false;
    value = parsed;
    return true;
}

bool ParseDouble(const std::string& text, double& value) {
    if (text.empty()) return false;
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0'
        || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

bool ParseVector3(const std::string& text, Eigen::Vector3d& value) {
    std::stringstream input(text);
    std::string item;
    int index = 0;
    while (std::getline(input, item, ',')) {
        if (index >= 3 || !ParseDouble(item, value[index])) return false;
        ++index;
    }
    return index == 3;
}

bool ParseJoints(const std::string& text, Eigen::Matrix<double, 7, 1>& joints, bool radians = true) {
    std::stringstream stream(text);
    std::string item;
    int axis = 0;
    while (std::getline(stream, item, ',')) {
        if (axis >= 7 || !ParseDouble(item, joints[axis])) return false;
        ++axis;
    }
    if (radians) joints *= kPi / 180.0;
    return axis == 7;
}

bool ParseOptions(int argc, char** argv, Options& options) {
    bool sensor_baud_explicit = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            Usage(argv[0]);
            std::exit(0);
        }
        auto next = [&]() -> const char* {
            return index + 1 < argc ? argv[++index] : nullptr;
        };

        if (argument == "--overwrite") {
            options.overwrite = true;
            continue;
        }
        if (argument == "--check-resume") {
            options.check_resume = true;
            options.automatic = true;
            continue;
        }
        if (argument == "--automatic" || argument == "--plan-only") {
            options.automatic = true;
            options.plan_only = argument == "--plan-only" || options.plan_only;
            continue;
        }
        if (argument == "--resume-workspace") {
            options.resume_workspace = true;
            options.automatic = true;
            continue;
        }
        const char* raw_value = next();
        if (raw_value == nullptr) {
            std::cerr << "Missing value for " << argument << '\n';
            return false;
        }
        const std::string value(raw_value);
        long long integer = 0;
        if (argument == "--robot-ip") {
            options.robot_ip = value;
        } else if (argument == "--sensor-device") {
            options.sensor_device = value;
        } else if (argument == "--output") {
            options.output_path = value;
        } else if (argument == "--path-output") {
            options.path_output = value;
        } else if (argument == "--path-mode") {
            if (value != "workspace" && value != "local") return false;
            options.path_mode = value;
        } else if (argument == "--workspace-config") {
            options.workspace_config = value;
        } else if (argument == "--probe-tcp-armtip-m") {
            if (!ParseVector3(value, options.tcp_armtip_m)) return false;
        } else if (argument == "--resume") {
            options.resume_csv = value;
            options.automatic = true;
        } else if (argument == "--start-joints-deg") {
            if (!ParseJoints(value, options.start_joints_rad)) return false;
            options.start_joints_set = true;
        } else if (argument == "--wrist-half-ranges-deg") {
            Eigen::Vector3d ranges;
            if (!ParseVector3(value, ranges) || (ranges.array() <= 0).any()) return false;
            options.joint_half_ranges_deg.tail<3>() = ranges;
        } else if (argument == "--joint-half-ranges-deg") {
            if (!ParseJoints(value, options.joint_half_ranges_deg, false)
                || (options.joint_half_ranges_deg.head<4>().array() < 0).any()
                || (options.joint_half_ranges_deg.tail<3>().array() <= 0).any()) return false;
        } else if (argument == "--sensor-protocol") {
            if (!ParseForceSensorProtocol(value, options.sensor_protocol)) return false;
        } else if (argument == "--probe-to-sensor-rpy-deg") {
            if (!ParseVector3(value, options.probe_to_sensor_rpy_deg)) return false;
        } else if (argument == "--sensor-to-tool-rpy-deg") {
            if (!ParseVector3(value, options.sensor_to_tool_rpy_deg)) return false;
            options.sensor_to_tool_rpy_set = true;
        } else if (!ParseInteger(value, integer)) {
            std::cerr << "Invalid integer for " << argument << ": " << value << '\n';
            return false;
        } else if (argument == "--robot-port") {
            if (integer <= 0 || integer > 65535) return false;
            options.robot_port = static_cast<int>(integer);
        } else if (argument == "--sensor-baud") {
            if (integer <= 0
                || static_cast<unsigned long long>(integer)
                       > std::numeric_limits<unsigned int>::max()) {
                return false;
            }
            options.sensor_baud = static_cast<unsigned int>(integer);
            sensor_baud_explicit = true;
        } else if (argument == "--poses") {
            if (integer < 6 || integer > 8000) return false;
            options.pose_count = static_cast<int>(integer);
            options.pose_count_set = true;
        } else if (argument == "--velocity") {
            if (integer < 1 || integer > 100) return false;
            options.velocity = static_cast<int>(integer);
        } else if (argument == "--move-timeout-sec") {
            if (integer < 1 || integer > 300) return false;
            options.move_timeout_s = static_cast<int>(integer);
        } else if (argument == "--samples-per-pose") {
            if (integer < 5 || integer > 1000) return false;
            options.samples_per_pose = static_cast<int>(integer);
        } else if (argument == "--capture-timeout-sec") {
            if (integer < 1 || integer > 120) return false;
            options.capture_timeout_s = static_cast<int>(integer);
        } else if (argument == "--sensor-stale-ms") {
            if (integer < 1 || integer > 1000) return false;
            options.sensor_stale_ms = static_cast<int>(integer);
        } else if (argument == "--robot-stale-ms") {
            if (integer < 1 || integer > 2000) return false;
            options.robot_stale_ms = static_cast<int>(integer);
        } else if (argument == "--max-pair-skew-ms") {
            if (integer < 0 || integer > 1000) return false;
            options.maximum_pair_skew_ms = static_cast<int>(integer);
        } else if (argument == "--min-window-duration-ms") {
            if (integer < 1 || integer > 60000) return false;
            options.minimum_window_duration_ms = static_cast<int>(integer);
        } else {
            std::cerr << "Unknown option: " << argument << '\n';
            return false;
        }
    }

    if (!sensor_baud_explicit) options.sensor_baud = DefaultForceSensorBaud(options.sensor_protocol);
    if (options.automatic && !options.pose_count_set) options.pose_count = 1000;
    if (options.start_joints_set && !options.plan_only) return false;
    if (!options.resume_csv.empty() && options.plan_only && !options.resume_workspace) return false;
    if (options.resume_workspace && options.resume_csv.empty()) return false;
    if (options.check_resume && (options.resume_csv.empty() || options.plan_only || options.resume_workspace)) return false;
    if ((!options.start_joints_set && options.resume_csv.empty() && options.robot_ip.empty())
        || options.robot_ip.size() > RMConnectionConfig::kMaximumIpv4TextLength
        || (!options.plan_only && !options.check_resume && options.sensor_device.empty()) || options.output_path.empty()
        || !options.sensor_to_tool_rpy_set) {
        return false;
    }
    if (options.path_output.empty()) options.path_output = options.output_path + ".path.json";
    if (options.path_output == options.output_path) return false;
    if (options.minimum_window_duration_ms
        >= options.capture_timeout_s * 1000) {
        std::cerr << "--min-window-duration-ms must be shorter than "
                     "--capture-timeout-sec\n";
        return false;
    }
    return true;
}

bool RawWrenchWithinLimits(const WrenchSample& sample, ForceSensorProtocol protocol,
                          std::string& diagnostic) {
    const char* names[] = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
    for (std::size_t axis = kForceX; axis <= kTorqueZ; ++axis) {
        const bool force = axis <= kForceZ;
        const double limit = force ? kRawForceLimitN
            : (protocol == ForceSensorProtocol::kKwr57Rs422 ? 2.0 : kRawTorqueLimitNm);
        if (!std::isfinite(sample.wrench_si[axis])
            || std::abs(sample.wrench_si[axis]) > limit) {
            std::ostringstream message;
            message << std::setprecision(9) << "raw wrench exceeded capture limit: axis="
                    << names[axis] << "; value=" << sample.wrench_si[axis]
                    << (force ? " N" : " N*m") << "; abs_limit=" << limit
                    << (force ? " N" : " N*m") << "; sensor_sequence=" << sample.sequence
                    << "; raw_si=[";
            for (std::size_t i = kForceX; i <= kTorqueZ; ++i)
                message << (i ? "," : "") << sample.wrench_si[i];
            message << "] (Fx,Fy,Fz in N; Tx,Ty,Tz in N*m)";
            diagnostic = message.str();
            return false;
        }
    }
    return true;
}

void WriteCsvHeader(std::ostream& output,
                    const Options& options,
                    const Eigen::Matrix3d& rotation_tool_from_sensor) {
    output << "# generated_by=force_sensor_calibration_capture\n"
           << (options.automatic ? "# automatic=true; robot_motion=MoveJ\n"
                                 : "# read_only=true; no_robot_motion_commands=true\n")
           << "# motion_allowed=true; settling_ms=0\n"
           << "# pairing=bounded_host_monotonic_receive_time; not_hardware_sync\n"
           << "# robot=" << options.robot_ip << ':' << options.robot_port << '\n'
           << "# sensor_device=" << options.sensor_device
           << "; sensor_baud=" << options.sensor_baud
           << "; sensor_protocol="
           << ForceSensorProtocolName(options.sensor_protocol)
           << '\n'
           << "# sensor_to_tool_rpy_deg="
           << FormatForceCalibrationCaptureRpyDeg(
                  options.sensor_to_tool_rpy_deg)
           << '\n'
           << "# probe_to_sensor_rpy_deg="
           << FormatForceCalibrationCaptureRpyDeg(options.probe_to_sensor_rpy_deg) << '\n'
           << "# minimum_window_duration_ms="
           << options.minimum_window_duration_ms << '\n'
           << "# rotation_tool_from_sensor_row_major=";
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            if (row != 0 || column != 0) output << ',';
            output << std::setprecision(15)
                   << rotation_tool_from_sensor(row, column);
        }
    }
    output << "\nr00,r01,r02,r10,r11,r12,r20,r21,r22,"
              "fx,fy,fz,tx,ty,tz\n";
}

void WriteCsvSample(std::ostream& output,
                    int pose_index,
                    const ForceCalibrationCaptureResult& capture) {
    const auto& diagnostics = capture.diagnostics;
    output << "# pose=" << pose_index
           << "; sensor_sequence=" << diagnostics.first_sensor_sequence
           << ".." << diagnostics.last_sensor_sequence
           << "; max_pair_skew_ms=" << std::fixed << std::setprecision(3)
           << diagnostics.maximum_pair_time_delta_ms
           << "; robot_window_duration_ms="
           << diagnostics.robot_window_duration_ms
           << "; sensor_window_duration_ms="
           << diagnostics.sensor_window_duration_ms
           << "; joint_span_deg="
           << diagnostics.maximum_joint_span_rad * 180.0 / kPi
           << "; position_span_mm="
           << diagnostics.maximum_position_span_m * 1000.0
           << "; orientation_span_deg="
           << diagnostics.maximum_orientation_span_rad * 180.0 / kPi << '\n';
    output << std::setprecision(15) << std::defaultfloat;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            output << capture.sample.rotation_base_from_sensor(row, column) << ',';
        }
    }
    for (int axis = 0; axis < 6; ++axis) {
        output << capture.sample.raw_wrench_sensor[axis]
               << (axis == 5 ? '\n' : ',');
    }
    output.flush();
}

bool WaitForInitialData(ForceSensorReader& sensor,
                        RMStateReader& robot,
                        std::chrono::seconds timeout,
                        std::string& error) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!g_stop_requested.load()
           && std::chrono::steady_clock::now() < deadline) {
        const WrenchSample wrench = sensor.LatestSample();
        const RobotStateSnapshot state = robot.Latest();
        if (wrench.IntegrityValid() && !wrench.stale
            && wrench.io_status == ForceSensorIoStatus::kStreaming
            && wrench.io_error == 0 && state.valid && !state.stale
            && state.arm_err == 0 && state.sys_err == 0) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    error = "no simultaneous healthy robot and force-sensor data before timeout";
    return false;
}

ForceCalibrationCaptureResult CapturePoseWindow(
    ForceSensorReader& sensor,
    RMStateReader& robot,
    const Eigen::Matrix3d& rotation_tool_from_sensor,
    const Options& options) {
    ForceCalibrationCaptureLimits limits;
    limits.minimum_samples = static_cast<std::size_t>(options.samples_per_pose);
    limits.maximum_pair_time_delta = std::chrono::milliseconds(options.maximum_pair_skew_ms);
    limits.minimum_window_duration = std::chrono::milliseconds(options.minimum_window_duration_ms);
    std::vector<ForceCalibrationCaptureObservation> observations;
    observations.reserve(static_cast<std::size_t>(options.samples_per_pose));
    std::uint64_t last_sensor_sequence = sensor.LatestSample().sequence;
    std::uint64_t last_robot_sequence = robot.Latest().sequence;
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(options.capture_timeout_s);
    ForceCalibrationCaptureResult result;
    auto fail = [&](const std::string& message) {
        result.valid = false;
        result.diagnostics.message = message;
        return result;
    };
    while (!g_stop_requested.load() && std::chrono::steady_clock::now() < deadline) {
        const WrenchSample wrench = sensor.LatestSample();
        const RobotStateSnapshot state = robot.Latest();
        if (wrench.sequence == 0 || wrench.sequence <= last_sensor_sequence
            || state.sequence == 0 || state.sequence <= last_robot_sequence) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        last_sensor_sequence = wrench.sequence;
        last_robot_sequence = state.sequence;
        const RMResult robot_result = robot.LastResult();
        if (!robot_result) return fail("robot state reader failed: " + robot_result.message);
        std::string limit_diagnostic;
        if (!RawWrenchWithinLimits(wrench, options.sensor_protocol, limit_diagnostic)) {
            result.error = ForceCalibrationCaptureError::kWrenchLimitExceeded;
            return fail(limit_diagnostic);
        }
        // Validate every observation before including it in the average.
        auto single_limits = limits;
        single_limits.minimum_samples = 1;
        single_limits.minimum_window_duration = std::chrono::milliseconds(0);
        auto healthy = CaptureForceCalibrationWindow({{state, wrench}},
                                                      rotation_tool_from_sensor, single_limits);
        if (!healthy.valid) return healthy;
        observations.push_back({state, wrench});
        if (observations.size() < limits.minimum_samples) continue;
        const auto robot_span = observations.back().robot.received_at - observations.front().robot.received_at;
        const auto sensor_span = observations.back().wrench.monotonic_timestamp
            - observations.front().wrench.monotonic_timestamp;
        if (robot_span < limits.minimum_window_duration || sensor_span < limits.minimum_window_duration) continue;
        result = CaptureForceCalibrationWindow(observations, rotation_tool_from_sensor, limits);
        return result;
    }
    if (g_stop_requested.load()) return fail("capture interrupted");
    return fail("no complete averaging window within " + std::to_string(options.capture_timeout_s)
                + " s; " + result.diagnostics.message);
}

bool PromptForCapture(int pose_index, int pose_count) {
    std::cout << "\nPose " << pose_index << '/' << pose_count
              << ": use the teach pendant to choose a DIFFERENT orientation,\n"
                 "stop all motion, keep the probe completely free of contact,\n"
                 "then press Enter to capture (q + Enter to stop): "
              << std::flush;
    while (!g_stop_requested.load()) {
        pollfd input{};
        input.fd = STDIN_FILENO;
        input.events = POLLIN;
        const int ready = ::poll(&input, 1, 100);
        if (ready < 0) {
            if (errno == EINTR) continue;
            std::cerr << "Cannot wait for operator input: "
                      << std::strerror(errno) << '\n';
            return false;
        }
        if (ready == 0) continue;
        if ((input.revents & (POLLIN | POLLHUP)) == 0) continue;
        std::string line;
        if (!std::getline(std::cin, line) || line == "q" || line == "Q") {
            return false;
        }
        return true;
    }
    return false;
}

bool PlanAutomaticPath(Options& options,
                       const Eigen::Matrix<double, 7, 1>& center,
                       const Eigen::Matrix3d& armtip_from_sensor,
                       ForceCalibrationPath& path,
                       const std::vector<Eigen::Matrix<double, 7, 1>>& preserved = {}) {
    RMKinematics kinematics;
    ForceCalibrationPathOptions settings;
    settings.pose_count = static_cast<std::size_t>(options.pose_count);
    settings.arm_half_ranges_deg = options.joint_half_ranges_deg.head<4>();
    settings.wrist_half_ranges_deg = options.joint_half_ranges_deg.tail<3>();
    settings.workspace_coverage = options.path_mode == "workspace";
    settings.tcp_armtip_m = options.tcp_armtip_m;
    settings.preserved_prefix_joints_rad = preserved;
    settings.minimum_body_z_m = options.height_constraints.minimum_body_z_m;
    settings.body_spheres = options.height_constraints.body_spheres;
    if (!options.workspace_config.empty()) {
        try {
            std::ifstream input(options.workspace_config);
            const auto config = nlohmann::json::parse(input);
            if (config.at("schema_version") != 1 || config.at("frame") != "rm75_base")
                throw std::runtime_error("workspace config must use schema 1 and rm75_base");
            auto vector = [](const nlohmann::json& value) -> Eigen::Vector3d {
                if (!value.is_array() || value.size() != 3) throw std::runtime_error("expected three coordinates");
                return {value[0].get<double>(), value[1].get<double>(), value[2].get<double>()};
            };
            for (const auto& box : config.value("allowed_regions", nlohmann::json::array()))
                settings.allowed_regions.push_back({vector(box.at("min_m")), vector(box.at("max_m"))});
            for (const auto& box : config.value("forbidden_boxes", nlohmann::json::array()))
                settings.forbidden_boxes.push_back({vector(box.at("min_m")), vector(box.at("max_m"))});
            for (const auto& sphere : config.value("forbidden_spheres", nlohmann::json::array()))
                settings.forbidden_spheres.push_back({vector(sphere.at("center_m")), sphere.at("radius_m").get<double>()});
            if (config.contains("minimum_body_z_m")) {
                settings.body_spheres.clear();
                settings.minimum_body_z_m = config.at("minimum_body_z_m").get<double>();
                if (!std::isfinite(settings.minimum_body_z_m)) throw std::runtime_error("invalid body height");
                for (const auto& body : config.at("body_spheres"))
                    settings.body_spheres.push_back({body.at("joint_index").get<int>(),
                        vector(body.at("center_m")), body.at("radius_m").get<double>(), body.at("name").get<std::string>()});
            }
            if (settings.allowed_regions.empty() && settings.forbidden_boxes.empty()
                && settings.forbidden_spheres.empty() && settings.body_spheres.empty())
                throw std::runtime_error("configure at least one allowed region or forbidden volume");
        } catch (const std::exception& error) {
            std::cerr << "Invalid workspace regions: " << error.what() << '\n';
            return false;
        }
    }
    options.height_constraints = settings;
    std::string error;
    if (!GenerateForceCalibrationPath(center, armtip_from_sensor, kinematics, settings, path, &error)) {
        std::cerr << "Cannot generate static calibration path: " << error << '\n';
        return false;
    }
    nlohmann::json document;
    document["schema_version"] = 1;
    document["mode"] = options.path_mode == "workspace" ? "workspace_coverage" : "local_seven_joint_sweep";
    document["joint_unit"] = "deg";
    document["pose_count"] = path.joints_rad.size();
    document["velocity_percent"] = options.velocity;
    document["settle_ms"] = 0;
    document["motion_allowed"] = true;
    document["preserved_prefix_count"] = path.preserved_prefix_count;
    document["minimum_window_duration_ms"] = options.minimum_window_duration_ms;
    document["samples_per_pose"] = options.samples_per_pose;
    document["return_to_center"] = !settings.workspace_coverage;
    document["position_note"] = "All seven joints sweep; wrist rotations retained; TCP is not fixed";
    document["collision_checked"] = false;
    if (!options.self_collision_config.is_null()) document["self_collision_guard"]=options.self_collision_config;
    document["workspace_config"] = options.workspace_config;
    document["workspace_constraints"] = {{"allowed_regions", nlohmann::json::array()},
        {"forbidden_boxes", nlohmann::json::array()}, {"forbidden_spheres", nlohmann::json::array()}};
    if (std::isfinite(settings.minimum_body_z_m)) {
        document["workspace_constraints"]["minimum_body_z_m"] = settings.minimum_body_z_m;
        document["workspace_constraints"]["body_spheres"] = nlohmann::json::array();
        for (const auto& body : settings.body_spheres)
            document["workspace_constraints"]["body_spheres"].push_back({{"name",body.name},
                {"joint_index",body.joint_index}, {"radius_m",body.radius_m},
                {"center_m",{body.center_m.x(),body.center_m.y(),body.center_m.z()}}});
        document["height_guard_m"] = .005;
        document["height_segment_checked"] = true;
    }
    auto coordinates = [](const Eigen::Vector3d& value) {
        return nlohmann::json::array({value.x(), value.y(), value.z()});
    };
    for (const auto& box : settings.allowed_regions)
        document["workspace_constraints"]["allowed_regions"].push_back(
            {{"min_m", coordinates(box.minimum_m)}, {"max_m", coordinates(box.maximum_m)}});
    for (const auto& box : settings.forbidden_boxes)
        document["workspace_constraints"]["forbidden_boxes"].push_back(
            {{"min_m", coordinates(box.minimum_m)}, {"max_m", coordinates(box.maximum_m)}});
    for (const auto& sphere : settings.forbidden_spheres)
        document["workspace_constraints"]["forbidden_spheres"].push_back(
            {{"center_m", coordinates(sphere.center_m)}, {"radius_m", sphere.radius_m}});
    for (int axis = 0; axis < 3; ++axis)
        document["probe_tcp_armtip_m"].push_back(options.tcp_armtip_m[axis]);
    if (settings.workspace_coverage) {
        document["workspace"]["candidate_count"] = path.workspace_candidate_count;
        document["workspace"]["candidate_cover_radius_m"] = path.candidate_cover_radius_m;
        document["workspace"]["coverage_note"] = "sampled kinematic candidates; not continuous-space or collision certification";
        for (int axis = 0; axis < 3; ++axis) {
            document["workspace"]["candidate_min_m"].push_back(path.workspace_min_m[axis]);
            document["workspace"]["candidate_max_m"].push_back(path.workspace_max_m[axis]);
        }
    }
    for (int axis = 0; axis < 7; ++axis)
        document["joint_half_ranges_deg"].push_back(options.joint_half_ranges_deg[axis]);
    document["maximum_step_deg"] = path.maximum_step_deg;
    document["maximum_gravity_separation_deg"] = path.maximum_gravity_separation_deg;
    for (int axis = 0; axis < 7; ++axis) document["center_joints_deg"].push_back(center[axis] * 180.0 / kPi);
    for (std::size_t i = 0; i < path.joints_rad.size(); ++i) {
        nlohmann::json point;
        point["index"] = i + 1;
        if (i < path.via_center.size()) point["via_center"] = path.via_center[i];
        for (int axis = 0; axis < 7; ++axis) point["joints_deg"].push_back(path.joints_rad[i][axis] * 180.0 / kPi);
        for (int axis = 0; axis < 3; ++axis) point["gravity_direction_sensor"].push_back(path.gravity_directions_sensor[i][axis]);
        if (settings.workspace_coverage)
            for (int axis = 0; axis < 3; ++axis)
                point["tcp_base_m"].push_back(path.tcp_positions_base_m[i][axis]);
        document["points"].push_back(std::move(point));
    }
    const auto filename = std::filesystem::absolute(options.path_output);
    std::filesystem::create_directories(filename.parent_path());
    std::ofstream output(filename);
    output << document.dump(2) << '\n';
    output.close();
    if (!output) return false;
    std::cout << "Automatic path: " << filename << "\nPoses: " << path.joints_rad.size()
              << "; max adjacent joint step: " << path.maximum_step_deg
              << " deg; gravity-direction spread: " << path.maximum_gravity_separation_deg
              << " deg\nMulti-direction gravity model conditioning passed.\n";
    if (settings.workspace_coverage)
        std::cout << "TCP candidate bounds m: [" << path.workspace_min_m.transpose() << "] .. ["
                  << path.workspace_max_m.transpose() << "]; candidates=" << path.workspace_candidate_count
                  << "; candidate covering radius=" << path.candidate_cover_radius_m << " m\n";
    return true;
}

bool MoveToTarget(RMCommand& command, RMStateReader& reader,
                   const Eigen::Matrix<double, 7, 1>& target,
                   const Options& options,
                   const Eigen::Matrix<double, 7, 1>* center = nullptr) {
    const auto feedback=reader.Latest();
    if (!feedback.valid || feedback.stale || feedback.arm_err || feedback.sys_err) {
        std::cerr << "No healthy current joint feedback before motion\n";
        return false;
    }
    const Eigen::Matrix<double, 7, 1> from = feedback.joints;
    RMKinematics kinematics;
    if (!ForceCalibrationAbovePlane(from, target, kinematics, options.height_constraints)) {
        if (!center || !ForceCalibrationAbovePlane(from, *center, kinematics, options.height_constraints)
            || !ForceCalibrationAbovePlane(*center, target, kinematics, options.height_constraints)) {
            std::cerr << "Component height constraint rejects joint motion\n";
            return false;
        }
        return MoveToTarget(command, reader, *center, options)
            && MoveToTarget(command, reader, target, options);
    }
    if (options.self_collision_check && !options.self_collision_check(from,target)) {
        if (center && ForceCalibrationAbovePlane(from,*center,kinematics,options.height_constraints)
            && ForceCalibrationAbovePlane(*center,target,kinematics,options.height_constraints)
            && options.self_collision_check(from,*center) && options.self_collision_check(*center,target))
            return MoveToTarget(command,reader,*center,options) && MoveToTarget(command,reader,target,options);
        std::cerr << "Self-collision guard rejected motion from actual feedback; no MoveJ sent\n";
        return false;
    }
    // The synchronous trajectory receiver must own the connection during MoveJ.
    reader.Stop();
    const int steps = std::max(1, static_cast<int>(std::ceil(
        (target - from).cwiseAbs().maxCoeff() / (10.0 * kPi / 180.0))));
    for (int step = 1; step <= steps; ++step) {
        if (g_stop_requested.load()) return false;
        const Eigen::Matrix<double, 7, 1> intermediate = from + (target - from) * (double(step) / steps);
        const RMResult moved = command.TryMoveJ(intermediate, options.velocity,
            options.move_timeout_s * 1000, &g_stop_requested);
        if (!moved) {
            std::cerr << "MoveJ failed: " << moved.message << '\n';
            return false;
        }
    }
    const auto started = reader.Start();
    if (!started) {
        std::cerr << "State reader restart failed: " << started.message << '\n';
        return false;
    }
    const auto sequence = reader.Latest().sequence;
    RobotStateSnapshot state;
    if (!reader.WaitForUpdate(sequence, std::chrono::milliseconds(2000), state)) {
        std::cerr << "No fresh feedback after MoveJ\n";
        return false;
    }
    return state.valid && !state.stale && state.arm_err == 0 && state.sys_err == 0;
}

bool ExitWorkspaceKeepout(RMCommand& command, RMStateReader& reader, const Options& options) {
    if (options.workspace_config.empty()) return true;
    try {
        std::ifstream input(options.workspace_config);
        const auto config = nlohmann::json::parse(input);
        if (config.value("exit_direction_base", std::vector<int>{}) != std::vector<int>{0, 0, 1})
            return true;
        const auto state = reader.Latest();
        if (!state.valid || state.stale || state.arm_err || state.sys_err) return false;
        const Eigen::Matrix3d rotation = RotationBaseFromControllerEuler(state.pose.tail<3>());
        const Eigen::Vector3d tcp = state.pose.head<3>() + rotation * options.tcp_armtip_m;
        double lift_m = 0;
        for (const auto& sphere : config.value("forbidden_spheres", nlohmann::json::array())) {
            const Eigen::Vector3d center(sphere.at("center_m").at(0).get<double>(),
                sphere.at("center_m").at(1).get<double>(), sphere.at("center_m").at(2).get<double>());
            const double radius = sphere.at("radius_m").get<double>();
            if ((tcp - center).norm() <= radius)
                lift_m = std::max(lift_m, center.z() + radius + 0.05 - tcp.z());
        }
        if (lift_m <= 0) return true;
        Eigen::Matrix<double, 6, 1> target = state.pose;
        target.z() += lift_m;
        std::cout << "Exit keep-out region: MoveL along Base +Z by " << lift_m << " m\n";
        reader.Stop();
        const auto moved = command.TryMoveL(target, options.velocity,
            options.move_timeout_s * 1000, &g_stop_requested);
        if (!moved) {
            std::cerr << "Exit MoveL failed: " << moved.message << '\n';
            return false;
        }
        const auto started = reader.Start();
        if (!started) return false;
        RobotStateSnapshot updated;
        const auto sequence = reader.Latest().sequence;
        return reader.WaitForUpdate(sequence, std::chrono::milliseconds(2000), updated)
            && updated.valid && !updated.stale && !updated.arm_err && !updated.sys_err;
    } catch (const std::exception& error) {
        std::cerr << "Cannot exit keep-out region: " << error.what() << '\n';
        return false;
    }
}

bool LoadResume(Options& options, ForceCalibrationPath& path,
                Eigen::Matrix<double, 7, 1>& center,
                std::vector<ForceCalibrationSample>& samples, std::string& prefix) {
    try {
        const auto snapshot = options.resume_csv + ".path.json";
        if (std::filesystem::exists(snapshot)) options.path_output = snapshot;
        std::ifstream route(options.path_output);
        const auto document = nlohmann::json::parse(route);
        if (document.contains("self_collision_guard")) {
            options.self_collision_config=document.at("self_collision_guard");
#ifdef CALIBRATION_HAS_SELF_COLLISION
            auto guard=std::make_shared<calibration_collision::Guard>(document.at("self_collision_guard"));
            options.self_collision_check=[guard](const auto& from,const auto& to) {
                const auto checked=guard->Segment(from,to);
                if (!checked.Valid()) std::cerr << "Self collision: "
                    << calibration_collision::Diagnostic(checked.failure).dump() << '\n';
                return checked.Valid();
            };
#else
            throw std::runtime_error("Path requires self-collision support; rebuild with BUILD_CALIBRATION_SELF_COLLISION=ON");
#endif
        }
        if (document.contains("workspace_constraints")
            && document["workspace_constraints"].contains("minimum_body_z_m")) {
            const auto& constraints = document["workspace_constraints"];
            options.height_constraints.minimum_body_z_m = constraints.at("minimum_body_z_m").get<double>();
            for (const auto& body : constraints.at("body_spheres")) {
                const auto& p = body.at("center_m");
                options.height_constraints.body_spheres.push_back({body.at("joint_index").get<int>(),
                    Eigen::Vector3d(p.at(0).get<double>(),p.at(1).get<double>(),p.at(2).get<double>()),
                    body.at("radius_m").get<double>(),body.at("name").get<std::string>()});
            }
            if (!std::isfinite(options.height_constraints.minimum_body_z_m)
                || options.height_constraints.body_spheres.empty()) throw std::runtime_error("invalid saved height envelope");
        }
        if (document.at("joint_unit") != "deg"
            || document.at("pose_count").get<int>() != options.pose_count
            || document.at("points").size() != static_cast<std::size_t>(options.pose_count)
            || document.at("center_joints_deg").size() != 7)
            throw std::runtime_error("original path size/unit mismatch");
        RMKinematics kinematics;
        auto parse = [&](const nlohmann::json& values) {
            if (values.size() != 7) throw std::runtime_error("path requires seven joints");
            Eigen::Matrix<double, 7, 1> q;
            for (int axis = 0; axis < 7; ++axis) {
                q[axis] = values.at(axis).get<double>() * kPi / 180.0;
                if (!std::isfinite(q[axis]) || q[axis] < kinematics.JointMinimums()[axis]
                    || q[axis] > kinematics.JointMaximums()[axis])
                    throw std::runtime_error("path joint outside model limits");
            }
            return q;
        };
        center = parse(document.at("center_joints_deg"));
        for (const auto& point : document.at("points")) {
            if (point.at("index").get<std::size_t>() != path.joints_rad.size() + 1)
                throw std::runtime_error("original path indices are not consecutive");
            path.joints_rad.push_back(parse(point.at("joints_deg")));
        }
        std::ifstream csv(options.resume_csv);
        if (!csv) throw std::runtime_error("cannot open partial CSV");
        std::ostringstream saved;
        saved << csv.rdbuf();
        prefix = saved.str();
        if (prefix.empty() || prefix.back() != '\n')
            throw std::runtime_error("partial CSV has an incomplete final line");
        std::istringstream lines(prefix);
        std::string line;
        bool generator = false, automatic = false, sensor_rpy = false, probe_rpy = false;
        int pending_pose = 0;
        while (std::getline(lines, line)) {
            if (line == "# generated_by=force_sensor_calibration_capture") generator = true;
            if (line == "# automatic=true; robot_motion=MoveJ") automatic = true;
            for (const auto& entry : {
                     std::make_pair(std::string("# sensor_to_tool_rpy_deg="), options.sensor_to_tool_rpy_deg),
                     std::make_pair(std::string("# probe_to_sensor_rpy_deg="), options.probe_to_sensor_rpy_deg)}) {
                if (line.rfind(entry.first, 0) == 0) {
                    Eigen::Vector3d angles;
                    if (!ParseVector3(line.substr(entry.first.size()), angles)
                        || (angles - entry.second).norm() > 1e-8)
                        throw std::runtime_error("partial CSV installation rotation mismatch");
                    if (entry.first.find("sensor_to_tool") != std::string::npos) sensor_rpy = true;
                    else probe_rpy = true;
                }
            }
            if (line.rfind("# pose=", 0) == 0) pending_pose = std::stoi(line.substr(7));
            if (line.empty() || line[0] == '#' || line.rfind("r00,", 0) == 0) continue;
            if (pending_pose != static_cast<int>(samples.size()) + 1)
                throw std::runtime_error("partial CSV pose indices are not consecutive");
            std::stringstream row(line);
            std::string value;
            ForceCalibrationSample sample;
            int column = 0;
            while (std::getline(row, value, ',')) {
                double number;
                if (column >= 15 || !ParseDouble(value, number))
                    throw std::runtime_error("partial CSV numeric row is invalid");
                if (column < 9) sample.rotation_base_from_sensor(column / 3, column % 3) = number;
                else sample.raw_wrench_sensor[column - 9] = number;
                ++column;
            }
            if (column != 15) throw std::runtime_error("partial CSV needs 15 columns");
            samples.push_back(sample);
        }
        if (!generator || !automatic || !sensor_rpy || !probe_rpy)
            throw std::runtime_error("partial CSV is missing automatic capture metadata");
        if (samples.empty())
            throw std::runtime_error("partial CSV contains zero accepted samples; choose a populated checkpoint");
        if (samples.size() >= path.joints_rad.size())
            throw std::runtime_error("partial CSV already contains all planned samples; no remaining poses");
        std::cout << "Resume: preserved " << samples.size() << " poses; next pose "
                  << samples.size() + 1 << '/' << options.pose_count << '\n';
        return true;
    } catch (const std::exception& error) {
        std::cerr << "Cannot resume: " << error.what() << '\n';
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        Usage(argv[0]);
        return 2;
    }
    if (options.check_resume) {
        ForceCalibrationPath path;
        Eigen::Matrix<double, 7, 1> center;
        std::vector<ForceCalibrationSample> samples;
        std::string prefix;
        return LoadResume(options, path, center, samples, prefix) ? 0 : 5;
    }
    bool workspace_execution = options.path_mode == "workspace" || options.resume_workspace;
    if (!options.resume_csv.empty() && !options.resume_workspace) {
        try {
            const auto snapshot = options.resume_csv + ".path.json";
            std::ifstream input(std::filesystem::exists(snapshot) ? snapshot : options.path_output);
            const auto mode = nlohmann::json::parse(input).value("mode", std::string());
            workspace_execution = mode != "local_seven_joint_sweep" && mode != "static_seven_joint_sweep"
                && mode != "static_wrist_rotation_grid";
        } catch (...) { workspace_execution = true; }
    }
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const Eigen::Matrix3d rotation_tool_from_sensor =
        RotationBaseFromControllerEuler(options.sensor_to_tool_rpy_deg * (kPi / 180.0));
    if (options.plan_only && !options.resume_workspace) {
        auto center = options.start_joints_rad;
        if (!options.start_joints_set) {
            RMCommand inspector(RMConnectionConfig{options.robot_ip, options.robot_port});
            inspector.SetQuiet(true);
            auto status = inspector.TryConnectTCPSocket();
            Eigen::Matrix<double, 6, 1> pose;
            int arm_error = 0, system_error = 0;
            if (status) status = inspector.TryReadArmState(center, pose, arm_error, system_error, 2000);
            if (!status || arm_error || system_error) {
                std::cerr << "Cannot read path center: " << status.message << '\n';
                return 4;
            }
        }
        ForceCalibrationPath path;
        return PlanAutomaticPath(options, center, rotation_tool_from_sensor, path) ? 0 : 5;
    }

    const std::filesystem::path output_path =
        std::filesystem::absolute(options.output_path);
    const auto unique_suffix =
        std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path partial_path = output_path.string()
        + ".partial." + std::to_string(::getpid())
        + "." + std::to_string(unique_suffix);
    std::error_code filesystem_error;
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(
            output_path.parent_path(), filesystem_error);
        if (filesystem_error) {
            std::cerr << "Cannot create output directory: "
                      << filesystem_error.message() << '\n';
            return 3;
        }
    }
    if (!options.overwrite
        && std::filesystem::exists(output_path)) {
        std::cerr << "Output already exists; use --overwrite: "
                  << output_path << '\n';
        return 3;
    }

    ForceCalibrationPath automatic_path;
    std::vector<ForceCalibrationSample> samples;
    Eigen::Matrix<double, 7, 1> original_center;
    std::string resume_prefix;
    if (!options.resume_csv.empty()
        && !LoadResume(options, automatic_path, original_center, samples, resume_prefix)) return 5;
    if (options.resume_workspace) {
        const std::vector<Eigen::Matrix<double, 7, 1>> preserved(
            automatic_path.joints_rad.begin(), automatic_path.joints_rad.begin() + samples.size());
        options.path_mode = "workspace";
        options.path_output = options.resume_csv + ".workspace.path.json";
        if (!PlanAutomaticPath(options, original_center, rotation_tool_from_sensor,
                               automatic_path, preserved)) return 5;
        std::cout << "Preserved " << samples.size() << " accepted samples; replanned only "
                  << options.pose_count - samples.size() << " remaining targets\n";
        if (options.plan_only) return 0;
    }

    ForceSensorConfig sensor_config;
    sensor_config.device = options.sensor_device;
    sensor_config.baud_rate = options.sensor_baud;
    sensor_config.protocol = options.sensor_protocol;
    sensor_config.stale_after =
        std::chrono::milliseconds(options.sensor_stale_ms);
    ForceSensorReader sensor(sensor_config);
    if (!sensor.Start()) {
        std::cerr << "Force sensor start failed: " << sensor.LastError() << '\n';
        return 4;
    }

    RMCommand command(RMConnectionConfig{
        options.robot_ip,
        options.robot_port,
    });
    command.SetQuiet(true);
    RMResult robot_result = command.TryConnectTCPSocket();
    if (!robot_result) {
        std::cerr << "Robot connection failed: " << robot_result.message << '\n';
        return 4;
    }
    Eigen::Matrix<double, 7, 1> initial_joints;
    Eigen::Matrix<double, 6, 1> initial_pose;
    int arm_error = 0;
    int system_error = 0;
    robot_result = command.TryReadArmState(
        initial_joints, initial_pose, arm_error, system_error, 2000);
    if (!robot_result || arm_error != 0 || system_error != 0) {
        std::cerr << "Initial robot state failed: " << robot_result.message
                  << " arm_err=" << arm_error
                  << " sys_err=" << system_error << '\n';
        return 4;
    }
    if (options.resume_csv.empty()) original_center = initial_joints;
    if (options.automatic && options.resume_csv.empty() && !PlanAutomaticPath(options, initial_joints,
                                                rotation_tool_from_sensor, automatic_path)) return 5;
    RMStateReader robot_reader(
        command,
        std::chrono::milliseconds(40),
        std::chrono::milliseconds(options.robot_stale_ms));
    robot_result = robot_reader.Start();
    if (!robot_result) {
        std::cerr << "Robot state reader failed: " << robot_result.message << '\n';
        return 4;
    }
    RobotStateSnapshot first_state;
    if (!robot_reader.WaitForUpdate(
            0, std::chrono::milliseconds(2000), first_state)) {
        std::cerr << "Robot state reader produced no update within 2 seconds\n";
        return 4;
    }
    std::string readiness_error;
    if (!WaitForInitialData(sensor,
                            robot_reader,
                            std::chrono::seconds(options.capture_timeout_s),
                            readiness_error)) {
        std::cerr << readiness_error << '\n';
        return 4;
    }

    std::ofstream output(partial_path, std::ios::trunc);
    if (!output) {
        std::cerr << "Cannot open partial output: " << partial_path << '\n';
        return 3;
    }
    if (options.resume_csv.empty()) WriteCsvHeader(output, options, rotation_tool_from_sensor);
    else output << resume_prefix << "# resumed_from=" << options.resume_csv
                << "; motion_allowed=true; settling_ms=0\n";
    if (options.automatic) {
        std::error_code error;
        std::filesystem::copy_file(options.path_output, partial_path.string() + ".path.json",
                                   std::filesystem::copy_options::none, error);
        if (error) {
            std::cerr << "Cannot preserve capture path: " << error.message() << '\n';
            return 3;
        }
    }

    BestEffortStopGuard stop_guard;
    if (options.automatic) stop_guard.Arm(command);
    if (options.automatic && workspace_execution) {
        std::cout << (options.self_collision_check
            ? "Workspace execution: self-collision and component height segments guarded; environment collision is not checked.\n"
            : "Workspace execution: configured component height segments checked; full-arm collision is not verified.\n");
        if (!ExitWorkspaceKeepout(command, robot_reader, options)) {
            std::cerr << "Workspace exit stopped; partial data: " << partial_path << '\n';
            return g_stop_requested.load() ? 130 : 4;
        }
    }
    std::cout << (options.automatic
                  ? "\nAUTOMATIC static calibration: MoveJ, average immediately; repeat.\n"
                  : "\nREAD-ONLY calibration capture ready. No robot motion command will be sent.\n")
              << "Output: " << output_path << '\n'
              << "Poses: " << options.pose_count
              << ", minimum samples per pose: " << options.samples_per_pose << '\n'
              << "Minimum capture window: "
              << options.minimum_window_duration_ms << " ms\n"
              << "Pairing uses host receive timestamps (max "
              << options.maximum_pair_skew_ms
              << " ms), not hardware synchronization.\n";

    samples.reserve(static_cast<std::size_t>(options.pose_count));
    int moved_pose_index = 0;
    for (int pose_index = static_cast<int>(samples.size()) + 1;
         pose_index <= options.pose_count && !g_stop_requested.load();) {
        if (options.automatic) {
            if (moved_pose_index != pose_index) {
                std::cout << "Moving to static pose " << pose_index << '/' << options.pose_count << '\n';
                if (!MoveToTarget(command, robot_reader,
                       automatic_path.joints_rad[static_cast<std::size_t>(pose_index - 1)], options, &original_center)) {
                    std::cerr << "Automatic capture stopped; partial data: " << partial_path << '\n';
                    return g_stop_requested.load() ? 130 : 4;
                }
                moved_pose_index = pose_index;
            }
        } else if (!PromptForCapture(pose_index, options.pose_count)) {
            g_stop_requested.store(true);
            break;
        }

        const ForceCalibrationCaptureResult capture = CapturePoseWindow(
            sensor, robot_reader, rotation_tool_from_sensor, options);
        if (!capture.valid) {
            std::cerr << "Capture rejected ["
                      << ForceCalibrationCaptureErrorString(capture.error)
                      << "]: " << capture.diagnostics.message
                      << "\n";
            if (options.automatic) {
                std::cerr << "Averaging capture failed; partial data: " << partial_path << '\n';
                return g_stop_requested.load() ? 130 : 5;
            }
            continue;
        }
        samples.push_back(capture.sample);
        WriteCsvSample(output, pose_index, capture);
        std::cout << "Accepted pose " << pose_index
                  << ": max skew=" << std::fixed << std::setprecision(3)
                  << capture.diagnostics.maximum_pair_time_delta_ms
                  << " ms, mean wrench=["
                  << capture.sample.raw_wrench_sensor.transpose() << "]\n";
        ++pose_index;
    }

    if (g_stop_requested.load()) {
        std::cerr << "Capture stopped; partial data kept at " << partial_path << '\n';
        return 130;
    }
    if (options.automatic) {
        if (!workspace_execution) {
            std::cout << "All static samples captured; returning to the initial joints\n";
            if (!MoveToTarget(command, robot_reader, original_center, options))
                return g_stop_requested.load() ? 130 : 4;
        } else {
            std::cout << "All samples captured; staying at the last workspace pose\n";
        }
        const auto stopped = RequestConfirmedStop(command);
        if (!stopped) {
            std::cerr << "Final stop failed: " << stopped.message << '\n';
            return 4;
        }
        stop_guard.Disarm();
    }

    ForceCalibration preview;
    double force_rms_n = 0.0;
    double torque_rms_nm = 0.0;
    double force_max_n = 0.0;
    double torque_max_nm = 0.0;
    std::string fit_error;
    if (!ForceCalibration::Fit(samples,
                               preview,
                               &force_rms_n,
                               &torque_rms_nm,
                               &fit_error,
                               &force_max_n,
                               &torque_max_nm)) {
        std::cerr << "Pose-diversity precheck failed: " << fit_error
                  << "\nPartial data kept at " << partial_path << '\n';
        return 5;
    }
    output << "# precheck_force_rms_n=" << force_rms_n
           << "; precheck_force_max_n=" << force_max_n
           << "; precheck_torque_rms_nm=" << torque_rms_nm
           << "; precheck_torque_max_nm=" << torque_max_nm << '\n';
    output << "# capture_complete=true\n";
    output.close();
    if (!output) {
        std::cerr << "Failed while flushing partial output: " << partial_path << '\n';
        return 3;
    }

    const ForceCalibrationCapturePublishResult publish =
        PublishForceCalibrationCaptureFile(partial_path.string(),
                                           output_path.string(),
                                           options.overwrite);
    if (!publish.published()) {
        std::cerr << "Cannot finalize capture CSV: " << publish.message
                  << "\nPartial data kept at " << partial_path << '\n';
        return 3;
    }
    if (publish.status
        == ForceCalibrationCapturePublishStatus::kPublishedPartialRetained) {
        std::cerr << "WARNING: " << publish.message << '\n';
    }

    std::cout << "\nCapture complete: " << output_path << '\n'
              << "Pose-diversity precheck passed. Next run "
                 "force_sensor_calibrate with independently measured R/t/TCP.\n";
    return 0;
}

// Synchronized capture-window implementation is colocated with its only CLI
// user to keep maintenance code in one source file.
#include <force_calibration_capture.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <unistd.h>

#if defined(__linux__)
#include <sys/syscall.h>
#endif

#include <Eigen/Geometry>

namespace {

constexpr double kRotationTolerance = 1e-6;

ForceCalibrationCaptureResult Failure(
    ForceCalibrationCaptureResult result,
    ForceCalibrationCaptureError error,
    std::string message,
    std::size_t rejected_index = std::numeric_limits<std::size_t>::max()) {
    result.valid = false;
    result.error = error;
    result.diagnostics.rejected_index = rejected_index;
    result.diagnostics.message = std::move(message);
    return result;
}

bool ProperRotation(const Eigen::Matrix3d& rotation) {
    if (!rotation.array().isFinite().all()) return false;
    return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
                   .norm()
            <= kRotationTolerance
        && std::abs(rotation.determinant() - 1.0) <= kRotationTolerance;
}

double WrappedAngleDifference(double lhs, double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

double AbsoluteMilliseconds(
    std::chrono::steady_clock::time_point lhs,
    std::chrono::steady_clock::time_point rhs) {
    const auto delta = lhs >= rhs ? lhs - rhs : rhs - lhs;
    return std::chrono::duration<double, std::milli>(delta).count();
}

bool ValidLimits(const ForceCalibrationCaptureLimits& limits) {
    return limits.minimum_samples > 0
        && limits.maximum_pair_time_delta.count() >= 0
        && limits.minimum_window_duration.count() >= 0;
}

}  // namespace

const char* ForceCalibrationCaptureErrorString(
    ForceCalibrationCaptureError error) noexcept {
    switch (error) {
        case ForceCalibrationCaptureError::kNone: return "none";
        case ForceCalibrationCaptureError::kInvalidLimits:
            return "invalid_limits";
        case ForceCalibrationCaptureError::kInsufficientSamples:
            return "insufficient_samples";
        case ForceCalibrationCaptureError::kWrenchLimitExceeded:
            return "wrench_limit_exceeded";
        case ForceCalibrationCaptureError::kInvalidSensorRotation:
            return "invalid_sensor_rotation";
        case ForceCalibrationCaptureError::kRobotInvalid:
            return "robot_invalid";
        case ForceCalibrationCaptureError::kRobotStale:
            return "robot_stale";
        case ForceCalibrationCaptureError::kRobotError:
            return "robot_error";
        case ForceCalibrationCaptureError::kWrenchInvalid:
            return "wrench_invalid";
        case ForceCalibrationCaptureError::kWrenchStale:
            return "wrench_stale";
        case ForceCalibrationCaptureError::kWrenchChecksumInvalid:
            return "wrench_checksum_invalid";
        case ForceCalibrationCaptureError::kWrenchIoError:
            return "wrench_io_error";
        case ForceCalibrationCaptureError::kNonFiniteValue:
            return "non_finite_value";
        case ForceCalibrationCaptureError::kTimestampMissing:
            return "timestamp_missing";
        case ForceCalibrationCaptureError::kTimestampSkew:
            return "timestamp_skew";
        case ForceCalibrationCaptureError::kRobotSequenceNotIncreasing:
            return "robot_sequence_not_increasing";
        case ForceCalibrationCaptureError::kSensorSequenceNotIncreasing:
            return "sensor_sequence_not_increasing";
        case ForceCalibrationCaptureError::kRobotTimestampNotIncreasing:
            return "robot_timestamp_not_increasing";
        case ForceCalibrationCaptureError::kSensorTimestampNotIncreasing:
            return "sensor_timestamp_not_increasing";
        case ForceCalibrationCaptureError::kWindowTooShort:
            return "window_too_short";
    }
    return "unknown";
}

ForceCalibrationCaptureResult CaptureForceCalibrationWindow(
    const std::vector<ForceCalibrationCaptureObservation>& observations,
    const Eigen::Matrix3d& rotation_tool_from_sensor,
    const ForceCalibrationCaptureLimits& limits) {
    ForceCalibrationCaptureResult result;
    result.diagnostics.observation_count = observations.size();

    if (!ValidLimits(limits)) {
        return Failure(std::move(result),
                       ForceCalibrationCaptureError::kInvalidLimits,
                       "capture limits must be finite and non-negative");
    }
    if (observations.size() < limits.minimum_samples) {
        std::ostringstream message;
        message << "capture window has " << observations.size()
                << " observations; at least " << limits.minimum_samples
                << " are required";
        return Failure(std::move(result),
                       ForceCalibrationCaptureError::kInsufficientSamples,
                       message.str());
    }
    if (!ProperRotation(rotation_tool_from_sensor)) {
        return Failure(std::move(result),
                       ForceCalibrationCaptureError::kInvalidSensorRotation,
                       "rotation_tool_from_sensor is not a proper rotation");
    }

    std::array<long double, 6> wrench_sum{};
    std::vector<Eigen::Matrix3d> rotations_base_from_tool;
    rotations_base_from_tool.reserve(observations.size());
    std::uint64_t previous_robot_sequence = 0;
    std::uint64_t previous_sensor_sequence = 0;
    std::chrono::steady_clock::time_point previous_robot_timestamp{};
    std::chrono::steady_clock::time_point previous_sensor_timestamp{};

    for (std::size_t index = 0; index < observations.size(); ++index) {
        const RobotStateSnapshot& robot = observations[index].robot;
        const WrenchSample& wrench = observations[index].wrench;

        if (!robot.valid) {
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kRobotInvalid,
                           "robot snapshot is invalid", index);
        }
        if (robot.stale) {
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kRobotStale,
                           "robot snapshot is stale", index);
        }
        if (robot.arm_err != 0 || robot.sys_err != 0) {
            std::ostringstream message;
            message << "robot snapshot has arm_err=" << robot.arm_err
                    << " sys_err=" << robot.sys_err;
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kRobotError,
                           message.str(), index);
        }
        if (!wrench.valid) {
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kWrenchInvalid,
                           "wrench sample is invalid", index);
        }
        if (wrench.stale) {
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kWrenchStale,
                           "wrench sample is stale", index);
        }
        if (!wrench.IntegrityValid()) {
            return Failure(
                std::move(result),
                ForceCalibrationCaptureError::kWrenchChecksumInvalid,
                "wrench protocol integrity is invalid", index);
        }
        if (wrench.io_error != 0
            || wrench.io_status != ForceSensorIoStatus::kStreaming) {
            std::ostringstream message;
            message << "wrench I/O is not healthy: status="
                    << ForceSensorIoStatusName(wrench.io_status)
                    << " errno=" << wrench.io_error;
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kWrenchIoError,
                           message.str(), index);
        }
        if (!robot.joints.array().isFinite().all()
            || !robot.pose.array().isFinite().all()
            || !std::all_of(wrench.wrench_si.begin(), wrench.wrench_si.end(),
                            [](double value) { return std::isfinite(value); })) {
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kNonFiniteValue,
                           "robot pose/joints or wrench contains a non-finite value",
                           index);
        }
        if (robot.received_at == std::chrono::steady_clock::time_point{}
            || wrench.monotonic_timestamp
                   == std::chrono::steady_clock::time_point{}) {
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kTimestampMissing,
                           "robot or wrench monotonic timestamp is missing",
                           index);
        }

        const double pair_delta_ms = AbsoluteMilliseconds(
            robot.received_at, wrench.monotonic_timestamp);
        result.diagnostics.maximum_pair_time_delta_ms = std::max(
            result.diagnostics.maximum_pair_time_delta_ms, pair_delta_ms);
        if (pair_delta_ms
            > static_cast<double>(limits.maximum_pair_time_delta.count())) {
            std::ostringstream message;
            message << "robot/wrench timestamp delta " << pair_delta_ms
                    << " ms exceeds "
                    << limits.maximum_pair_time_delta.count() << " ms";
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kTimestampSkew,
                           message.str(), index);
        }

        if (robot.sequence == 0
            || (index != 0 && robot.sequence <= previous_robot_sequence)) {
            std::ostringstream message;
            message << "robot sequence " << robot.sequence
                    << " is not strictly greater than "
                    << previous_robot_sequence;
            return Failure(
                std::move(result),
                ForceCalibrationCaptureError::kRobotSequenceNotIncreasing,
                message.str(), index);
        }
        previous_robot_sequence = robot.sequence;

        if (wrench.sequence == 0
            || (index != 0 && wrench.sequence <= previous_sensor_sequence)) {
            std::ostringstream message;
            message << "sensor sequence " << wrench.sequence
                    << " is not strictly greater than "
                    << previous_sensor_sequence;
            return Failure(
                std::move(result),
                ForceCalibrationCaptureError::kSensorSequenceNotIncreasing,
                message.str(), index);
        }
        previous_sensor_sequence = wrench.sequence;
        if (index == 0) {
            result.diagnostics.first_sensor_sequence = wrench.sequence;
        }
        result.diagnostics.last_sensor_sequence = wrench.sequence;

        if (index != 0 && robot.received_at <= previous_robot_timestamp) {
            return Failure(
                std::move(result),
                ForceCalibrationCaptureError::kRobotTimestampNotIncreasing,
                "robot timestamps are not strictly increasing", index);
        }
        previous_robot_timestamp = robot.received_at;
        if (index != 0
            && wrench.monotonic_timestamp <= previous_sensor_timestamp) {
            return Failure(
                std::move(result),
                ForceCalibrationCaptureError::kSensorTimestampNotIncreasing,
                "sensor timestamps are not strictly increasing", index);
        }
        previous_sensor_timestamp = wrench.monotonic_timestamp;

        const Eigen::Matrix3d rotation_base_from_tool =
            RotationBaseFromControllerEuler(robot.pose.tail<3>());
        if (!ProperRotation(rotation_base_from_tool)) {
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kNonFiniteValue,
                           "controller pose did not produce a proper rotation",
                           index);
        }
        rotations_base_from_tool.push_back(rotation_base_from_tool);
        for (std::size_t axis = 0; axis < wrench.wrench_si.size(); ++axis) {
            wrench_sum[axis] +=
                static_cast<long double>(wrench.wrench_si[axis]);
        }
    }

    result.diagnostics.robot_window_duration_ms =
        std::chrono::duration<double, std::milli>(
            observations.back().robot.received_at
            - observations.front().robot.received_at)
            .count();
    result.diagnostics.sensor_window_duration_ms =
        std::chrono::duration<double, std::milli>(
            observations.back().wrench.monotonic_timestamp
            - observations.front().wrench.monotonic_timestamp)
            .count();
    const double minimum_window_duration_ms =
        static_cast<double>(limits.minimum_window_duration.count());
    if (result.diagnostics.robot_window_duration_ms
            < minimum_window_duration_ms
        || result.diagnostics.sensor_window_duration_ms
               < minimum_window_duration_ms) {
        std::ostringstream message;
        message << "capture window is too short: robot_duration_ms="
                << result.diagnostics.robot_window_duration_ms
                << " sensor_duration_ms="
                << result.diagnostics.sensor_window_duration_ms
                << " required_ms=" << minimum_window_duration_ms;
        return Failure(std::move(result),
                       ForceCalibrationCaptureError::kWindowTooShort,
                       message.str());
    }

    // Measure the complete window span, not merely displacement from its first
    // or last observation. Spans are recorded without rejecting pose changes.
    for (std::size_t lhs = 0; lhs < observations.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1; rhs < observations.size(); ++rhs) {
            for (int joint = 0; joint < 7; ++joint) {
                result.diagnostics.maximum_joint_span_rad = std::max(
                    result.diagnostics.maximum_joint_span_rad,
                    std::abs(WrappedAngleDifference(
                        observations[lhs].robot.joints[joint],
                        observations[rhs].robot.joints[joint])));
            }
            result.diagnostics.maximum_position_span_m = std::max(
                result.diagnostics.maximum_position_span_m,
                (observations[lhs].robot.pose.head<3>()
                 - observations[rhs].robot.pose.head<3>())
                    .norm());
            const Eigen::Matrix3d relative_rotation =
                rotations_base_from_tool[rhs]
                * rotations_base_from_tool[lhs].transpose();
            result.diagnostics.maximum_orientation_span_rad = std::max(
                result.diagnostics.maximum_orientation_span_rad,
                Eigen::AngleAxisd(relative_rotation).angle());
        }
    }

    for (std::size_t axis = 0; axis < wrench_sum.size(); ++axis) {
        const long double mean =
            wrench_sum[axis] / static_cast<long double>(observations.size());
        const double value = static_cast<double>(mean);
        if (!std::isfinite(value)) {
            return Failure(std::move(result),
                           ForceCalibrationCaptureError::kNonFiniteValue,
                           "mean raw wrench is not finite");
        }
        result.sample.raw_wrench_sensor[static_cast<int>(axis)] = value;
    }
    result.diagnostics.mean_raw_wrench_sensor =
        result.sample.raw_wrench_sensor;

    // Align quaternion signs before averaging the measured orientations.
    const Eigen::Quaterniond reference(rotations_base_from_tool.front());
    Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
    for (const Eigen::Matrix3d& rotation : rotations_base_from_tool) {
        Eigen::Quaterniond quaternion(rotation);
        if (reference.coeffs().dot(quaternion.coeffs()) < 0.0) {
            quaternion.coeffs() *= -1.0;
        }
        quaternion_sum += quaternion.coeffs();
    }
    if (!quaternion_sum.array().isFinite().all()
        || quaternion_sum.norm() <= 1e-12) {
        return Failure(std::move(result),
                       ForceCalibrationCaptureError::kNonFiniteValue,
                       "mean robot orientation is undefined");
    }
    Eigen::Quaterniond mean_rotation;
    mean_rotation.coeffs() = quaternion_sum.normalized();
    result.sample.rotation_base_from_sensor =
        mean_rotation.toRotationMatrix() * rotation_tool_from_sensor;
    if (!ProperRotation(result.sample.rotation_base_from_sensor)) {
        return Failure(std::move(result),
                       ForceCalibrationCaptureError::kNonFiniteValue,
                       "computed rotation_base_from_sensor is invalid");
    }

    result.valid = true;
    result.error = ForceCalibrationCaptureError::kNone;
    result.diagnostics.message = "capture window accepted";
    return result;
}

std::string FormatForceCalibrationCaptureRpyDeg(
    const Eigen::Vector3d& sensor_to_tool_rpy_deg) {
    std::ostringstream output;
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << sensor_to_tool_rpy_deg.x() << ','
           << sensor_to_tool_rpy_deg.y() << ','
           << sensor_to_tool_rpy_deg.z();
    return output.str();
}

ForceCalibrationCapturePublishResult PublishForceCalibrationCaptureFile(
    const std::string& partial_path,
    const std::string& output_path,
    bool overwrite) {
    ForceCalibrationCapturePublishResult result;
    if (overwrite) {
        if (::rename(partial_path.c_str(), output_path.c_str()) == 0) {
            result.status = ForceCalibrationCapturePublishStatus::kPublished;
            result.message = "capture file published by atomic replacement";
            return result;
        }
        const int saved_errno = errno;
        result.message = "cannot atomically replace capture file: "
            + std::string(std::strerror(saved_errno));
        return result;
    }

#if defined(__linux__) && defined(SYS_renameat2)
    constexpr unsigned int kRenameNoReplace = 1U;
    if (::syscall(SYS_renameat2,
                  AT_FDCWD,
                  partial_path.c_str(),
                  AT_FDCWD,
                  output_path.c_str(),
                  kRenameNoReplace)
        == 0) {
        result.status = ForceCalibrationCapturePublishStatus::kPublished;
        result.message = "capture file published without replacement";
        return result;
    }
    const int rename_errno = errno;
    if (rename_errno == EEXIST) {
        result.status = ForceCalibrationCapturePublishStatus::kDestinationExists;
        result.message = "capture output already exists";
        return result;
    }
    const bool rename_no_replace_unsupported = rename_errno == ENOSYS
        || rename_errno == EINVAL
#if defined(EOPNOTSUPP)
        || rename_errno == EOPNOTSUPP
#endif
#if defined(ENOTSUP)
        || rename_errno == ENOTSUP
#endif
        ;
    if (!rename_no_replace_unsupported) {
        result.message = "cannot publish capture file without replacement: "
            + std::string(std::strerror(rename_errno));
        return result;
    }
#endif

    // Portable POSIX fallback: creating the final hard link is atomic and
    // fails if any directory entry already owns output_path. If hard links are
    // unsupported, fail closed and retain the partial file.
    if (::link(partial_path.c_str(), output_path.c_str()) != 0) {
        const int link_errno = errno;
        if (link_errno == EEXIST) {
            result.status = ForceCalibrationCapturePublishStatus::kDestinationExists;
            result.message = "capture output already exists";
        } else {
            result.message = "cannot publish capture file with no-clobber fallback: "
                + std::string(std::strerror(link_errno));
        }
        return result;
    }
    if (::unlink(partial_path.c_str()) != 0) {
        const int unlink_errno = errno;
        result.status =
            ForceCalibrationCapturePublishStatus::kPublishedPartialRetained;
        result.message = "capture file published, but stale partial name remains: "
            + std::string(std::strerror(unlink_errno));
        return result;
    }
    result.status = ForceCalibrationCapturePublishStatus::kPublished;
    result.message = "capture file published without replacement";
    return result;
}
