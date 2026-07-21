#include <force_calibration_capture.hpp>

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
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

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
    unsigned int sensor_baud = 115200;
    std::string output_path;
    Eigen::Vector3d sensor_to_tool_rpy_deg = Eigen::Vector3d::Zero();
    bool sensor_to_tool_rpy_set = false;
    int pose_count = 8;
    int samples_per_pose = 20;
    int capture_timeout_s = 10;
    int sensor_stale_ms = 50;
    int robot_stale_ms = 100;
    int maximum_pair_skew_ms = 50;
    int minimum_window_duration_ms = 500;
    bool overwrite = false;
};

void Usage(const char* program) {
    std::cout
        << "Usage: " << program << " --robot-ip IP --sensor-device PATH\n"
        << "       --sensor-to-tool-rpy-deg RX,RY,RZ --output FILE [options]\n\n"
        << "Interactively capture synchronized RM75 pose + raw wrench rows for\n"
        << "force_sensor_calibrate. This program is read-only and never sends\n"
        << "MoveJ, ServoJ, Hold or Stop commands.\n\n"
        << "The force sensor uses Haptron Modbus RTU read-only queries.\n\n"
        << "Required:\n"
        << "  --robot-ip IP\n"
        << "  --sensor-device PATH\n"
        << "  --sensor-to-tool-rpy-deg RX,RY,RZ\n"
        << "      Independently measured R_tool_from_sensor = Rz*Ry*Rx\n"
        << "  --output FILE\n\n"
        << "Options:\n"
        << "  --robot-port PORT          default 8080\n"
        << "  --sensor-baud BAUD         default 115200\n"
        << "  --poses N                  default 8, minimum 6\n"
        << "  --samples-per-pose N       default 20, minimum 5\n"
        << "  --capture-timeout-sec SEC  default 10\n"
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

bool ParseOptions(int argc, char** argv, Options& options) {
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
        } else if (argument == "--poses") {
            if (integer < 6 || integer > 100) return false;
            options.pose_count = static_cast<int>(integer);
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

    if (options.robot_ip.empty() || options.robot_ip.size() >= 16
        || options.sensor_device.empty() || options.output_path.empty()
        || !options.sensor_to_tool_rpy_set) {
        return false;
    }
    if (options.minimum_window_duration_ms
        >= options.capture_timeout_s * 1000) {
        std::cerr << "--min-window-duration-ms must be shorter than "
                     "--capture-timeout-sec\n";
        return false;
    }
    return true;
}

bool RawWrenchWithinLimits(const WrenchSample& sample) {
    for (std::size_t axis = kForceX; axis <= kForceZ; ++axis) {
        if (!std::isfinite(sample.wrench_si[axis])
            || std::abs(sample.wrench_si[axis]) > kRawForceLimitN) {
            return false;
        }
    }
    for (std::size_t axis = kTorqueX; axis <= kTorqueZ; ++axis) {
        if (!std::isfinite(sample.wrench_si[axis])
            || std::abs(sample.wrench_si[axis]) > kRawTorqueLimitNm) {
            return false;
        }
    }
    return true;
}

void WriteCsvHeader(std::ostream& output,
                    const Options& options,
                    const Eigen::Matrix3d& rotation_tool_from_sensor) {
    output << "# generated_by=force_sensor_calibration_capture\n"
           << "# read_only=true; no_robot_motion_commands=true\n"
           << "# pairing=bounded_host_monotonic_receive_time; not_hardware_sync\n"
           << "# robot=" << options.robot_ip << ':' << options.robot_port << '\n'
           << "# sensor_device=" << options.sensor_device
           << "; sensor_baud=" << options.sensor_baud
           << "; sensor_protocol="
           << ForceSensorProtocolName(
                  ForceSensorProtocol::kHaptronModbusRtu)
           << '\n'
           << "# sensor_to_tool_rpy_deg="
           << FormatForceCalibrationCaptureRpyDeg(
                  options.sensor_to_tool_rpy_deg)
           << '\n'
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
        if (wrench.valid && !wrench.stale && wrench.checksum_valid
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
    std::vector<ForceCalibrationCaptureObservation> observations;
    observations.reserve(static_cast<std::size_t>(options.samples_per_pose));
    std::uint64_t last_sensor_sequence = sensor.LatestSample().sequence;
    std::uint64_t last_robot_sequence = robot.Latest().sequence;
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(options.capture_timeout_s);
    std::string collection_error;

    while (!g_stop_requested.load()
           && std::chrono::steady_clock::now() < deadline) {
        bool enough_duration = false;
        if (observations.size() >= 2) {
            const double robot_duration_ms =
                std::chrono::duration<double, std::milli>(
                    observations.back().robot.received_at
                    - observations.front().robot.received_at)
                    .count();
            const double sensor_duration_ms =
                std::chrono::duration<double, std::milli>(
                    observations.back().wrench.monotonic_timestamp
                    - observations.front().wrench.monotonic_timestamp)
                    .count();
            enough_duration = robot_duration_ms
                    >= static_cast<double>(options.minimum_window_duration_ms)
                && sensor_duration_ms
                    >= static_cast<double>(options.minimum_window_duration_ms);
        }
        if (observations.size()
                >= static_cast<std::size_t>(options.samples_per_pose)
            && enough_duration) {
            break;
        }

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
        if (!robot_result) {
            collection_error = "robot state reader failed: " + robot_result.message;
            break;
        }
        if (!RawWrenchWithinLimits(wrench)) {
            collection_error = "raw wrench exceeded 50 N / 5 N*m capture limit";
            break;
        }
        observations.push_back({state, wrench});
    }

    ForceCalibrationCaptureLimits limits;
    limits.minimum_samples = static_cast<std::size_t>(options.samples_per_pose);
    limits.maximum_pair_time_delta =
        std::chrono::milliseconds(options.maximum_pair_skew_ms);
    limits.minimum_window_duration =
        std::chrono::milliseconds(options.minimum_window_duration_ms);
    ForceCalibrationCaptureResult result = CaptureForceCalibrationWindow(
        observations, rotation_tool_from_sensor, limits);
    if (!collection_error.empty()) {
        result.valid = false;
        result.diagnostics.message = collection_error;
    } else if (g_stop_requested.load()) {
        result.valid = false;
        result.diagnostics.message = "capture interrupted";
    }
    return result;
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

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        Usage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

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

    const Eigen::Matrix3d rotation_tool_from_sensor =
        RotationBaseFromControllerEuler(
            options.sensor_to_tool_rpy_deg * (kPi / 180.0));

    ForceSensorConfig sensor_config;
    sensor_config.device = options.sensor_device;
    sensor_config.baud_rate = options.sensor_baud;
    sensor_config.stale_after =
        std::chrono::milliseconds(options.sensor_stale_ms);
    ForceSensorReader sensor(sensor_config);
    if (!sensor.Start()) {
        std::cerr << "Force sensor start failed: " << sensor.LastError() << '\n';
        return 4;
    }

    RMCommand command;
    command.quiet = true;
    command.rlm_port = options.robot_port;
    std::strncpy(command.rlm_ip,
                 options.robot_ip.c_str(),
                 sizeof(command.rlm_ip) - 1);
    command.rlm_ip[sizeof(command.rlm_ip) - 1] = '\0';
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
    WriteCsvHeader(output, options, rotation_tool_from_sensor);

    std::cout << "\nREAD-ONLY calibration capture ready. No robot motion command "
                 "will be sent.\n"
              << "Output: " << output_path << '\n'
              << "Poses: " << options.pose_count
              << ", minimum samples per pose: " << options.samples_per_pose << '\n'
              << "Minimum capture window: "
              << options.minimum_window_duration_ms << " ms\n"
              << "Pairing uses host receive timestamps (max "
              << options.maximum_pair_skew_ms
              << " ms), not hardware synchronization.\n";

    std::vector<ForceCalibrationSample> samples;
    samples.reserve(static_cast<std::size_t>(options.pose_count));
    for (int pose_index = 1;
         pose_index <= options.pose_count && !g_stop_requested.load();) {
        if (!PromptForCapture(pose_index, options.pose_count)) {
            g_stop_requested.store(true);
            break;
        }

        const ForceCalibrationCaptureResult capture = CapturePoseWindow(
            sensor, robot_reader, rotation_tool_from_sensor, options);
        if (!capture.valid) {
            std::cerr << "Capture rejected ["
                      << ForceCalibrationCaptureErrorString(capture.error)
                      << "]: " << capture.diagnostics.message
                      << "\nKeep the arm stationary and retry this pose.\n";
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
        && limits.minimum_window_duration.count() >= 0
        && std::isfinite(limits.maximum_joint_span_rad)
        && limits.maximum_joint_span_rad >= 0.0
        && std::isfinite(limits.maximum_position_span_m)
        && limits.maximum_position_span_m >= 0.0
        && std::isfinite(limits.maximum_orientation_span_rad)
        && limits.maximum_orientation_span_rad >= 0.0;
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
        case ForceCalibrationCaptureError::kWindowMotion:
            return "window_motion";
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
        if (!wrench.checksum_valid) {
            return Failure(
                std::move(result),
                ForceCalibrationCaptureError::kWrenchChecksumInvalid,
                "wrench checksum is invalid", index);
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
    // or last observation. A move that returns to its start is still rejected.
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

    if (result.diagnostics.maximum_joint_span_rad
            > limits.maximum_joint_span_rad
        || result.diagnostics.maximum_position_span_m
               > limits.maximum_position_span_m
        || result.diagnostics.maximum_orientation_span_rad
               > limits.maximum_orientation_span_rad) {
        std::ostringstream message;
        message << "capture window is not stationary: joint_span_rad="
                << result.diagnostics.maximum_joint_span_rad
                << " position_span_m="
                << result.diagnostics.maximum_position_span_m
                << " orientation_span_rad="
                << result.diagnostics.maximum_orientation_span_rad;
        return Failure(std::move(result),
                       ForceCalibrationCaptureError::kWindowMotion,
                       message.str());
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

    // The normalized quaternion sum is an unambiguous local mean because the
    // stationary-window gate keeps all rotations in the same hemisphere.
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
