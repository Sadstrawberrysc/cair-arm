// RM75 production controller entry.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <json.hpp>

#include <contact_sensing.hpp>
#include <force_calibration.hpp>
#include <force_sensor.hpp>
#include <realman_command.hpp>
#include <redis_bridge.hpp>
#include <rm75_control.hpp>

namespace {

std::atomic<bool> g_stop_requested{false};
constexpr std::chrono::milliseconds kMinimumAsyncServoSendGap{10};
constexpr const char* kImplicitProfileName =
    "rm75_v6_axial_force_3n_tcp188_60s";
constexpr double kRuntimeTareMaximumJointSpanDeg = 0.02;
// RM75 pose feedback is quantized at roughly 0.001 rad. At the legacy
// 188 mm probe TCP this can appear as about 0.19 mm of endpoint motion even
// while joint feedback and the physical arm remain stationary. Keep the
// independent orientation gate and allow a consistent endpoint envelope.
constexpr double kRuntimeTareMaximumTcpSpanMm = 0.35;
constexpr double kRuntimeTareMaximumOrientationSpanDeg = 0.08;
// Provisional commissioning remains explicitly gated by the no-contact tare,
// old six-axis sensor range checks, low motion speed and --execute.  The
// temporary 4 N/3.2 N retreat state machine and 5 N compensated axial gate
// are intentionally not part of the default legacy-style force path.
constexpr bool kProvisionalCommissioningLocked = false;

void HandleSignal(int) {
    g_stop_requested.store(true);
}

// -------------------------------------------------------------------------
// Controller configuration and command-line contract
// -------------------------------------------------------------------------

enum class ControllerMode {
    kObserve,
    kDryRun,
    kExecute,
};

const char* ModeName(ControllerMode mode) {
    switch (mode) {
        case ControllerMode::kObserve: return "observe";
        case ControllerMode::kDryRun: return "dry_run";
        case ControllerMode::kExecute: return "execute";
    }
    return "unknown";
}

struct Options {
    ControllerMode mode = ControllerMode::kObserve;
    bool simulate = false;
    bool redis_enabled = true;
    std::string robot_ip = "192.168.50.254";
    int robot_port = 8080;
    std::string sensor_device = "/dev/ttyUSB0";
    unsigned int sensor_baud = 115200;
    int sensor_stale_ms = 50;
    int robot_stale_ms = 100;
    int command_stale_ms = 500;
    std::string calibration_path;
    std::string expected_sensor_id;
    std::string probe_model_path = "../model/Lprobe-IFS.STL";
    std::string redis_host = "127.0.0.1";
    int redis_port = 7777;
    int period_ms = 20;
    int duration_s = 0;
    int publish_every = 5;
    int execute_warmup_s = 2;
    int tare_no_contact_s = 0;
    std::string runtime_log_path;

    double desired_force_n = -1.0;
    double raw_force_limit_n = 50.0;
    double raw_torque_limit_nm = 5.0;
    double approach_speed_cm_s = 0.0;
    double approach_direction_tool_z = 0.0;
    double maximum_linear_speed_cm_s = 0.5;
    double maximum_approach_distance_mm = 5.0;
    double maximum_orientation_excursion_deg = 5.0;
    double max_tracking_joint_error_deg = 5.0;
    double max_tracking_position_error_mm = 3.0;
    double max_tracking_orientation_error_deg = 2.0;
    double manual_y_m = 0.0;
    double manual_rz_deg = 0.0;
    double contact_pitch_gain_rad_per_m = 0.0;
    int manual_phase = -1;
    bool manual_action = false;
    bool manual_terminate = false;
    bool allow_provisional_force_control = false;
    bool implicit_commissioning_profile = false;
    // Reproduce the remaining legacy six-axis force/contact dynamics.
    // Unfiltered force limits and all RM75 motion gates remain independent.
    bool legacy_six_axis_force_stage = false;

    bool allow_near_singularity = false;
    bool enable_force_retract = false;
    double retract_direction_tool_z = 0.0;
    double retract_distance_mm = 5.0;
    double retract_speed_cm_s = 0.2;
};

// -------------------------------------------------------------------------
// Non-blocking runtime recording
// -------------------------------------------------------------------------

struct RuntimeLogRow {
    std::uint64_t cycle = 0;
    std::int64_t monotonic_ns = 0;
    double work_us = 0.0;
    double cycle_interval_us = 0.0;
    double deadline_lateness_us = 0.0;
    bool deadline_missed = false;
    Rm75SupervisorState state = Rm75SupervisorState::kInitializing;
    std::uint64_t robot_sequence = 0;
    std::uint64_t sensor_sequence = 0;
    std::int64_t robot_received_monotonic_ns = 0;
    std::int64_t sensor_received_monotonic_ns = 0;
    std::int64_t sensor_source_unix_ns = 0;
    bool robot_valid = false;
    bool wrench_valid = false;
    int arm_error = 0;
    int system_error = 0;
    double robot_age_ms = 0.0;
    double sensor_age_ms = 0.0;
    bool checksum_valid = false;
    ForceSensorIoStatus sensor_io_status = ForceSensorIoStatus::kDisconnected;
    int sensor_io_error = 0;
    bool redis_enabled = false;
    bool redis_subscriber_connected = false;
    bool redis_publisher_connected = false;
    bool command_present = false;
    bool command_valid = false;
    bool command_fresh = false;
    std::uint64_t command_sequence = 0;
    std::uint64_t command_producer_sequence = 0;
    std::uint64_t command_connection_generation = 0;
    double command_age_ms = 0.0;
    bool command_action_enabled = false;
    bool command_terminate = false;
    int command_phase_index = -1;
    double command_model_y_m = 0.0;
    double command_model_rz_deg = 0.0;
    double command_desired_force_n = -1.0;
    std::string command_hold_reason;
    std::uint64_t servo_submitted_sequence = 0;
    std::uint64_t servo_consumed_sequence = 0;
    std::uint64_t servo_discarded_sequence = 0;
    std::uint64_t servo_pending_sequence = 0;
    std::uint64_t servo_sent_sequence = 0;
    std::uint64_t servo_result_sequence = 0;
    RMErrorCode servo_result_code = RMErrorCode::kNone;
    bool servo_outstanding = false;
    Eigen::Matrix<double, 7, 1> actual_joints =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 1> actual_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> raw_wrench =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> compensated_wrench =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> control_wrench =
        Eigen::Matrix<double, 6, 1>::Zero();
    ContactEstimate contact;
    Eigen::Vector3d filtered_contact_point = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 1> requested_delta =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> desired_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 7, 1> target_joints =
        Eigen::Matrix<double, 7, 1>::Zero();
    std::string fault;
};

struct CycleTimingResult {
    std::chrono::steady_clock::time_point finished{};
    double work_us = 0.0;
    double deadline_lateness_us = 0.0;
    bool deadline_missed = false;
};

class AsyncRuntimeLogger {
public:
    bool Start(const std::string& path, std::string* error) {
        if (path.empty()) return true;
        path_ = path;
        const std::filesystem::path output(path);
        std::error_code ec;
        if (!output.parent_path().empty()) {
            std::filesystem::create_directories(output.parent_path(), ec);
            if (ec) {
                if (error != nullptr) *error = "cannot create log directory";
                return false;
            }
        }
        stream_.open(path);
        if (!stream_) {
            if (error != nullptr) *error = "cannot open runtime log: " + path;
            return false;
        }
        stream_ << "cycle,monotonic_ns,work_us,cycle_interval_us,"
                   "deadline_lateness_us,deadline_missed,state,"
                   "robot_sequence,sensor_sequence,"
                   "robot_received_monotonic_ns,sensor_received_monotonic_ns,"
                   "sensor_source_unix_ns,robot_valid,wrench_valid,"
                   "arm_error,system_error,robot_age_ms,sensor_age_ms,"
                   "checksum_valid,sensor_io_status,sensor_io_error,"
                   "redis_enabled,redis_subscriber_connected,"
                   "redis_publisher_connected,command_present,command_valid,"
                   "command_fresh,command_sequence,command_producer_sequence,"
                   "command_connection_generation,"
                   "command_age_ms,command_action_enabled,command_terminate,"
                   "command_phase_index,command_model_y_m,command_model_rz_deg,"
                   "command_desired_force_n,command_hold_reason,"
                   "servo_submitted_sequence,servo_consumed_sequence,"
                   "servo_discarded_sequence,servo_pending_sequence,"
                   "servo_sent_sequence,"
                   "servo_result_sequence,servo_result_code,servo_outstanding,"
                   "actual_j1_rad,actual_j2_rad,actual_j3_rad,actual_j4_rad,"
                   "actual_j5_rad,actual_j6_rad,actual_j7_rad,"
                   "actual_x_m,actual_y_m,actual_z_m,actual_rx_rad,"
                   "actual_ry_rad,actual_rz_rad,"
                   "raw_fx_n,raw_fy_n,raw_fz_n,raw_tx_nm,raw_ty_nm,raw_tz_nm,"
                   "tool_fx_n,tool_fy_n,tool_fz_n,tool_tx_nm,tool_ty_nm,tool_tz_nm,"
                   "control_fx_n,control_fy_n,control_fz_n,"
                   "control_tx_nm,control_ty_nm,control_tz_nm,"
                   "contact_valid,contact_x_m,contact_y_m,contact_z_m,contact_residual_nm,"
                   "contact_point_error_m,filtered_contact_x_m,"
                   "filtered_contact_y_m,filtered_contact_z_m,"
                   "requested_dx_tool_m,requested_dy_tool_m,requested_dz_tool_m,"
                   "requested_drx_tool_rad,requested_dry_tool_rad,"
                   "requested_drz_tool_rad,"
                   "desired_x_m,desired_y_m,desired_z_m,desired_rx_rad,desired_ry_rad,desired_rz_rad,"
                   "j1_rad,j2_rad,j3_rad,j4_rad,j5_rad,j6_rad,j7_rad,fault\n";
        queue_.resize(kMaximumRows);
        queue_head_ = 0;
        queue_size_ = 0;
        running_.store(true);
        thread_ = std::thread(&AsyncRuntimeLogger::ThreadMain, this);
        return true;
    }

    CycleTimingResult PushAndMeasure(
        RuntimeLogRow row,
        std::chrono::steady_clock::time_point cycle_started,
        std::chrono::steady_clock::time_point deadline) {
        CycleTimingResult timing;
        if (!running_.load()) {
            timing.finished = std::chrono::steady_clock::now();
            timing.work_us = std::chrono::duration<double, std::micro>(
                                 timing.finished - cycle_started)
                                 .count();
            timing.deadline_missed = timing.finished > deadline;
            timing.deadline_lateness_us = timing.deadline_missed
                ? std::chrono::duration<double, std::micro>(
                      timing.finished - deadline)
                      .count()
                : 0.0;
            return timing;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_size_ >= kMaximumRows) {
                queue_head_ = (queue_head_ + 1) % kMaximumRows;
                --queue_size_;
                ++dropped_rows_;
            }
            const std::size_t index =
                (queue_head_ + queue_size_) % kMaximumRows;
            queue_[index] = std::move(row);
            timing.finished = std::chrono::steady_clock::now();
            timing.work_us = std::chrono::duration<double, std::micro>(
                                 timing.finished - cycle_started)
                                 .count();
            timing.deadline_missed = timing.finished > deadline;
            timing.deadline_lateness_us = timing.deadline_missed
                ? std::chrono::duration<double, std::micro>(
                      timing.finished - deadline)
                      .count()
                : 0.0;
            queue_[index].work_us = timing.work_us;
            queue_[index].deadline_lateness_us =
                timing.deadline_lateness_us;
            queue_[index].deadline_missed = timing.deadline_missed;
            ++queue_size_;
        }
        condition_.notify_one();
        return timing;
    }

    void Stop() {
        if (!running_.exchange(false)) return;
        condition_.notify_all();
        if (thread_.joinable()) thread_.join();
        stream_.close();
    }

    ~AsyncRuntimeLogger() { Stop(); }
    std::uint64_t DroppedRows() const { return dropped_rows_.load(); }
    const std::string& Path() const { return path_; }

private:
    void ThreadMain() {
        stream_ << std::fixed << std::setprecision(9);
        for (;;) {
            RuntimeLogRow row;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [&]() {
                    return !running_.load() || queue_size_ != 0;
                });
                if (queue_size_ == 0 && !running_.load()) break;
                row = std::move(queue_[queue_head_]);
                queue_head_ = (queue_head_ + 1) % kMaximumRows;
                --queue_size_;
            }
            stream_ << row.cycle << ',' << row.monotonic_ns << ','
                    << row.work_us << ',' << row.cycle_interval_us << ','
                    << row.deadline_lateness_us << ','
                    << (row.deadline_missed ? 1 : 0) << ','
                    << ToString(row.state) << ',' << row.robot_sequence << ','
                    << row.sensor_sequence
                    << ',' << row.robot_received_monotonic_ns
                    << ',' << row.sensor_received_monotonic_ns
                    << ',' << row.sensor_source_unix_ns
                    << ',' << (row.robot_valid ? 1 : 0)
                    << ',' << (row.wrench_valid ? 1 : 0)
                    << ',' << row.arm_error << ',' << row.system_error
                    << ',' << row.robot_age_ms << ',' << row.sensor_age_ms
                    << ',' << (row.checksum_valid ? 1 : 0)
                    << ',' << ForceSensorIoStatusName(row.sensor_io_status)
                    << ',' << row.sensor_io_error
                    << ',' << (row.redis_enabled ? 1 : 0)
                    << ',' << (row.redis_subscriber_connected ? 1 : 0)
                    << ',' << (row.redis_publisher_connected ? 1 : 0)
                    << ',' << (row.command_present ? 1 : 0)
                    << ',' << (row.command_valid ? 1 : 0)
                    << ',' << (row.command_fresh ? 1 : 0)
                    << ',' << row.command_sequence
                    << ',' << row.command_producer_sequence
                    << ',' << row.command_connection_generation
                    << ',' << row.command_age_ms
                    << ',' << (row.command_action_enabled ? 1 : 0)
                    << ',' << (row.command_terminate ? 1 : 0)
                    << ',' << row.command_phase_index
                    << ',' << row.command_model_y_m
                    << ',' << row.command_model_rz_deg
                    << ',' << row.command_desired_force_n;
            std::string escaped_command_hold_reason = row.command_hold_reason;
            std::replace(escaped_command_hold_reason.begin(),
                         escaped_command_hold_reason.end(), ',', ';');
            stream_ << ',' << escaped_command_hold_reason
                    << ',' << row.servo_submitted_sequence
                    << ',' << row.servo_consumed_sequence
                    << ',' << row.servo_discarded_sequence
                    << ',' << row.servo_pending_sequence
                    << ',' << row.servo_sent_sequence
                    << ',' << row.servo_result_sequence
                    << ',' << static_cast<int>(row.servo_result_code)
                    << ',' << (row.servo_outstanding ? 1 : 0);
            for (int i = 0; i < 7; ++i) stream_ << ',' << row.actual_joints[i];
            for (int i = 0; i < 6; ++i) stream_ << ',' << row.actual_pose[i];
            for (int i = 0; i < 6; ++i) stream_ << ',' << row.raw_wrench[i];
            for (int i = 0; i < 6; ++i) stream_ << ',' << row.compensated_wrench[i];
            for (int i = 0; i < 6; ++i) stream_ << ',' << row.control_wrench[i];
            stream_ << ',' << (row.contact.valid ? 1 : 0)
                    << ',' << row.contact.point.x()
                    << ',' << row.contact.point.y()
                    << ',' << row.contact.point.z()
                    << ',' << row.contact.residual
                    << ',' << row.contact.point_error_m
                    << ',' << row.filtered_contact_point.x()
                    << ',' << row.filtered_contact_point.y()
                    << ',' << row.filtered_contact_point.z();
            for (int i = 0; i < 6; ++i) stream_ << ',' << row.requested_delta[i];
            for (int i = 0; i < 6; ++i) stream_ << ',' << row.desired_pose[i];
            for (int i = 0; i < 7; ++i) stream_ << ',' << row.target_joints[i];
            std::string escaped_fault = row.fault;
            std::replace(escaped_fault.begin(), escaped_fault.end(), ',', ';');
            stream_ << ',' << escaped_fault << '\n';
        }
        stream_.flush();
    }

    static constexpr std::size_t kMaximumRows = 8192;
    std::string path_;
    std::ofstream stream_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<RuntimeLogRow> queue_;
    std::size_t queue_head_ = 0;
    std::size_t queue_size_ = 0;
    std::atomic<std::uint64_t> dropped_rows_{0};
};

std::string TimestampForFilename() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream output;
    output << std::put_time(&local, "%Y%m%d_%H%M%S");
    return output.str();
}

std::string DefaultRuntimeLogPath() {
    return (std::filesystem::current_path() / "logs"
            / ("main_rm75_" + TimestampForFilename())
            / "runtime.csv")
        .string();
}

std::filesystem::path ExecutableDirectory(const char* argv0) {
    std::error_code error;
    const std::filesystem::path proc_executable =
        std::filesystem::read_symlink("/proc/self/exe", error);
    if (!error && !proc_executable.empty()) {
        return proc_executable.parent_path();
    }
    error.clear();
    const std::filesystem::path absolute = std::filesystem::absolute(argv0, error);
    return error ? std::filesystem::current_path() : absolute.parent_path();
}

// The only implicit hardware profile. All explicit CLI invocations continue
// to use ParseOptions defaults and validation below.
void ApplyImplicitCommissioningProfile(const char* argv0, Options& options) {
    const std::filesystem::path executable_directory = ExecutableDirectory(argv0);
    options.mode = ControllerMode::kExecute;
    options.redis_enabled = false;
    options.robot_ip = "192.168.50.254";
    options.robot_port = 8080;
    options.sensor_device =
        "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0DU5LC-if00-port0";
    options.sensor_baud = 115200;
    options.calibration_path =
        (executable_directory / "rm75_force_calibration_v6_provisional.json").string();
    options.expected_sensor_id = "DU0DU5LC";
    options.probe_model_path =
        (executable_directory / "../model/Lprobe-IFS.STL").lexically_normal().string();
    options.duration_s = 60;
    options.tare_no_contact_s = 3;
    // First RM75 force-control baseline: reproduce the original six-axis
    // operating target before tuning new gains or force bands.
    options.desired_force_n = -3.0;
    options.raw_force_limit_n = 50.0;
    options.raw_torque_limit_nm = 5.0;
    options.approach_speed_cm_s = 0.05;
    options.approach_direction_tool_z = 1.0;
    // Preserve the RM75 commissioning speed envelope while restoring the
    // original six-axis force target and admittance parameters. The legacy
    // 3 mm/cycle no-contact step is intentionally not restored.
    options.maximum_linear_speed_cm_s = 0.05;
    options.maximum_approach_distance_mm = 250.0;
    // Zero disables the separate total Cartesian-orientation excursion gate.
    options.maximum_orientation_excursion_deg = 0.0;
    options.max_tracking_joint_error_deg = 5.0;
    options.max_tracking_position_error_mm = 10.0;
    // Zero disables Cartesian orientation tracking-error supervision for the
    // current force-contact commissioning profile. Joint and position
    // tracking supervision remain enabled.
    options.max_tracking_orientation_error_deg = 0.0;
    options.manual_action = true;
    options.allow_provisional_force_control = true;
    // Return the default commissioning path to the basic axial-force loop.
    // The legacy stage remains available through the explicit CLI option for
    // offline comparison, but its wrench/contact Kalman filters and contact
    // attitude admittance must not affect the default real-machine run.
    options.legacy_six_axis_force_stage = false;
    // Match the original six-axis contact logic: no separate overload-retreat
    // state. Forces above the -3 N target unload only through the same axial
    // admittance equation.
    options.enable_force_retract = false;
    options.runtime_log_path =
        (executable_directory / "logs"
         / ("rm75_v6_axial_force_3n_tcp188_60s_"
            + TimestampForFilename() + ".csv"))
            .string();
    options.implicit_commissioning_profile = true;
}

void Usage(const char* program) {
    std::cout
        << "Usage: " << program
        << " [--execute | --calibration FILE [options]]\n\n"
        << "Confirmed local commissioning shortcut:\n"
        << "  " << program << " --execute\n"
        << "    With no other arguments, load the bounded RM75/v6/DU0DU5LC profile.\n"
        << "    The local profile is limited to 60 s and 250 mm total TCP travel.\n"
        << "    Real motion remains authorized only by the explicit --execute token.\n\n"
        << "Safe modes (default is observe):\n"
        << "  --observe             acquire/publish only; never plan or send motion\n"
        << "  --dry-run-control     plan and log motion; never send ServoJ\n"
        << "  --execute             send planned ServoJ (explicit hardware action)\n"
        << "  --simulate            no hardware; implies dry-run and --no-redis\n\n"
        << "Hardware/configuration:\n"
        << "  --robot-ip IP --robot-port PORT\n"
        << "  --sensor-device PATH --sensor-baud BAUD --sensor-stale-ms MS\n"
        << "  --robot-stale-ms MS --command-stale-ms MS\n"
        << "  --calibration FILE --expected-sensor-id ID --probe-model FILE\n"
        << "  --redis-host HOST --redis-port PORT --no-redis\n"
        << "    local --no-redis execute requires --duration-sec 1..60\n"
        << "  --period-ms MS --duration-sec SEC --publish-every N\n"
        << "  --runtime-log FILE\n\n"
        << "Control (SI except names carrying cm/mm/deg):\n"
        << "  --desired-force-n N               default -1, allowed -3..-0.1\n"
        << "  --raw-force-limit-n N --raw-torque-limit-nm NM (cannot exceed 50/5)\n"
        << "  --execute-warmup-sec SEC          no-contact gate, default 2\n"
        << "  --tare-no-contact-sec SEC         explicit stationary tare, 1..10\n"
        << "  --allow-provisional-force-control explicit restricted commissioning\n"
        << "  --approach-speed-cm-s SPEED        default 0, maximum 0.5\n"
        << "  --approach-direction-tool-z -1|0|1\n"
        << "  --max-linear-speed-cm-s SPEED     maximum 0.5\n"
        << "  --max-approach-distance-mm MM      default/maximum 5\n"
        << "    bare --execute local profile uses a dedicated 250 mm ceiling\n"
        << "  --max-orientation-excursion-deg DEG default 5, 0 disables, maximum 15\n"
        << "  --max-tracking-joint-error-deg DEG --max-tracking-position-error-mm MM\n"
        << "  --max-tracking-orientation-error-deg DEG (0 disables)\n"
        << "  --manual-action --manual-terminate --manual-y-m M\n"
        << "  --manual-rz-deg DEG --manual-phase -1..2\n"
        << "  --contact-pitch-gain-rad-per-m G  default 0 (confirm sign in dry-run)\n"
        << "  --legacy-six-axis-force-stage     old wrench/contact filters and\n"
        << "                                    contact-roll dynamics\n"
        << "  --allow-near-singularity\n"
        << "  --enable-force-retract --retract-direction-tool-z -1|1\n"
        << "  --retract-distance-mm MM --retract-speed-cm-s SPEED\n";
}

bool ParseInt(const char* text, int& output) {
    try {
        std::size_t used = 0;
        output = std::stoi(text, &used, 10);
        return used == std::strlen(text);
    } catch (...) {
        return false;
    }
}

bool ParseUnsigned(const char* text, unsigned int& output) {
    try {
        std::size_t used = 0;
        const unsigned long value = std::stoul(text, &used, 10);
        if (used != std::strlen(text)
            || value > std::numeric_limits<unsigned int>::max()) return false;
        output = static_cast<unsigned int>(value);
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseDouble(const char* text, double& output) {
    try {
        std::size_t used = 0;
        output = std::stod(text, &used);
        return used == std::strlen(text) && std::isfinite(output);
    } catch (...) {
        return false;
    }
}

bool ParseOptions(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        auto value = [&]() -> const char* {
            return i + 1 < argc ? argv[++i] : nullptr;
        };
        if (argument == "--help" || argument == "-h") {
            Usage(argv[0]);
            std::exit(0);
        } else if (argument == "--observe") {
            options.mode = ControllerMode::kObserve;
        } else if (argument == "--dry-run-control") {
            options.mode = ControllerMode::kDryRun;
        } else if (argument == "--execute") {
            options.mode = ControllerMode::kExecute;
        } else if (argument == "--simulate") {
            options.simulate = true;
            options.mode = ControllerMode::kDryRun;
            options.redis_enabled = false;
        } else if (argument == "--no-redis") {
            options.redis_enabled = false;
        } else if (argument == "--robot-ip") {
            const char* item = value(); if (!item) return false; options.robot_ip = item;
        } else if (argument == "--robot-port") {
            const char* item = value(); if (!item || !ParseInt(item, options.robot_port)) return false;
        } else if (argument == "--sensor-device") {
            const char* item = value(); if (!item) return false; options.sensor_device = item;
        } else if (argument == "--sensor-baud") {
            const char* item = value(); if (!item || !ParseUnsigned(item, options.sensor_baud)) return false;
        } else if (argument == "--sensor-stale-ms") {
            const char* item = value(); if (!item || !ParseInt(item, options.sensor_stale_ms)) return false;
        } else if (argument == "--robot-stale-ms") {
            const char* item = value(); if (!item || !ParseInt(item, options.robot_stale_ms)) return false;
        } else if (argument == "--command-stale-ms") {
            const char* item = value(); if (!item || !ParseInt(item, options.command_stale_ms)) return false;
        } else if (argument == "--calibration") {
            const char* item = value(); if (!item) return false; options.calibration_path = item;
        } else if (argument == "--expected-sensor-id") {
            const char* item = value(); if (!item) return false; options.expected_sensor_id = item;
        } else if (argument == "--probe-model") {
            const char* item = value(); if (!item) return false; options.probe_model_path = item;
        } else if (argument == "--redis-host") {
            const char* item = value(); if (!item) return false; options.redis_host = item;
        } else if (argument == "--redis-port") {
            const char* item = value(); if (!item || !ParseInt(item, options.redis_port)) return false;
        } else if (argument == "--period-ms") {
            const char* item = value(); if (!item || !ParseInt(item, options.period_ms)) return false;
        } else if (argument == "--duration-sec") {
            const char* item = value(); if (!item || !ParseInt(item, options.duration_s)) return false;
        } else if (argument == "--publish-every") {
            const char* item = value(); if (!item || !ParseInt(item, options.publish_every)) return false;
        } else if (argument == "--runtime-log") {
            const char* item = value(); if (!item) return false; options.runtime_log_path = item;
        } else if (argument == "--execute-warmup-sec") {
            const char* item = value(); if (!item || !ParseInt(item, options.execute_warmup_s)) return false;
        } else if (argument == "--tare-no-contact-sec") {
            const char* item = value(); if (!item || !ParseInt(item, options.tare_no_contact_s)) return false;
        } else if (argument == "--allow-provisional-force-control") {
            options.allow_provisional_force_control = true;
        } else if (argument == "--legacy-six-axis-force-stage") {
            options.legacy_six_axis_force_stage = true;
        } else if (argument == "--desired-force-n") {
            const char* item = value(); if (!item || !ParseDouble(item, options.desired_force_n)) return false;
        } else if (argument == "--raw-force-limit-n") {
            const char* item = value(); if (!item || !ParseDouble(item, options.raw_force_limit_n)) return false;
        } else if (argument == "--raw-torque-limit-nm") {
            const char* item = value(); if (!item || !ParseDouble(item, options.raw_torque_limit_nm)) return false;
        } else if (argument == "--approach-speed-cm-s") {
            const char* item = value(); if (!item || !ParseDouble(item, options.approach_speed_cm_s)) return false;
        } else if (argument == "--approach-direction-tool-z") {
            const char* item = value(); if (!item || !ParseDouble(item, options.approach_direction_tool_z)) return false;
        } else if (argument == "--max-linear-speed-cm-s") {
            const char* item = value(); if (!item || !ParseDouble(item, options.maximum_linear_speed_cm_s)) return false;
        } else if (argument == "--max-approach-distance-mm") {
            const char* item = value(); if (!item || !ParseDouble(item, options.maximum_approach_distance_mm)) return false;
        } else if (argument == "--max-orientation-excursion-deg") {
            const char* item = value(); if (!item || !ParseDouble(item, options.maximum_orientation_excursion_deg)) return false;
        } else if (argument == "--max-tracking-joint-error-deg") {
            const char* item = value(); if (!item || !ParseDouble(item, options.max_tracking_joint_error_deg)) return false;
        } else if (argument == "--max-tracking-position-error-mm") {
            const char* item = value(); if (!item || !ParseDouble(item, options.max_tracking_position_error_mm)) return false;
        } else if (argument == "--max-tracking-orientation-error-deg") {
            const char* item = value(); if (!item || !ParseDouble(item, options.max_tracking_orientation_error_deg)) return false;
        } else if (argument == "--manual-y-m") {
            const char* item = value(); if (!item || !ParseDouble(item, options.manual_y_m)) return false;
        } else if (argument == "--manual-rz-deg") {
            const char* item = value(); if (!item || !ParseDouble(item, options.manual_rz_deg)) return false;
        } else if (argument == "--contact-pitch-gain-rad-per-m") {
            const char* item = value(); if (!item || !ParseDouble(item, options.contact_pitch_gain_rad_per_m)) return false;
        } else if (argument == "--manual-phase") {
            const char* item = value(); if (!item || !ParseInt(item, options.manual_phase)) return false;
        } else if (argument == "--manual-action") {
            options.manual_action = true;
        } else if (argument == "--manual-terminate") {
            options.manual_terminate = true;
        } else if (argument == "--allow-near-singularity") {
            options.allow_near_singularity = true;
        } else if (argument == "--enable-force-retract") {
            options.enable_force_retract = true;
        } else if (argument == "--retract-direction-tool-z") {
            const char* item = value(); if (!item || !ParseDouble(item, options.retract_direction_tool_z)) return false;
        } else if (argument == "--retract-distance-mm") {
            const char* item = value(); if (!item || !ParseDouble(item, options.retract_distance_mm)) return false;
        } else if (argument == "--retract-speed-cm-s") {
            const char* item = value(); if (!item || !ParseDouble(item, options.retract_speed_cm_s)) return false;
        } else {
            std::cerr << "Unknown option: " << argument << '\n';
            return false;
        }
    }

    const auto direction_valid = [](double value, bool allow_zero) {
        return (allow_zero && value == 0.0) || value == -1.0 || value == 1.0;
    };
    if (options.calibration_path.empty()
        || options.robot_port <= 0 || options.robot_port > 65535
        || options.redis_port <= 0 || options.redis_port > 65535
        || options.period_ms < 10 || options.period_ms > 100
        || options.sensor_stale_ms < options.period_ms || options.sensor_stale_ms > 1000
        || options.robot_stale_ms < options.period_ms || options.robot_stale_ms > 2000
        || options.command_stale_ms < options.period_ms || options.command_stale_ms > 10000
        || options.duration_s < 0 || options.publish_every <= 0
        || options.execute_warmup_s < 2 || options.execute_warmup_s > 10
        || options.tare_no_contact_s < 0 || options.tare_no_contact_s > 10
        || options.desired_force_n < -3.0 || options.desired_force_n > -0.1
        || options.raw_force_limit_n <= 0.0 || options.raw_force_limit_n > 50.0
        || options.raw_torque_limit_nm <= 0.0
        || options.raw_torque_limit_nm > 5.0
        || options.approach_speed_cm_s < 0.0 || options.approach_speed_cm_s > 0.5
        || options.maximum_linear_speed_cm_s <= 0.0
        || options.maximum_linear_speed_cm_s > 0.5
        || options.maximum_approach_distance_mm <= 0.0
        || options.maximum_approach_distance_mm
               > (options.implicit_commissioning_profile ? 250.0 : 5.0)
        || options.maximum_orientation_excursion_deg < 0.0
        || options.maximum_orientation_excursion_deg > 15.0
        || options.max_tracking_joint_error_deg <= 0.0
        || options.max_tracking_joint_error_deg > 20.0
        || options.max_tracking_position_error_mm <= 0.0
        || options.max_tracking_position_error_mm > 20.0
        || options.max_tracking_orientation_error_deg < 0.0
        || options.max_tracking_orientation_error_deg > 10.0
        || !direction_valid(options.approach_direction_tool_z, true)
        || options.manual_y_m < -0.2 || options.manual_y_m > 0.2
        || options.manual_rz_deg < -180.0 || options.manual_rz_deg > 180.0
        || std::abs(options.contact_pitch_gain_rad_per_m) > 1000.0
        || options.manual_phase < -1 || options.manual_phase > 2
        || options.retract_distance_mm <= 0.0
        || options.retract_distance_mm
               > (options.implicit_commissioning_profile ? 200.0 : 5.0)
        || options.retract_speed_cm_s <= 0.0 || options.retract_speed_cm_s > 0.5
        || (options.enable_force_retract
            && !direction_valid(options.retract_direction_tool_z, false))) {
        return false;
    }
    if (options.mode == ControllerMode::kExecute && options.simulate) return false;
    if (options.mode == ControllerMode::kExecute && options.period_ms != 20) {
        return false;
    }
    return true;
}

Eigen::Matrix<double, 6, 1> ArrayToEigen(const std::array<double, 6>& input) {
    Eigen::Matrix<double, 6, 1> output;
    for (int i = 0; i < 6; ++i) output[i] = input[static_cast<std::size_t>(i)];
    return output;
}

bool RawWrenchWithinLimits(const Eigen::Matrix<double, 6, 1>& wrench,
                           double force_limit_n,
                           double torque_limit_nm) {
    return wrench.array().isFinite().all()
        && wrench.head<3>().cwiseAbs().maxCoeff() <= force_limit_n
        && wrench.tail<3>().cwiseAbs().maxCoeff() <= torque_limit_nm;
}

std::int64_t MonotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::int64_t MonotonicNs(std::chrono::steady_clock::time_point time) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               time.time_since_epoch())
        .count();
}

std::int64_t SystemNs(std::chrono::system_clock::time_point time) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               time.time_since_epoch())
        .count();
}

double Percentile99(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t index = std::min(
        values.size() - 1,
        static_cast<std::size_t>(std::ceil(values.size() * 0.99) - 1));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

// -------------------------------------------------------------------------
// Robot stop/stationary and no-contact tare gates
// -------------------------------------------------------------------------

RMResult RequestConfirmedStop(RMCommand& command) {
    RMResult result = RMResult::Failure(
        RMErrorCode::kTimeout, "StopMotion was not attempted");
    for (int attempt = 0; attempt < 2; ++attempt) {
        result = command.TryStopMotion(1000);
        if (result) return result;
        if (result.code == RMErrorCode::kTimeout) break;
        if (attempt == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return result;
}

class BestEffortStopGuard {
public:
    void Arm(RMCommand& command) noexcept {
        command_ = &command;
        armed_ = true;
    }

    void Disarm() noexcept { armed_ = false; }

    ~BestEffortStopGuard() {
        if (!armed_ || command_ == nullptr || !command_->IsConnected()) return;
        try {
            (void)command_->TryStopMotion(500);
        } catch (...) {
            // Destructors must remain noexcept. A failed best-effort Stop is
            // still protected by the transport's latched Servo gate.
        }
    }

private:
    RMCommand* command_ = nullptr;
    bool armed_ = false;
};

double WrappedAngleDifference(double lhs, double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

bool ServoOutstanding(const ServoSendSnapshot& status) {
    return status.submitted_sequence
        > std::max(status.sent_sequence, status.discarded_sequence);
}

double MaximumWrappedJointDeltaDeg(
    const Eigen::Matrix<double, 7, 1>& lhs,
    const Eigen::Matrix<double, 7, 1>& rhs) {
    double maximum = 0.0;
    for (int joint = 0; joint < 7; ++joint) {
        maximum = std::max(
            maximum,
            std::abs(WrappedAngleDifference(lhs[joint], rhs[joint]))
                * 180.0 / M_PI);
    }
    return maximum;
}

Eigen::Vector3d ProbeTcpBase(
    const Eigen::Matrix<double, 6, 1>& controller_pose,
    const Eigen::Vector3d& probe_tcp_tool_m) {
    return controller_pose.head<3>()
        + RotationBaseFromControllerEuler(controller_pose.tail<3>())
            * probe_tcp_tool_m;
}

RMResult StopAndConfirmStationary(
    RMCommand& command,
    RMStateReader& state_reader,
    const Eigen::Vector3d& probe_tcp_tool_m,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) {
    const RMResult stop_result = RequestConfirmedStop(command);
    if (!stop_result) return stop_result;

    RobotStateSnapshot previous = state_reader.Latest();
    if (!previous.valid || previous.stale) {
        return RMResult::Failure(
            RMErrorCode::kProtocol,
            "StopMotion was acknowledged but robot feedback is invalid or stale");
    }
    RobotStateSnapshot window_anchor = previous;
    int stationary_updates = 0;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        RobotStateSnapshot current;
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (!state_reader.WaitForUpdate(
                previous.sequence,
                std::min(std::chrono::milliseconds(250),
                         std::max(std::chrono::milliseconds(1), remaining)),
                current)) {
            if (!state_reader.running()) break;
            continue;
        }
        if (!current.valid || current.stale
            || current.arm_err != 0 || current.sys_err != 0) {
            return RMResult::Failure(
                RMErrorCode::kProtocol,
                "StopMotion was acknowledged but healthy feedback was lost");
        }

        const double adjacent_joint_deg =
            MaximumWrappedJointDeltaDeg(current.joints, previous.joints);
        const double window_joint_deg =
            MaximumWrappedJointDeltaDeg(current.joints, window_anchor.joints);
        const double adjacent_tcp_mm =
            (ProbeTcpBase(current.pose, probe_tcp_tool_m)
             - ProbeTcpBase(previous.pose, probe_tcp_tool_m)).norm()
            * 1000.0;
        const double window_tcp_mm =
            (ProbeTcpBase(current.pose, probe_tcp_tool_m)
             - ProbeTcpBase(window_anchor.pose, probe_tcp_tool_m)).norm()
            * 1000.0;
        const Eigen::Matrix3d adjacent_rotation =
            RotationBaseFromControllerEuler(current.pose.tail<3>())
            * RotationBaseFromControllerEuler(previous.pose.tail<3>()).transpose();
        const Eigen::Matrix3d window_rotation =
            RotationBaseFromControllerEuler(current.pose.tail<3>())
            * RotationBaseFromControllerEuler(window_anchor.pose.tail<3>()).transpose();
        const double adjacent_orientation_deg =
            Eigen::AngleAxisd(adjacent_rotation).angle() * 180.0 / M_PI;
        const double window_orientation_deg =
            Eigen::AngleAxisd(window_rotation).angle() * 180.0 / M_PI;
        const bool stationary = adjacent_joint_deg <= 0.01
            && window_joint_deg <= 0.02
            && adjacent_tcp_mm <= 0.05
            && window_tcp_mm <= 0.10
            && adjacent_orientation_deg <= 0.01
            && window_orientation_deg <= 0.02;
        if (stationary) {
            ++stationary_updates;
            if (stationary_updates >= 5) return RMResult::Success();
        } else {
            stationary_updates = 0;
            window_anchor = current;
        }
        previous = current;
    }
    return RMResult::Failure(
        RMErrorCode::kTimeout,
        "StopMotion was acknowledged but five stationary feedback updates "
        "were not observed within the timeout");
}

struct RuntimeTareResult {
    bool valid = false;
    std::size_t samples = 0;
    Eigen::Matrix<double, 6, 1> sensor_offset =
        Eigen::Matrix<double, 6, 1>::Zero();
    double maximum_force_norm_n = 0.0;
    double maximum_torque_norm_nm = 0.0;
    double maximum_force_deviation_n = 0.0;
    double maximum_torque_deviation_nm = 0.0;
    double maximum_joint_span_deg = 0.0;
    double maximum_tcp_span_mm = 0.0;
    double maximum_orientation_span_deg = 0.0;
    std::string error;
};

RuntimeTareResult CollectRuntimeTare(
    ForceSensorReader& force_reader,
    RMStateReader& state_reader,
    const ForceCalibration& calibration,
    const Eigen::Vector3d& probe_tcp_tool_m,
    int seconds,
    double raw_force_limit_n,
    double raw_torque_limit_nm) {
    RuntimeTareResult result;
    std::vector<Eigen::Matrix<double, 6, 1>> samples;
    samples.reserve(static_cast<std::size_t>(seconds * 50));
    std::uint64_t last_sensor_sequence = 0;
    RobotStateSnapshot anchor;
    bool have_anchor = false;
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(seconds);

    while (!g_stop_requested.load()
           && std::chrono::steady_clock::now() < deadline) {
        const RobotStateSnapshot robot = state_reader.Latest();
        const WrenchSample sample = force_reader.LatestSample();
        const RMResult reader_result = state_reader.LastResult();
        if (!robot.valid || robot.stale || !reader_result
            || robot.arm_err != 0 || robot.sys_err != 0) {
            result.error = "runtime tare lost healthy robot feedback";
            return result;
        }
        if (!sample.valid || sample.stale || !sample.checksum_valid
            || sample.io_status != ForceSensorIoStatus::kStreaming
            || sample.io_error != 0) {
            result.error = "runtime tare lost healthy force-sensor data";
            return result;
        }
        if (sample.sequence == 0 || sample.sequence == last_sensor_sequence) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        last_sensor_sequence = sample.sequence;
        const Eigen::Matrix<double, 6, 1> raw =
            ArrayToEigen(sample.wrench_si);
        if (!RawWrenchWithinLimits(raw,
                                   raw_force_limit_n,
                                   raw_torque_limit_nm)) {
            result.error = "runtime tare rejected raw sensor overrange";
            return result;
        }
        const Eigen::Matrix3d rotation_base_from_tool =
            RotationBaseFromControllerEuler(robot.pose.tail<3>());
        const Eigen::Matrix3d rotation_base_from_sensor =
            rotation_base_from_tool * calibration.rotation_tool_from_sensor;
        const CompensatedWrench compensated =
            calibration.Compensate(raw, rotation_base_from_sensor);
        if (!compensated.valid) {
            result.error = "runtime tare compensation failed: "
                + compensated.error;
            return result;
        }
        if (!have_anchor) {
            anchor = robot;
            have_anchor = true;
        }
        result.maximum_joint_span_deg = std::max(
            result.maximum_joint_span_deg,
            MaximumWrappedJointDeltaDeg(robot.joints, anchor.joints));
        result.maximum_tcp_span_mm = std::max(
            result.maximum_tcp_span_mm,
            (ProbeTcpBase(robot.pose, probe_tcp_tool_m)
             - ProbeTcpBase(anchor.pose, probe_tcp_tool_m)).norm() * 1000.0);
        const Eigen::Matrix3d orientation_delta =
            rotation_base_from_tool
            * RotationBaseFromControllerEuler(anchor.pose.tail<3>()).transpose();
        result.maximum_orientation_span_deg = std::max(
            result.maximum_orientation_span_deg,
            Eigen::AngleAxisd(orientation_delta).angle() * 180.0 / M_PI);
        result.maximum_force_norm_n = std::max(
            result.maximum_force_norm_n,
            compensated.sensor.head<3>().norm());
        result.maximum_torque_norm_nm = std::max(
            result.maximum_torque_norm_nm,
            compensated.sensor.tail<3>().norm());
        samples.push_back(compensated.sensor);
    }

    result.samples = samples.size();
    const std::size_t minimum_samples =
        static_cast<std::size_t>(seconds * 20);
    if (g_stop_requested.load()) {
        result.error = "runtime tare interrupted";
        return result;
    }
    if (samples.size() < minimum_samples) {
        result.error = "runtime tare collected too few fresh samples";
        return result;
    }
    for (const auto& sample : samples) result.sensor_offset += sample;
    result.sensor_offset /= static_cast<double>(samples.size());
    for (const auto& sample : samples) {
        result.maximum_force_deviation_n = std::max(
            result.maximum_force_deviation_n,
            (sample.head<3>() - result.sensor_offset.head<3>()).norm());
        result.maximum_torque_deviation_nm = std::max(
            result.maximum_torque_deviation_nm,
            (sample.tail<3>() - result.sensor_offset.tail<3>()).norm());
    }
    if (result.maximum_joint_span_deg > kRuntimeTareMaximumJointSpanDeg
        || result.maximum_tcp_span_mm > kRuntimeTareMaximumTcpSpanMm
        || result.maximum_orientation_span_deg
               > kRuntimeTareMaximumOrientationSpanDeg) {
        std::ostringstream message;
        message << "runtime tare requires a stationary robot"
                << " (joint_span_deg=" << result.maximum_joint_span_deg
                << ", limit=" << kRuntimeTareMaximumJointSpanDeg
                << "; tcp_span_mm=" << result.maximum_tcp_span_mm
                << ", limit=" << kRuntimeTareMaximumTcpSpanMm
                << "; orientation_span_deg="
                << result.maximum_orientation_span_deg
                << ", limit=" << kRuntimeTareMaximumOrientationSpanDeg
                << ')';
        result.error = message.str();
        return result;
    }
    if (result.maximum_force_deviation_n > 0.25
        || result.maximum_torque_deviation_nm > 0.02) {
        std::ostringstream message;
        message << "runtime tare residual wrench is not stable"
                << " (force_deviation_n="
                << result.maximum_force_deviation_n
                << ", limit=0.25; torque_deviation_nm="
                << result.maximum_torque_deviation_nm
                << ", limit=0.02)";
        result.error = message.str();
        return result;
    }
    result.valid = true;
    return result;
}

void ApplyRuntimeTare(const ForceCalibration& calibration,
                      const Eigen::Matrix<double, 6, 1>& sensor_offset,
                      CompensatedWrench& compensated) {
    if (!compensated.valid) return;
    compensated.sensor -= sensor_offset;
    const Eigen::Vector3d force_sensor = compensated.sensor.head<3>();
    const Eigen::Vector3d torque_at_tool_sensor =
        compensated.sensor.tail<3>()
        - calibration.translation_sensor_to_tool_m.cross(force_sensor);
    compensated.tool.head<3>() =
        calibration.rotation_tool_from_sensor * force_sensor;
    compensated.tool.tail<3>() =
        calibration.rotation_tool_from_sensor * torque_at_tool_sensor;
}

}  // namespace

// -------------------------------------------------------------------------
// Process orchestration: validate -> start I/O -> control loop -> stop/report
// -------------------------------------------------------------------------

int main(int argc, char** argv) {
    Options options;
    if (argc == 2 && std::string(argv[1]) == "--execute") {
        ApplyImplicitCommissioningProfile(argv[0], options);
    }
    if (!ParseOptions(argc, argv, options)) {
        Usage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);
    const bool provisional_execute =
        options.mode == ControllerMode::kExecute
        && options.allow_provisional_force_control;
    if (options.allow_provisional_force_control
        && options.mode != ControllerMode::kExecute) {
        std::cerr << "Provisional force-control override is execute-only\n";
        return 3;
    }
    if (options.tare_no_contact_s > 0
        && (options.simulate || options.redis_enabled)) {
        std::cerr << "Runtime tare requires real hardware with --no-redis\n";
        return 3;
    }
    if (options.mode == ControllerMode::kExecute
        && !provisional_execute && options.tare_no_contact_s > 0) {
        std::cerr << "Runtime tare in execute requires the explicit provisional "
                     "commissioning override\n";
        return 3;
    }
    if (provisional_execute) {
        const int maximum_commissioning_duration_s =
            options.implicit_commissioning_profile ? 60 : 5;
        const double maximum_commissioning_distance_mm =
            options.implicit_commissioning_profile ? 250.0 : 1.0;
        const bool constrained = !options.redis_enabled
            && options.duration_s >= 1
            && options.duration_s <= maximum_commissioning_duration_s
            && options.tare_no_contact_s >= 3
            && options.tare_no_contact_s <= 10
            && std::abs(options.desired_force_n + 3.0) <= 1e-12
            && options.approach_speed_cm_s > 0.0
            && options.approach_speed_cm_s <= 0.05
            && options.approach_direction_tool_z != 0.0
            && options.maximum_linear_speed_cm_s <= 0.5
            && options.maximum_approach_distance_mm
                   <= maximum_commissioning_distance_mm
            && options.maximum_orientation_excursion_deg <= 1.0
            && options.manual_action && !options.manual_terminate
            && options.manual_phase == -1
            && options.manual_y_m == 0.0
            && options.manual_rz_deg == 0.0
            && options.contact_pitch_gain_rad_per_m == 0.0
            && !options.allow_near_singularity
            && !options.enable_force_retract;
        if (!constrained) {
            std::cerr
                << "Provisional execute rejected: commissioning requires "
                   "--no-redis, duration 1.."
                << maximum_commissioning_duration_s
                << " s, tare >=3 s, -3 N, speed <=0.05 cm/s, "
                   "max linear speed <=0.5 cm/s, max distance <="
                << maximum_commissioning_distance_mm
                << " mm, no Y/RZ/pitch or independent retract, "
                   "and --manual-action\n";
            return 3;
        }
        if (kProvisionalCommissioningLocked) {
            std::cerr
                << "Provisional execute locked: repeated commissioning runs "
                   "exceeded the safe axial force envelope. Validate contact "
                   "hysteresis, overshoot stopping, and unloading behavior "
                   "before re-enabling hardware motion.\n";
            return 3;
        }
    }
    if (options.mode == ControllerMode::kExecute
        && !options.redis_enabled
        && (options.duration_s <= 0 || options.duration_s > 60)) {
        std::cerr << "Execute rejected: --no-redis requires a bounded "
                     "--duration-sec in 1..60\n";
        return 3;
    }
    if (options.mode == ControllerMode::kExecute
        && options.redis_enabled
        && (options.manual_action || options.manual_terminate
            || options.manual_y_m != 0.0 || options.manual_rz_deg != 0.0
            || options.manual_phase != -1)) {
        std::cerr << "Execute rejected: manual intent flags require "
                     "--no-redis and a bounded duration\n";
        return 3;
    }
    if (options.mode == ControllerMode::kExecute
        && options.approach_speed_cm_s > 0.0
        && options.approach_direction_tool_z == 0.0) {
        std::cerr << "Execute rejected: non-zero approach speed requires an explicit direction\n";
        return 3;
    }
    if (options.mode == ControllerMode::kExecute
        && options.approach_speed_cm_s > 0.0
        && options.approach_direction_tool_z * options.desired_force_n >= 0.0) {
        std::cerr << "Execute rejected: approach direction is inconsistent with the configured force sign\n";
        return 3;
    }
    if (options.runtime_log_path.empty()) {
        options.runtime_log_path = DefaultRuntimeLogPath();
    }

    ForceCalibration calibration;
    std::string error;
    if (!calibration.LoadJson(options.calibration_path, &error)) {
        std::cerr << "Calibration rejected: " << error << '\n';
        return 3;
    }
    if (!options.expected_sensor_id.empty()
        && calibration.sensor_id != options.expected_sensor_id) {
        std::cerr << "Calibration sensor_id does not match --expected-sensor-id\n";
        return 3;
    }
    std::string selected_probe_sha256;
    if (!ComputeFileSha256(options.probe_model_path,
                           selected_probe_sha256,
                           &error)) {
        std::cerr << "Probe model digest failed: " << error << '\n';
        return 3;
    }
    std::string calibration_probe_sha256 = calibration.probe_model_sha256;
    std::transform(calibration_probe_sha256.begin(),
                   calibration_probe_sha256.end(),
                   calibration_probe_sha256.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    if (!calibration_probe_sha256.empty()
        && calibration_probe_sha256 != selected_probe_sha256) {
        std::cerr << "Calibration probe_model_sha256 does not match selected STL\n";
        return 3;
    }
    if (options.mode == ControllerMode::kExecute) {
        if (options.expected_sensor_id.empty()) {
            std::cerr << "Execute rejected: --expected-sensor-id is required\n";
            return 3;
        }
        if (!calibration.tool_chain_verified
            || !calibration.calibration_residuals_verified
            || calibration_probe_sha256.empty()) {
            if (!provisional_execute || calibration_probe_sha256.empty()) {
                std::cerr << "Execute rejected: verified residuals, R/t/TCP and "
                             "probe SHA-256 are required\n";
                return 3;
            }
            std::cerr << "WARNING: provisional commissioning uses an "
                         "unverified calibration with runtime no-contact tare.\n";
        }
        if (options.allow_near_singularity) {
            std::cerr << "Execute rejected: --allow-near-singularity is dry-run only\n";
            return 3;
        }
    }
    const std::string model_filename =
        std::filesystem::path(options.probe_model_path).filename().string();
    if (calibration.probe_model != model_filename
        && calibration.probe_model != options.probe_model_path) {
        std::cerr << "Calibration probe_model does not match selected STL: "
                  << calibration.probe_model << " vs " << model_filename << '\n';
        return 3;
    }

    ContactLocation contact_locator;
    if (!contact_locator.LoadSTL(options.probe_model_path)) {
        std::cerr << "Probe model rejected: " << contact_locator.lastError() << '\n';
        return 3;
    }

    Rm75ControlConfig control_config;
    control_config.cycle_s = options.period_ms / 1000.0;
    control_config.desired_force_n = options.desired_force_n;
    control_config.force_retract_enabled = options.enable_force_retract;
    // Keep the generic overload retreat usable when an explicit caller raises
    // the target magnitude from the default 1 N. The release point must stay
    // above the target so it cannot chatter with normal admittance control.
    if (control_config.force_retract_enabled) {
        control_config.force_retract_release_z_n = std::max(
            control_config.force_retract_release_z_n,
            std::abs(options.desired_force_n) + 0.2);
    }
    if (options.legacy_six_axis_force_stage) {
        control_config.contact_threshold_n = 0.99;
        control_config.force_virtual_mass = 3.0;
        control_config.force_virtual_damping = 20.0;
        control_config.legacy_wrench_filter_enabled = true;
        control_config.legacy_contact_filter_enabled = true;
        control_config.legacy_contact_roll_enabled = true;
        if (options.implicit_commissioning_profile) {
            control_config.legacy_contact_roll_limits_enabled = false;
        }
    }
    if (provisional_execute) {
        // Reproduce the original six-axis contact behavior: contact at
        // 0.99 N, M=3, D=20 and a -3 N target. There is no separate retreat
        // state; force above target reverses through the same admittance law.
        control_config.contact_threshold_n = 0.99;
        control_config.force_virtual_mass = 3.0;
        control_config.force_virtual_damping = 20.0;
        control_config.force_retract_enabled = false;
        control_config.force_limit_z_n = 50.0;
        control_config.force_limit_xy_n = 20.0;
        control_config.torque_limit_nm = 5.0;
    }
    control_config.approach_speed_m_s = options.approach_speed_cm_s / 100.0;
    control_config.approach_direction_tool_z = options.approach_direction_tool_z;
    control_config.max_linear_speed_m_s = options.maximum_linear_speed_cm_s / 100.0;
    control_config.contact_pitch_gain_rad_per_m =
        options.contact_pitch_gain_rad_per_m;
    control_config.probe_tcp_tool_m = calibration.rotation_tool_from_sensor
        * (calibration.probe_tcp_sensor_m
           - calibration.translation_sensor_to_tool_m);
    Rm75ControlLaw control_law(control_config);

    Rm75ServoPlannerConfig planner_config;
    planner_config.period_ms = options.period_ms;
    planner_config.joint_speed_scale = std::min(
        0.5, planner_config.minimum_dispatch_gap_ms / options.period_ms);
    planner_config.allow_near_singularity = options.allow_near_singularity;
    Rm75ServoPlanner planner(planner_config);

    RedisBridgeConfig redis_config;
    redis_config.enabled = options.redis_enabled;
    redis_config.host = options.redis_host;
    redis_config.port = options.redis_port;
    RedisBridge redis(redis_config);
    if (!redis.Start(&error)) {
        std::cerr << "Redis bridge configuration rejected: " << error << '\n';
        return 3;
    }

    AsyncRuntimeLogger logger;
    if (!logger.Start(options.runtime_log_path, &error)) {
        std::cerr << error << '\n';
        return 3;
    }

    ForceSensorConfig sensor_config;
    sensor_config.device = options.sensor_device;
    sensor_config.baud_rate = options.sensor_baud;
    sensor_config.stale_after = std::chrono::milliseconds(options.sensor_stale_ms);
    std::unique_ptr<ForceSensorReader> force_reader;
    if (!options.simulate) {
        force_reader = std::make_unique<ForceSensorReader>(sensor_config);
        if (!force_reader->Start()) {
            std::cerr << "Force sensor start failed: " << force_reader->LastError() << '\n';
            return 4;
        }
    }

    RMCommand command;
    std::unique_ptr<RMStateReader> state_reader;
    BestEffortStopGuard execute_stop_guard;
    RobotStateSnapshot robot_state;
    if (!options.simulate) {
        command.quiet = true;
        command.rlm_port = options.robot_port;
        std::strncpy(command.rlm_ip, options.robot_ip.c_str(), sizeof(command.rlm_ip) - 1);
        command.rlm_ip[sizeof(command.rlm_ip) - 1] = '\0';
        RMResult result = command.TryConnectTCPSocket();
        if (!result) {
            std::cerr << "Robot connection failed: " << result.message << '\n';
            return 4;
        }
        if (options.mode == ControllerMode::kExecute) {
            execute_stop_guard.Arm(command);
            result = RequestConfirmedStop(command);
            if (!result) {
                std::cerr << "Execute startup Stop failed before state read: "
                          << result.message << '\n';
                return 4;
            }
        }
        result = command.TryReadArmState(robot_state.joints,
                                         robot_state.pose,
                                         robot_state.arm_err,
                                         robot_state.sys_err,
                                         2000);
        if (!result || robot_state.arm_err != 0 || robot_state.sys_err != 0) {
            std::cerr << "Initial robot state failed: " << result.message << '\n';
            return 4;
        }
        robot_state = command.CachedRobotState(
            std::chrono::milliseconds(options.robot_stale_ms));
        state_reader = std::make_unique<RMStateReader>(
            command,
            std::chrono::milliseconds(40),
            std::chrono::milliseconds(options.robot_stale_ms));
        result = state_reader->Start();
        if (!result) {
            std::cerr << "Robot state reader failed: " << result.message << '\n';
            return 4;
        }
        RobotStateSnapshot first_async;
        if (!state_reader->WaitForUpdate(
                0, std::chrono::milliseconds(2000), first_async)) {
            std::cerr << "Robot state reader produced no fresh state within 2 seconds\n";
            return 4;
        }
        robot_state = first_async;
    } else {
        robot_state.joints << 0.4, 0.6, -0.5, 0.7, 0.4, 0.8, -0.2;
        robot_state.pose = planner.PoseFromJoints(robot_state.joints);
        robot_state.sequence = 1;
        robot_state.received_at = std::chrono::steady_clock::now();
        robot_state.valid = true;
        robot_state.stale = false;
    }

    RuntimeTareResult runtime_tare;
    bool runtime_tare_applied = false;
    if (options.mode == ControllerMode::kExecute) {
        std::cout << "Stopping inherited motion and confirming stationary feedback...\n";
        const RMResult stationary_result = StopAndConfirmStationary(
            command,
            *state_reader,
            control_config.probe_tcp_tool_m);
        if (!stationary_result) {
            std::cerr << "Execute startup stop/stationary gate failed: "
                      << stationary_result.message << '\n';
            return 4;
        }
        if (g_stop_requested.load()) {
            std::cerr << "Execute cancelled before force warm-up\n";
            return 130;
        }
        const int warmup_seconds = provisional_execute
            ? options.tare_no_contact_s
            : options.execute_warmup_s;
        std::cout << "Running mandatory no-contact force warm-up for "
                  << warmup_seconds << " seconds...\n";
        runtime_tare = CollectRuntimeTare(
            *force_reader,
            *state_reader,
            calibration,
            control_config.probe_tcp_tool_m,
            warmup_seconds,
            options.raw_force_limit_n,
            options.raw_torque_limit_nm);
        if (g_stop_requested.load()) {
            const RMResult stop_result = StopAndConfirmStationary(
                command,
                *state_reader,
                control_config.probe_tcp_tool_m);
            if (!stop_result) {
                std::cerr << "Stop after cancelled warm-up was not physically confirmed: "
                          << stop_result.message << '\n';
            } else {
                execute_stop_guard.Disarm();
            }
            return 130;
        }
        if (!runtime_tare.valid) {
            std::cerr << "Execute warm-up failed: " << runtime_tare.error
                      << " samples=" << runtime_tare.samples << '\n';
            return 5;
        }
        if (provisional_execute) {
            if (runtime_tare.maximum_force_norm_n > 3.0
                || runtime_tare.maximum_torque_norm_nm > 0.1) {
                std::cerr << "Provisional tare rejected: uncompensated residual "
                             "exceeds 3 N / 0.1 N*m\n";
                return 5;
            }
            runtime_tare_applied = true;
        } else if (runtime_tare.maximum_force_norm_n > 0.5
                   || runtime_tare.maximum_torque_norm_nm > 0.05) {
            std::cerr << "Execute warm-up failed: samples="
                      << runtime_tare.samples
                      << " force_max_n=" << runtime_tare.maximum_force_norm_n
                      << " torque_max_nm="
                      << runtime_tare.maximum_torque_norm_nm
                      << " (required <= 0.5 N / 0.05 N*m)\n";
            return 5;
        }
        robot_state = state_reader->Latest();
    } else if (options.tare_no_contact_s > 0) {
        std::cout << "Running explicit stationary no-contact tare for "
                  << options.tare_no_contact_s << " seconds...\n";
        runtime_tare = CollectRuntimeTare(
            *force_reader,
            *state_reader,
            calibration,
            control_config.probe_tcp_tool_m,
            options.tare_no_contact_s,
            options.raw_force_limit_n,
            options.raw_torque_limit_nm);
        if (!runtime_tare.valid) {
            std::cerr << "Runtime tare failed: " << runtime_tare.error
                      << " samples=" << runtime_tare.samples << '\n';
            return 5;
        }
        if (runtime_tare.maximum_force_norm_n > 3.0
            || runtime_tare.maximum_torque_norm_nm > 0.1) {
            std::cerr << "Runtime tare rejected residual above 3 N / 0.1 N*m\n";
            return 5;
        }
        runtime_tare_applied = true;
        robot_state = state_reader->Latest();
    }

    if (runtime_tare_applied) {
        std::cout << "runtime_tare_sensor=["
                  << runtime_tare.sensor_offset.transpose() << "]\n"
                  << "runtime_tare_samples=" << runtime_tare.samples << '\n'
                  << "runtime_tare_max_deviation="
                  << runtime_tare.maximum_force_deviation_n << " N / "
                  << runtime_tare.maximum_torque_deviation_nm << " N*m\n";
    }

    std::cout << "main_rm75 mode: " << ModeName(options.mode) << '\n'
              << "configuration_profile: "
              << (options.implicit_commissioning_profile
                      ? kImplicitProfileName
                      : "explicit_cli")
              << '\n'
              << "period_ms: " << options.period_ms << '\n'
              << "sensor_protocol: "
              << ForceSensorProtocolName(sensor_config.protocol) << '\n'
              << "sensor_stale_ms: " << options.sensor_stale_ms << '\n'
              << "robot_stale_ms: " << options.robot_stale_ms << '\n'
              << "duration_s: " << options.duration_s << '\n'
              << "desired_force_n: " << options.desired_force_n << '\n'
              << "approach_speed_cm_s: " << options.approach_speed_cm_s << '\n'
              << "independent_force_retract: "
              << (control_config.force_retract_enabled
                      ? "enabled" : "disabled")
              << '\n'
              << "contact_threshold_n: "
              << control_config.contact_threshold_n << '\n'
              << "force_virtual_mass: "
              << control_config.force_virtual_mass << '\n'
              << "force_virtual_damping: "
              << control_config.force_virtual_damping << '\n'
              << "legacy_six_axis_force_stage: "
              << (options.legacy_six_axis_force_stage ? "enabled" : "disabled")
              << '\n'
              << "legacy_wrench_kalman: "
              << (control_config.legacy_wrench_filter_enabled
                      ? "enabled" : "disabled")
              << '\n'
              << "legacy_contact_kalman: "
              << (control_config.legacy_contact_filter_enabled
                      ? "enabled" : "disabled")
              << '\n'
              << "legacy_contact_attitude_admittance: "
              << (control_config.legacy_contact_roll_enabled
                      ? "enabled" : "disabled")
              << '\n'
              << "maximum_approach_distance_mm: "
              << options.maximum_approach_distance_mm << '\n'
              << "maximum_orientation_excursion_deg: "
              << options.maximum_orientation_excursion_deg << '\n'
              << "probe_tcp_sensor_m: ["
              << calibration.probe_tcp_sensor_m.transpose() << "]\n"
              << "probe_tcp_tool_m: ["
              << control_config.probe_tcp_tool_m.transpose() << "]\n"
              << "maximum_linear_speed_cm_s: "
              << options.maximum_linear_speed_cm_s << '\n'
              << "compensated_wrench_limits_force_xy_z_n: ["
              << control_config.force_limit_xy_n << ' '
              << control_config.force_limit_z_n << "]\n"
              << "compensated_wrench_limit_torque_nm: "
              << control_config.torque_limit_nm << '\n'
              << "raw_wrench_limits_force_torque: ["
              << options.raw_force_limit_n << ' '
              << options.raw_torque_limit_nm << "]\n"
              << "max_tracking_joint_error_deg: "
              << options.max_tracking_joint_error_deg << '\n'
              << "max_tracking_position_error_mm: "
              << options.max_tracking_position_error_mm << '\n'
              << "max_tracking_orientation_error_deg: "
              << options.max_tracking_orientation_error_deg << '\n'
              << "runtime_log: " << options.runtime_log_path << '\n';
    if (control_config.force_retract_enabled) {
        std::cout << "retract_direction_tool_z: "
                  << options.retract_direction_tool_z << '\n'
                  << "retract_distance_mm: "
                  << options.retract_distance_mm << '\n'
                  << "retract_speed_cm_s: "
                  << options.retract_speed_cm_s << '\n';
    }
    if (options.mode != ControllerMode::kExecute) {
        std::cout << "No real motion command will be sent in this mode.\n";
    }

    redis.PublishStatus(Rm75SupervisorState::kInitializing,
                        "ready",
                        std::string("RM75 controller started in ") + ModeName(options.mode),
                        "",
                        0);

    Eigen::Matrix<double, 7, 1> model_joints = robot_state.joints;
    Eigen::Matrix<double, 6, 1> model_pose = robot_state.pose;
    Eigen::Matrix<double, 7, 1> previous_joint_delta =
        Eigen::Matrix<double, 7, 1>::Zero();
    const Eigen::Vector3d initial_probe_tcp =
        ProbeTcpBase(model_pose, control_config.probe_tcp_tool_m);
    const Eigen::Matrix3d initial_tool_rotation =
        RotationBaseFromControllerEuler(model_pose.tail<3>());
    const auto started_at = std::chrono::steady_clock::now();
    auto next_tick = started_at;
    std::uint64_t cycle = 0;
    std::uint64_t missed_periods = 0;
    std::uint64_t periods_over_22ms = 0;
    std::uint64_t periods_over_40ms = 0;
    int consecutive_missed_periods = 0;
    int maximum_consecutive_missed_periods = 0;
    int stale_wrench_cycles = 0;
    bool fatal_fault = false;
    std::string fatal_fault_code;
    Rm75SupervisorState previous_state = Rm75SupervisorState::kInitializing;
    std::string previous_reported_fault;
    std::vector<double> work_durations_us;
    constexpr std::size_t kMaximumTimingSamples = 30000;
    work_durations_us.reserve(kMaximumTimingSamples);
    bool retreat_active = false;
    bool legacy_completion_pending = false;
    double retreat_distance_m = 0.0;
    double retreat_initial_abs_fz_n = 0.0;
    double retreat_best_abs_fz_n = std::numeric_limits<double>::infinity();
    int retreat_cycles = 0;
    auto previous_cycle_started = started_at;
    ServoSendSnapshot servo_status;
    std::uint64_t last_servo_sent_sequence = 0;
    auto last_servo_progress_at = started_at;
    if (options.mode == ControllerMode::kExecute) {
        servo_status = command.ServoStatus();
        last_servo_sent_sequence = servo_status.sent_sequence;
    }

    while (!g_stop_requested.load()) {
        const auto cycle_started = std::chrono::steady_clock::now();
        if (options.duration_s > 0
            && cycle_started - started_at >= std::chrono::seconds(options.duration_s)) {
            break;
        }
        ++cycle;
        const double cycle_interval_us = cycle == 1
            ? 0.0
            : std::chrono::duration<double, std::micro>(
                  cycle_started - previous_cycle_started).count();
        previous_cycle_started = cycle_started;
        if (cycle > 1 && cycle_interval_us > 22000.0) ++periods_over_22ms;
        if (cycle > 1 && cycle_interval_us > 40000.0) ++periods_over_40ms;

        RMResult asynchronous_robot_result = RMResult::Success();
        if (!options.simulate) {
            robot_state = state_reader->Latest();
            asynchronous_robot_result = state_reader->LastResult();
        } else {
            robot_state.received_at = cycle_started;
            robot_state.sequence = cycle;
            robot_state.stale = false;
        }
        const bool robot_valid = robot_state.valid && !robot_state.stale
            && robot_state.arm_err == 0 && robot_state.sys_err == 0
            && static_cast<bool>(asynchronous_robot_result);
        if (!robot_valid && options.mode == ControllerMode::kExecute) {
            fatal_fault = true;
            fatal_fault_code = asynchronous_robot_result
                ? "robot_state_invalid_or_stale"
                : "robot_async_io_or_command_error";
        }
        if (robot_valid && options.mode == ControllerMode::kExecute) {
            double maximum_joint_error_deg = 0.0;
            double minimum_actual_joint_margin_deg =
                std::numeric_limits<double>::infinity();
            for (int joint = 0; joint < 7; ++joint) {
                maximum_joint_error_deg = std::max(
                    maximum_joint_error_deg,
                    std::abs(WrappedAngleDifference(robot_state.joints[joint],
                                                    model_joints[joint]))
                        * 180.0 / M_PI);
                const double lower_margin = robot_state.joints[joint]
                    - planner.Kinematics().joint_min[joint];
                const double upper_margin = planner.Kinematics().joint_max[joint]
                    - robot_state.joints[joint];
                minimum_actual_joint_margin_deg = std::min(
                    minimum_actual_joint_margin_deg,
                    std::min(lower_margin, upper_margin) * 180.0 / M_PI);
            }
            const Eigen::Matrix3d tracking_rotation_error =
                RotationBaseFromControllerEuler(robot_state.pose.tail<3>())
                * RotationBaseFromControllerEuler(model_pose.tail<3>()).transpose();
            const double orientation_error_deg =
                Eigen::AngleAxisd(tracking_rotation_error).angle()
                * 180.0 / M_PI;
            const double position_error_mm =
                (ProbeTcpBase(robot_state.pose, control_config.probe_tcp_tool_m)
                 - ProbeTcpBase(model_pose, control_config.probe_tcp_tool_m))
                    .norm()
                * 1000.0;
            const Eigen::Vector3d actual_probe_tcp =
                ProbeTcpBase(robot_state.pose,
                             control_config.probe_tcp_tool_m);
            const double actual_total_distance_mm =
                (actual_probe_tcp - initial_probe_tcp).norm() * 1000.0;
            const Eigen::Matrix3d actual_rotation =
                RotationBaseFromControllerEuler(robot_state.pose.tail<3>());
            const double actual_total_orientation_deg =
                Eigen::AngleAxisd(
                    actual_rotation * initial_tool_rotation.transpose()).angle()
                * 180.0 / M_PI;
            if (minimum_actual_joint_margin_deg
                    < planner.Config().joint_limit_stop_deg) {
                fatal_fault = true;
                fatal_fault_code = "actual_joint_limit_margin_exceeded";
            } else if (actual_total_distance_mm
                       > options.maximum_approach_distance_mm) {
                fatal_fault = true;
                fatal_fault_code = "actual_maximum_approach_distance_exceeded";
            } else if (options.maximum_orientation_excursion_deg > 0.0
                       && actual_total_orientation_deg
                              > options.maximum_orientation_excursion_deg) {
                fatal_fault = true;
                fatal_fault_code = "actual_maximum_orientation_excursion_exceeded";
            } else if (maximum_joint_error_deg
                       > options.max_tracking_joint_error_deg) {
                fatal_fault = true;
                fatal_fault_code = "robot_joint_tracking_error_exceeded";
            } else if (position_error_mm
                       > options.max_tracking_position_error_mm) {
                fatal_fault = true;
                fatal_fault_code = "robot_position_tracking_error_exceeded";
            } else if (options.max_tracking_orientation_error_deg > 0.0
                       && orientation_error_deg
                              > options.max_tracking_orientation_error_deg) {
                fatal_fault = true;
                fatal_fault_code = "robot_orientation_tracking_error_exceeded";
            }
        }

        WrenchSample force_sample;
        Eigen::Matrix<double, 6, 1> raw_wrench =
            Eigen::Matrix<double, 6, 1>::Zero();
        if (!options.simulate) {
            force_sample = force_reader->LatestSample();
            raw_wrench = ArrayToEigen(force_sample.wrench_si);
        } else {
            force_sample.valid = true;
            force_sample.stale = false;
            force_sample.checksum_valid = true;
            force_sample.sequence = cycle;
            force_sample.timestamp = std::chrono::system_clock::now();
            force_sample.monotonic_timestamp = cycle_started;
            force_sample.io_status = ForceSensorIoStatus::kStreaming;
            const Eigen::Matrix3d rotation_base_from_tool =
                RotationBaseFromControllerEuler(model_pose.tail<3>());
            const Eigen::Matrix3d rotation_base_from_sensor =
                rotation_base_from_tool * calibration.rotation_tool_from_sensor;
            const Eigen::Vector3d gravity_sensor =
                rotation_base_from_sensor.transpose() * calibration.gravity_base_n;
            raw_wrench.head<3>() = calibration.force_bias_n + gravity_sensor;
            raw_wrench.tail<3>() = calibration.torque_bias_nm
                + calibration.center_of_mass_sensor_m.cross(gravity_sensor);
            raw_wrench.z() -= 1.0;
        }

        const Eigen::Matrix3d rotation_base_from_tool =
            RotationBaseFromControllerEuler(robot_state.pose.tail<3>());
        const Eigen::Matrix3d rotation_base_from_sensor =
            rotation_base_from_tool * calibration.rotation_tool_from_sensor;
        CompensatedWrench compensated;
        const bool raw_wrench_in_range = RawWrenchWithinLimits(
            raw_wrench,
            options.raw_force_limit_n,
            options.raw_torque_limit_nm);
        if (force_sample.valid && !force_sample.stale
            && raw_wrench_in_range) {
            compensated = calibration.Compensate(raw_wrench,
                                                 rotation_base_from_sensor,
                                                 force_sample.sequence,
                                                 SystemNs(force_sample.timestamp));
            if (runtime_tare_applied) {
                ApplyRuntimeTare(calibration,
                                 runtime_tare.sensor_offset,
                                 compensated);
            }
        } else if (!raw_wrench_in_range) {
            compensated.error = "force_sensor_raw_overrange";
        } else {
            compensated.error = "force_sensor_invalid_or_stale";
        }
        const bool wrench_valid = compensated.valid;
        if (options.mode == ControllerMode::kExecute
            && force_sample.valid && !force_sample.stale
            && !raw_wrench_in_range) {
            fatal_fault = true;
            fatal_fault_code = "force_sensor_raw_overrange";
        }
        stale_wrench_cycles = wrench_valid ? 0 : stale_wrench_cycles + 1;
        if (options.mode == ControllerMode::kExecute
            && stale_wrench_cycles * options.period_ms > 500) {
            fatal_fault = true;
            fatal_fault_code = "force_sensor_stale_over_500ms";
        }

        ContactEstimate contact;
        // Below the contact threshold is a normal finite no-contact state, not
        // a model-loading failure and not an infinite diagnostic residual.
        contact.error = ContactEstimateError::ForceTooSmall;
        contact.residual = 0.0;
        contact.point_error_m = 0.0;
        if (compensated.valid
            && compensated.sensor.head<3>().norm() >= control_config.contact_threshold_n) {
            contact = contact_locator.estimateContactPoint(
                compensated.sensor, control_config.contact_threshold_n);
        }

        ControlIntent intent;
        intent.model_y_m = options.manual_y_m;
        intent.model_rz_deg = options.manual_rz_deg;
        intent.desired_force_n = options.desired_force_n;
        intent.phase_index = options.manual_phase;
        intent.action_enabled = options.manual_action;
        intent.terminate = options.manual_terminate;
        RedisCommandSnapshot redis_command;
        const bool redis_publisher_connected =
            options.redis_enabled && redis.PublisherConnected();
        RedisCommandDecision command_decision;
        if (options.redis_enabled) {
            redis_command = redis.LatestCommand();
            command_decision = RedisBridge::EvaluateCommandForControl(
                redis_command,
                MonotonicNs(cycle_started),
                options.command_stale_ms,
                options.desired_force_n);
            // Redis-enabled control is fail-closed. Local manual commands
            // remain available only through the explicit --no-redis mode.
            intent = command_decision.intent;
        } else {
            command_decision.intent = intent;
            command_decision.valid = true;
        }
        const bool redis_subscriber_connected =
            options.redis_enabled && redis_command.subscriber_connected;
        const std::string& redis_hold_reason = command_decision.hold_reason;
        if (options.mode == ControllerMode::kExecute
            && wrench_valid
            && std::abs(compensated.tool.z())
                   >= control_config.contact_threshold_n
            && compensated.tool.z() * intent.desired_force_n <= 0.0) {
            fatal_fault = true;
            fatal_fault_code = "contact_force_sign_inconsistent";
        }

        Rm75ControlInput control_input;
        control_input.current_pose = model_pose;
        control_input.compensated_wrench_tool = compensated.tool;
        if (contact.valid) {
            control_input.contact_point_probe_m =
                calibration.rotation_tool_from_sensor
                * (contact.point - calibration.probe_tcp_sensor_m);
        } else {
            control_input.contact_point_probe_m.setZero();
        }
        control_input.robot_valid = robot_valid;
        control_input.wrench_valid = wrench_valid;
        control_input.contact_valid = contact.valid;
        const bool arm_control = options.mode != ControllerMode::kObserve;
        Rm75ControlOutput control_output =
            control_law.Step(control_input, intent, arm_control);
        if (!redis_hold_reason.empty() && control_output.fault.empty()) {
            control_output.fault = redis_hold_reason;
        }

        Rm75ServoPlan servo_plan;
        servo_plan.target_joints = model_joints;
        servo_plan.model_pose = model_pose;
        servo_plan.valid = true;
        bool retreat_step_planned = false;
        double retreat_planned_step_m = 0.0;

        if (control_output.request_retract) {
            const bool axial_fz_unload_requested =
                std::abs(compensated.tool.z())
                    >= control_config.force_retract_threshold_z_n;
            const bool non_axial_components_safe =
                std::abs(compensated.tool.x()) <= control_config.force_limit_xy_n
                && std::abs(compensated.tool.y())
                       <= control_config.force_limit_xy_n
                && compensated.tool.tail<3>().cwiseAbs().maxCoeff()
                       <= control_config.torque_limit_nm;
            if (options.mode == ControllerMode::kExecute
                && options.enable_force_retract
                && axial_fz_unload_requested
                && non_axial_components_safe) {
                if (!retreat_active) {
                    retreat_active = true;
                    retreat_initial_abs_fz_n =
                        std::abs(compensated.tool.z());
                    retreat_best_abs_fz_n = retreat_initial_abs_fz_n;
                    retreat_distance_m = 0.0;
                    retreat_cycles = 0;
                }
            } else if (options.mode == ControllerMode::kExecute) {
                fatal_fault = true;
                fatal_fault_code = options.enable_force_retract
                    ? "force_retract_rejected_non_axial_overload"
                    : control_output.fault;
            }
        }
        if (control_output.state == Rm75SupervisorState::kFault
            && !control_output.request_retract
            && !fatal_fault) {
            fatal_fault = true;
            fatal_fault_code = control_output.fault.empty()
                ? "control_supervisor_fault"
                : control_output.fault;
        }

        if (retreat_active && !wrench_valid) {
            fatal_fault = true;
            fatal_fault_code = "force_retract_requires_fresh_wrench";
        }
        if (retreat_active && wrench_valid
            && std::abs(compensated.tool.z())
                   <= control_config.force_retract_release_z_n) {
            // The contact latch in Rm75ControlLaw keeps the next cycle in
            // admittance mode, so this handoff cannot restart fixed-speed
            // approach after a successful unloading retreat.
            retreat_active = false;
            retreat_distance_m = 0.0;
            retreat_cycles = 0;
            previous_joint_delta.setZero();
        }
        if (retreat_active && wrench_valid) {
            ++retreat_cycles;
            retreat_best_abs_fz_n = std::min(
                retreat_best_abs_fz_n,
                std::abs(compensated.tool.z()));
        }

        if (retreat_active && !fatal_fault) {
            const double step_m = std::min(
                options.retract_speed_cm_s / 100.0 * options.period_ms / 1000.0,
                options.retract_distance_mm / 1000.0 - retreat_distance_m);
            if (step_m <= 0.0) {
                fatal_fault = true;
                fatal_fault_code = "force_retract_completed_operator_reset_required";
            } else {
                Eigen::Matrix<double, 6, 1> retreat_pose = model_pose;
                const Eigen::Vector3d retreat_base =
                    RotationBaseFromControllerEuler(model_pose.tail<3>())
                    * Eigen::Vector3d(0.0, 0.0,
                                      options.retract_direction_tool_z * step_m);
                retreat_pose.head<3>() += retreat_base;
                servo_plan = planner.Plan(model_joints,
                                          model_pose,
                                          retreat_pose,
                                          previous_joint_delta);
                if (!servo_plan.valid) {
                    fatal_fault = true;
                    fatal_fault_code = std::string("force_retract_plan_")
                        + Rm75PlanErrorString(servo_plan.error);
                    control_output.fault = servo_plan.detail;
                }
                control_output.state = Rm75SupervisorState::kRetreat;
                control_output.desired_pose = retreat_pose;
                control_output.fault = "wrench_unload_requested";
                if (servo_plan.valid) {
                    retreat_step_planned = true;
                    retreat_planned_step_m = step_m;
                }
            }
        } else if (!retreat_active && !fatal_fault
                   && control_output.command_motion) {
            servo_plan = planner.Plan(model_joints,
                                      model_pose,
                                      control_output.desired_pose,
                                      previous_joint_delta);
            if (!servo_plan.valid) {
                fatal_fault = true;
                fatal_fault_code = std::string("servo_plan_")
                    + Rm75PlanErrorString(servo_plan.error);
                control_output.fault = servo_plan.detail;
            }
        }

        // A fixed-period target must never be queued behind an earlier target
        // that is still being written, and two actual socket writes must stay
        // at least 10 ms apart. Together with the planner's 0.5 speed and 0.25
        // acceleration envelopes, this bounds dispatch jitter conservatively.
        if (!fatal_fault && options.mode == ControllerMode::kExecute) {
            if (g_stop_requested.load()) {
                fatal_fault = true;
                fatal_fault_code = "operator_stop_requested";
                control_output.fault = "SIGINT/SIGTERM requested StopMotion";
            }
            servo_status = command.ServoStatus();
            if (!fatal_fault
                && servo_status.result_sequence != 0
                && !servo_status.result) {
                fatal_fault = true;
                fatal_fault_code = "servoj_async_failed";
                control_output.fault = servo_status.result.message;
            } else if (!fatal_fault && ServoOutstanding(servo_status)) {
                fatal_fault = true;
                fatal_fault_code = "servoj_previous_target_outstanding";
                control_output.fault =
                    "previous ServoJ target is still pending or in flight";
            } else if (!fatal_fault
                       && servo_status.sent_sequence != 0
                       && servo_status.sent_at
                              != std::chrono::steady_clock::time_point{}
                       && cycle_started - servo_status.sent_at
                              < kMinimumAsyncServoSendGap) {
                fatal_fault = true;
                fatal_fault_code = "servoj_dispatch_cadence_unsafe";
                control_output.fault =
                    "previous ServoJ socket write was less than 10 ms ago";
            }
        }

        if (!fatal_fault && servo_plan.valid
            && (control_output.command_motion || retreat_step_planned)) {
            const Eigen::Matrix3d planned_rotation =
                RotationBaseFromControllerEuler(servo_plan.model_pose.tail<3>());
            const Eigen::AngleAxisd orientation_excursion(
                planned_rotation * initial_tool_rotation.transpose());
            if (options.maximum_orientation_excursion_deg > 0.0
                && orientation_excursion.angle() * 180.0 / M_PI
                       > options.maximum_orientation_excursion_deg) {
                fatal_fault = true;
                fatal_fault_code = "maximum_orientation_excursion_exceeded";
            } else if ((ProbeTcpBase(servo_plan.model_pose,
                              control_config.probe_tcp_tool_m)
                 - initial_probe_tcp).norm()
                > options.maximum_approach_distance_mm / 1000.0) {
                fatal_fault = true;
                fatal_fault_code = "maximum_approach_distance_exceeded";
            } else {
                if (options.mode == ControllerMode::kExecute) {
                    const RMResult send_result =
                        command.TryServoJ(servo_plan.target_joints, false);
                    if (!send_result) {
                        fatal_fault = true;
                        fatal_fault_code = "servoj_send_failed";
                        control_output.fault = send_result.message;
                    }
                }
                if (!fatal_fault) {
                    model_joints = servo_plan.target_joints;
                    model_pose = servo_plan.model_pose;
                    previous_joint_delta = servo_plan.joint_delta;
                    if (retreat_step_planned
                        && options.mode == ControllerMode::kExecute) {
                        retreat_distance_m += retreat_planned_step_m;
                    }
                }
            }
        } else if (!fatal_fault && options.mode == ControllerMode::kExecute) {
            const RMResult hold_result = command.TryHoldMotion(model_joints, false);
            if (!hold_result) {
                fatal_fault = true;
                fatal_fault_code = "hold_send_failed";
                control_output.fault = hold_result.message;
            }
            previous_joint_delta.setZero();
        }

        if (!fatal_fault && options.mode == ControllerMode::kExecute) {
            servo_status = command.ServoStatus();
            if (servo_status.sent_sequence > last_servo_sent_sequence) {
                last_servo_sent_sequence = servo_status.sent_sequence;
                last_servo_progress_at = cycle_started;
            }
            if (servo_status.result_sequence != 0 && !servo_status.result) {
                fatal_fault = true;
                fatal_fault_code = "servoj_async_failed";
                control_output.fault = servo_status.result.message;
            } else if (ServoOutstanding(servo_status)
                       && cycle_started - last_servo_progress_at
                              > std::chrono::milliseconds(100)) {
                fatal_fault = true;
                fatal_fault_code = "servoj_mailbox_stalled";
                control_output.fault =
                    "ServoJ target remained unsent for more than 100 ms";
            }
        }

        if (fatal_fault) {
            control_output.state = Rm75SupervisorState::kFault;
            if (control_output.fault.empty()) control_output.fault = fatal_fault_code;
            if (options.mode == ControllerMode::kExecute) {
                const RMResult stop_result = StopAndConfirmStationary(
                    command,
                    *state_reader,
                    control_config.probe_tcp_tool_m);
                if (!stop_result) {
                    std::cerr << "StopMotion/stationary confirmation failed: "
                              << stop_result.message << '\n';
                    fatal_fault_code += "_stop_not_physically_confirmed";
                    control_output.fault = stop_result.message;
                } else {
                    execute_stop_guard.Disarm();
                }
            }
        }

        if (control_output.state != previous_state
            || control_output.fault != previous_reported_fault) {
            const bool completed = control_output.completed;
            if (completed) legacy_completion_pending = true;
            redis.PublishStatus(control_output.state,
                                fatal_fault ? "error" : (completed ? "success" : "active"),
                                completed
                                    ? (control_output.completion_reason.empty()
                                           ? "Movement completed"
                                           : control_output.completion_reason)
                                          : std::string("Controller state: ")
                                                + ToString(control_output.state),
                                control_output.fault,
                                intent.sequence);
            previous_state = control_output.state;
            previous_reported_fault = control_output.fault;
        }

        if (legacy_completion_pending
            || cycle % static_cast<std::uint64_t>(options.publish_every) == 0) {
            RedisSensorMessage message;
            message.sequence = force_sample.sequence;
            message.timestamp_ns = force_sample.monotonic_timestamp
                    == std::chrono::steady_clock::time_point{}
                ? 0
                : MonotonicNs(force_sample.monotonic_timestamp);
            message.source_timestamp_unix_ns =
                force_sample.timestamp == std::chrono::system_clock::time_point{}
                ? 0
                : SystemNs(force_sample.timestamp);
            message.raw_wrench_sensor = raw_wrench;
            message.compensated_wrench_tool = compensated.tool;
            message.legacy_contact_point_sensor_m = contact.point;
            if (contact.valid) {
                message.contact_point_probe_m =
                    contact.point - calibration.probe_tcp_sensor_m;
            }
            message.contact_residual_nm = contact.valid ? contact.residual : 0.0;
            message.contact_point_error_m =
                contact.valid ? contact.point_error_m : 0.0;
            message.contact_error = ContactEstimateErrorString(contact.error);
            message.wrench_valid = wrench_valid;
            message.contact_valid = contact.valid;
            message.checksum_valid = force_sample.checksum_valid;
            message.sensor_stale = force_sample.stale;
            message.sensor_io_status =
                ForceSensorIoStatusName(force_sample.io_status);
            message.sensor_io_error = force_sample.io_error;
            message.legacy_completion = legacy_completion_pending;
            message.control_state = ToString(control_output.state);
            message.fault = control_output.fault;
            redis.PublishSensor(message);
            legacy_completion_pending = false;
        }

        RuntimeLogRow row;
        row.cycle = cycle;
        row.monotonic_ns = MonotonicNs();
        row.cycle_interval_us = cycle_interval_us;
        row.state = fatal_fault ? Rm75SupervisorState::kFault : control_output.state;
        row.robot_sequence = robot_state.sequence;
        row.sensor_sequence = force_sample.sequence;
        row.robot_received_monotonic_ns =
            robot_state.received_at == std::chrono::steady_clock::time_point{}
            ? 0
            : MonotonicNs(robot_state.received_at);
        row.sensor_received_monotonic_ns =
            force_sample.monotonic_timestamp
                    == std::chrono::steady_clock::time_point{}
            ? 0
            : MonotonicNs(force_sample.monotonic_timestamp);
        row.sensor_source_unix_ns =
            force_sample.timestamp == std::chrono::system_clock::time_point{}
            ? 0
            : SystemNs(force_sample.timestamp);
        row.robot_valid = robot_valid;
        row.wrench_valid = wrench_valid;
        row.arm_error = robot_state.arm_err;
        row.system_error = robot_state.sys_err;
        row.robot_age_ms = robot_state.received_at
                == std::chrono::steady_clock::time_point{}
            ? std::numeric_limits<double>::infinity()
            : std::max(0.0,
                       std::chrono::duration<double, std::milli>(
                           cycle_started - robot_state.received_at).count());
        row.sensor_age_ms = force_sample.monotonic_timestamp
                == std::chrono::steady_clock::time_point{}
            ? std::numeric_limits<double>::infinity()
            : std::max(0.0,
                       std::chrono::duration<double, std::milli>(
                           cycle_started
                           - force_sample.monotonic_timestamp).count());
        row.checksum_valid = force_sample.checksum_valid;
        row.sensor_io_status = force_sample.io_status;
        row.sensor_io_error = force_sample.io_error;
        row.redis_enabled = options.redis_enabled;
        row.redis_subscriber_connected = redis_subscriber_connected;
        row.redis_publisher_connected = redis_publisher_connected;
        row.command_present = command_decision.present;
        row.command_valid = command_decision.valid;
        row.command_fresh = command_decision.fresh;
        row.command_sequence = intent.sequence;
        row.command_producer_sequence = redis_command.producer_sequence;
        row.command_connection_generation =
            redis_command.connection_generation;
        row.command_age_ms = command_decision.age_ms;
        row.command_action_enabled = intent.action_enabled;
        row.command_terminate = intent.terminate;
        row.command_phase_index = intent.phase_index;
        row.command_model_y_m = intent.model_y_m;
        row.command_model_rz_deg = intent.model_rz_deg;
        row.command_desired_force_n = intent.desired_force_n;
        row.command_hold_reason = redis_hold_reason;
        row.servo_submitted_sequence = servo_status.submitted_sequence;
        row.servo_consumed_sequence = servo_status.consumed_sequence;
        row.servo_discarded_sequence = servo_status.discarded_sequence;
        row.servo_pending_sequence = servo_status.pending_sequence;
        row.servo_sent_sequence = servo_status.sent_sequence;
        row.servo_result_sequence = servo_status.result_sequence;
        row.servo_result_code = servo_status.result_sequence == 0
            ? RMErrorCode::kNone
            : servo_status.result.code;
        row.servo_outstanding = ServoOutstanding(servo_status);
        row.actual_joints = robot_state.joints;
        row.actual_pose = robot_state.pose;
        row.raw_wrench = raw_wrench;
        row.compensated_wrench = compensated.tool;
        row.control_wrench = control_output.control_wrench_tool;
        row.contact = contact;
        row.filtered_contact_point =
            control_output.filtered_contact_point_probe_m;
        row.requested_delta = control_output.requested_delta;
        row.desired_pose = control_output.desired_pose;
        row.target_joints = servo_plan.target_joints;
        row.fault = fatal_fault ? fatal_fault_code : control_output.fault;

        next_tick += std::chrono::milliseconds(options.period_ms);
        const CycleTimingResult timing = logger.PushAndMeasure(
            std::move(row), cycle_started, next_tick);
        const auto work_finished = timing.finished;
        const bool missed = timing.deadline_missed;
        const double work_us = timing.work_us;
        if (work_durations_us.size() < kMaximumTimingSamples) {
            work_durations_us.push_back(work_us);
        } else {
            work_durations_us[static_cast<std::size_t>(
                (cycle - 1) % kMaximumTimingSamples)] = work_us;
        }
        if (missed) {
            ++missed_periods;
            ++consecutive_missed_periods;
            maximum_consecutive_missed_periods = std::max(
                maximum_consecutive_missed_periods,
                consecutive_missed_periods);
        } else {
            consecutive_missed_periods = 0;
        }
        if (!fatal_fault && options.mode == ControllerMode::kExecute
            && (consecutive_missed_periods >= 2 || work_us > 40000.0
                || cycle_interval_us > 40000.0)) {
            fatal_fault = true;
            fatal_fault_code = "control_deadline_exceeded";
            const RMResult stop_result = StopAndConfirmStationary(
                command,
                *state_reader,
                control_config.probe_tcp_tool_m);
            if (!stop_result) {
                std::cerr << "Stop/stationary confirmation after deadline failed: "
                          << stop_result.message << '\n';
                fatal_fault_code += "_stop_not_physically_confirmed";
            } else {
                execute_stop_guard.Disarm();
            }
        }

        if (fatal_fault) break;

        if (!missed) {
            std::this_thread::sleep_until(next_tick);
        } else if (work_finished - next_tick
                   > std::chrono::milliseconds(options.period_ms)) {
            next_tick = work_finished;
        }
    }

    if (options.mode == ControllerMode::kExecute && !fatal_fault) {
        const RMResult stop_result = StopAndConfirmStationary(
            command,
            *state_reader,
            control_config.probe_tcp_tool_m);
        if (!stop_result) {
            std::cerr << "Final stop/stationary confirmation failed: "
                      << stop_result.message << '\n';
            fatal_fault = true;
            fatal_fault_code = "final_stop_not_physically_confirmed";
        } else {
            execute_stop_guard.Disarm();
        }
    }
    if (options.mode == ControllerMode::kExecute) {
        servo_status = command.ServoStatus();
    }
    if (state_reader) state_reader->Stop();
    if (force_reader) force_reader->Stop();
    redis.PublishStatus(fatal_fault ? Rm75SupervisorState::kFault
                                    : Rm75SupervisorState::kHold,
                        fatal_fault ? "error" : "success",
                        fatal_fault ? "Controller stopped on fault"
                                    : "Controller stopped cleanly",
                        fatal_fault_code,
                        0);
    redis.Stop();
    logger.Stop();

    const double p99_us = Percentile99(work_durations_us);
    const double maximum_us = work_durations_us.empty()
        ? 0.0
        : *std::max_element(work_durations_us.begin(), work_durations_us.end());
    const double on_time_percent = cycle == 0
        ? 0.0
        : 100.0 * (cycle - missed_periods) / cycle;
    const std::uint64_t measured_periods = cycle > 0 ? cycle - 1 : 0;
    const double within_22ms_percent = measured_periods == 0
        ? 0.0
        : 100.0 * (measured_periods - periods_over_22ms) / measured_periods;
    std::filesystem::path summary_path(options.runtime_log_path);
    summary_path.replace_extension(".summary.json");
    nlohmann::json summary;
    summary["schema_version"] = 2;
    summary["controller"] = "main_rm75";
    summary["mode"] = ModeName(options.mode);
    summary["simulated"] = options.simulate;
    summary["result"] = fatal_fault ? "fault" : "completed";
    summary["fault_code"] = fatal_fault_code.empty()
        ? nlohmann::json(nullptr)
        : nlohmann::json(fatal_fault_code);
    summary["calibration"] = {
        {"sensor_id", calibration.sensor_id},
        {"probe_model", calibration.probe_model},
        {"probe_model_sha256", selected_probe_sha256},
        {"rotation_tool_from_sensor_row_major", {
            calibration.rotation_tool_from_sensor(0, 0),
            calibration.rotation_tool_from_sensor(0, 1),
            calibration.rotation_tool_from_sensor(0, 2),
            calibration.rotation_tool_from_sensor(1, 0),
            calibration.rotation_tool_from_sensor(1, 1),
            calibration.rotation_tool_from_sensor(1, 2),
            calibration.rotation_tool_from_sensor(2, 0),
            calibration.rotation_tool_from_sensor(2, 1),
            calibration.rotation_tool_from_sensor(2, 2)}},
        {"translation_sensor_to_tool_m", {
            calibration.translation_sensor_to_tool_m.x(),
            calibration.translation_sensor_to_tool_m.y(),
            calibration.translation_sensor_to_tool_m.z()}},
        {"probe_tcp_sensor_m", {
            calibration.probe_tcp_sensor_m.x(),
            calibration.probe_tcp_sensor_m.y(),
            calibration.probe_tcp_sensor_m.z()}},
        {"tool_chain_verified", calibration.tool_chain_verified},
        {"residuals_verified", calibration.calibration_residuals_verified}};
    summary["provisional_force_control"] = provisional_execute;
    summary["configuration_profile"] = options.implicit_commissioning_profile
        ? kImplicitProfileName
        : "explicit_cli";
    summary["control"] = {
        {"duration_s", options.duration_s},
        {"desired_force_n", options.desired_force_n},
        {"approach_speed_cm_s", options.approach_speed_cm_s},
        {"approach_direction_tool_z", options.approach_direction_tool_z},
        {"maximum_linear_speed_cm_s", options.maximum_linear_speed_cm_s},
        {"maximum_approach_distance_mm",
         options.maximum_approach_distance_mm},
        {"retract_direction_tool_z", options.retract_direction_tool_z},
        {"retract_distance_mm", options.retract_distance_mm},
        {"retract_speed_cm_s", options.retract_speed_cm_s},
        {"maximum_orientation_excursion_deg",
         options.maximum_orientation_excursion_deg},
        {"force_retract_enabled",
         control_config.force_retract_enabled},
        {"force_limit_xy_n", control_config.force_limit_xy_n},
        {"force_limit_z_n", control_config.force_limit_z_n},
        {"torque_limit_nm", control_config.torque_limit_nm},
        {"raw_force_limit_n", options.raw_force_limit_n},
        {"raw_torque_limit_nm", options.raw_torque_limit_nm},
        {"contact_threshold_n", control_config.contact_threshold_n},
        {"force_virtual_mass", control_config.force_virtual_mass},
        {"force_virtual_damping", control_config.force_virtual_damping},
        {"legacy_six_axis_force_stage",
         options.legacy_six_axis_force_stage},
        {"legacy_wrench_filter_enabled",
         control_config.legacy_wrench_filter_enabled},
        {"legacy_contact_filter_enabled",
         control_config.legacy_contact_filter_enabled},
        {"legacy_contact_roll_enabled",
         control_config.legacy_contact_roll_enabled},
        {"legacy_wrench_filter_measurement_noise",
         control_config.legacy_wrench_filter_measurement_noise},
        {"legacy_wrench_filter_process_noise",
         control_config.legacy_wrench_filter_process_noise},
        {"legacy_contact_filter_measurement_noise",
         control_config.legacy_contact_filter_measurement_noise},
        {"legacy_contact_filter_process_noise",
         control_config.legacy_contact_filter_process_noise},
        {"legacy_contact_roll_virtual_mass",
         control_config.legacy_contact_roll_virtual_mass},
        {"legacy_contact_roll_virtual_damping",
         control_config.legacy_contact_roll_virtual_damping},
        {"legacy_contact_roll_scale",
         control_config.legacy_contact_roll_scale},
        {"legacy_contact_roll_limits_enabled",
         control_config.legacy_contact_roll_limits_enabled},
        {"max_tracking_joint_error_deg",
         options.max_tracking_joint_error_deg},
        {"max_tracking_position_error_mm",
         options.max_tracking_position_error_mm},
        {"max_tracking_orientation_error_deg",
         options.max_tracking_orientation_error_deg}};
    summary["runtime_tare"] = {
        {"applied", runtime_tare_applied},
        {"samples", runtime_tare.samples},
        {"sensor_offset", {
            runtime_tare.sensor_offset[0], runtime_tare.sensor_offset[1],
            runtime_tare.sensor_offset[2], runtime_tare.sensor_offset[3],
            runtime_tare.sensor_offset[4], runtime_tare.sensor_offset[5]}},
        {"maximum_force_norm_n", runtime_tare.maximum_force_norm_n},
        {"maximum_torque_norm_nm", runtime_tare.maximum_torque_norm_nm},
        {"maximum_force_deviation_n",
         runtime_tare.maximum_force_deviation_n},
        {"maximum_torque_deviation_nm",
         runtime_tare.maximum_torque_deviation_nm},
        {"maximum_joint_span_deg", runtime_tare.maximum_joint_span_deg},
        {"maximum_tcp_span_mm", runtime_tare.maximum_tcp_span_mm},
        {"maximum_orientation_span_deg",
         runtime_tare.maximum_orientation_span_deg}};
    summary["timing"] = {
        {"period_ms", options.period_ms},
        {"cycles", cycle},
        {"missed_periods", missed_periods},
        {"periods_within_22ms_percent", within_22ms_percent},
        {"periods_over_22ms", periods_over_22ms},
        {"periods_over_40ms", periods_over_40ms},
        {"maximum_consecutive_missed_periods",
         maximum_consecutive_missed_periods},
        {"work_p99_us", p99_us},
        {"work_max_us", maximum_us}};
    summary["runtime_csv"] = options.runtime_log_path;
    summary["logger_dropped_rows"] = logger.DroppedRows();
    summary["redis"] = {
        {"enabled", options.redis_enabled},
        {"command_stale_ms", options.command_stale_ms},
        {"fail_closed", true},
        {"requires_new_command_after_reconnect", true}};
    summary["servo"] = {
        {"submitted_sequence", servo_status.submitted_sequence},
        {"consumed_sequence", servo_status.consumed_sequence},
        {"discarded_sequence", servo_status.discarded_sequence},
        {"pending_sequence", servo_status.pending_sequence},
        {"sent_sequence", servo_status.sent_sequence},
        {"result_sequence", servo_status.result_sequence},
        {"result_code", servo_status.result_sequence == 0
                            ? static_cast<int>(RMErrorCode::kNone)
                            : static_cast<int>(servo_status.result.code)},
        {"mailbox_pending", servo_status.pending()},
        {"outstanding", ServoOutstanding(servo_status)}};
    std::ofstream summary_stream(summary_path);
    if (summary_stream) {
        summary_stream << std::setw(2) << summary << '\n';
    } else {
        std::cerr << "Unable to write runtime summary: "
                  << summary_path.string() << '\n';
    }
    std::cout << "cycles: " << cycle << '\n'
              << "result: " << (fatal_fault ? "fault" : "completed") << '\n'
              << "fault_code: "
              << (fatal_fault_code.empty() ? "none" : fatal_fault_code) << '\n'
              << "missed_periods: " << missed_periods << '\n'
              << "on_time_percent: " << on_time_percent << '\n'
              << "periods_within_22ms_percent: " << within_22ms_percent << '\n'
              << "periods_over_22ms: " << periods_over_22ms << '\n'
              << "periods_over_40ms: " << periods_over_40ms << '\n'
              << "maximum_consecutive_missed_periods: "
              << maximum_consecutive_missed_periods << '\n'
              << "work_p99_us: " << p99_us << '\n'
              << "work_max_us: " << maximum_us << '\n'
              << "logger_dropped_rows: " << logger.DroppedRows() << '\n'
              << "runtime_log_written: " << logger.Path() << '\n';
    std::cout << "runtime_summary_written: " << summary_path.string() << '\n';

    return fatal_fault ? 6 : 0;
}
