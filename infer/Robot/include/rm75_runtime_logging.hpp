#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <json.hpp>

#include <contact_sensing.hpp>
#include <force_calibration.hpp>
#include <force_sensor.hpp>
#include <realman_command.hpp>
#include <rm75_control.hpp>

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
    int command_protocol_version = 0;
    std::string command_session_id;
    std::int64_t command_producer_timestamp_unix_ms = 0;
    double command_phase_confidence = 0.0;
    std::uint64_t command_connection_generation = 0;
    double command_age_ms = 0.0;
    bool command_action_enabled = false;
    bool command_terminate = false;
    bool command_recovery_mode = false;
    int command_mask_lr_majority = 0;
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
    double reference_model_position_error_mm = 0.0;
    double reference_model_orientation_error_deg = 0.0;
    double robot_model_position_error_mm = 0.0;
    double robot_model_tool_y_error_mm = 0.0;
    bool target_force_unloading = false;
    bool target_force_recovering = false;
    bool target_force_reference_rebased = false;
    bool visual_y_tracking_paused = false;
    bool visual_y_tracking_reference_rebased = false;
    bool recovery_search_active = false;
    int recovery_locked_mask_side = 0;
    double recovery_tool_y_velocity_m_s = 0.0;
    double recovery_search_distance_m = 0.0;
    bool recovery_distance_limit_reached = false;
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
    bool Start(const std::string& path, std::string* error);
    CycleTimingResult PushAndMeasure(
        RuntimeLogRow row,
        std::chrono::steady_clock::time_point cycle_started,
        std::chrono::steady_clock::time_point deadline);
    void Stop();
    ~AsyncRuntimeLogger();

    std::uint64_t DroppedRows() const;
    const std::string& Path() const;

private:
    void ThreadMain();

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

struct RuntimeTareSummary {
    bool applied = false;
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
};

struct RuntimeControlSummary {
    int duration_s = 0;
    int phase_index = -1;
    Rm75ControlConfig config;
    Rm75RuntimeSafetyConfig safety;
    double planned_scan_distance_m = 0.0;
    double maximum_reference_model_position_error_mm = 0.0;
    double maximum_reference_model_orientation_error_deg = 0.0;
    double maximum_robot_model_position_error_mm = 0.0;
    double maximum_robot_model_tool_y_error_mm = 0.0;
    std::uint64_t visual_y_tracking_paused_cycles = 0;
    std::uint64_t recovery_search_cycles = 0;
    std::uint64_t visual_y_tracking_reference_rebases = 0;
    std::uint64_t target_force_unloading_cycles = 0;
    std::uint64_t target_force_recovering_cycles = 0;
    std::uint64_t target_force_reference_rebases = 0;
    std::uint64_t robot_state_stale_hold_cycles = 0;
    std::uint64_t robot_state_recoveries = 0;
};

struct RuntimeTimingSummary {
    int period_ms = 10;
    std::uint64_t cycles = 0;
    std::uint64_t missed_periods = 0;
    std::uint64_t periods_over_22ms = 0;
    std::uint64_t periods_over_40ms = 0;
    std::uint64_t maximum_consecutive_missed_periods = 0;
    double periods_within_22ms_percent = 0.0;
    double on_time_percent = 0.0;
    double work_p99_us = 0.0;
    double work_max_us = 0.0;
};

struct RuntimeSummaryData {
    std::string mode;
    bool simulated = false;
    bool fatal_fault = false;
    std::string completion_reason;
    std::string fault_code;
    bool provisional_force_control = false;
    std::string configuration_profile;

    ForceCalibration calibration;
    std::string selected_probe_sha256;
    Eigen::Vector3d probe_tcp_arm_tip_m = Eigen::Vector3d::Zero();
    RuntimeControlSummary control;
    RuntimeTareSummary tare;
    RuntimeTimingSummary timing;

    std::string runtime_csv;
    std::uint64_t logger_dropped_rows = 0;
    bool redis_enabled = false;
    int command_stale_ms = 500;
    ServoSendSnapshot servo;
    bool servo_outstanding = false;
};

nlohmann::json BuildRuntimeSummary(const RuntimeSummaryData& data);
bool WriteRuntimeSummary(const std::string& runtime_csv_path,
                         const nlohmann::json& summary,
                         std::string* summary_path,
                         std::string* error);
void PrintRuntimeCompletion(const RuntimeSummaryData& data,
                            const std::string& summary_path,
                            const std::string& logger_path);
