#include <rm75_runtime_logging.hpp>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <utility>

#include <runtime_schema.hpp>

bool AsyncRuntimeLogger::Start(const std::string& path, std::string* error) {
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
    stream_ << kRuntimeCsvHeader << '\n';
    queue_.resize(kMaximumRows);
    queue_head_ = 0;
    queue_size_ = 0;
    running_.store(true);
    thread_ = std::thread(&AsyncRuntimeLogger::ThreadMain, this);
    return true;
}

CycleTimingResult AsyncRuntimeLogger::PushAndMeasure(
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
        queue_[index].deadline_lateness_us = timing.deadline_lateness_us;
        queue_[index].deadline_missed = timing.deadline_missed;
        ++queue_size_;
    }
    condition_.notify_one();
    return timing;
}

void AsyncRuntimeLogger::Stop() {
    if (!running_.exchange(false)) return;
    condition_.notify_all();
    if (thread_.joinable()) thread_.join();
    stream_.close();
}

AsyncRuntimeLogger::~AsyncRuntimeLogger() { Stop(); }

std::uint64_t AsyncRuntimeLogger::DroppedRows() const {
    return dropped_rows_.load();
}

const std::string& AsyncRuntimeLogger::Path() const { return path_; }

void AsyncRuntimeLogger::ThreadMain() {
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
                << row.sensor_sequence << ','
                << row.robot_received_monotonic_ns << ','
                << row.sensor_received_monotonic_ns << ','
                << row.sensor_source_unix_ns << ','
                << (row.robot_valid ? 1 : 0) << ','
                << (row.wrench_valid ? 1 : 0) << ','
                << row.arm_error << ',' << row.system_error << ','
                << row.robot_age_ms << ',' << row.sensor_age_ms << ','
                << (row.checksum_valid ? 1 : 0) << ','
                << ForceSensorIoStatusName(row.sensor_io_status) << ','
                << row.sensor_io_error << ','
                << (row.redis_enabled ? 1 : 0) << ','
                << (row.redis_subscriber_connected ? 1 : 0) << ','
                << (row.redis_publisher_connected ? 1 : 0) << ','
                << (row.command_present ? 1 : 0) << ','
                << (row.command_valid ? 1 : 0) << ','
                << (row.command_fresh ? 1 : 0) << ','
                << row.command_sequence << ','
                << row.command_producer_sequence << ','
                << row.command_protocol_version;
        std::replace(row.command_session_id.begin(),
                     row.command_session_id.end(), ',', ';');
        stream_ << ',' << row.command_session_id << ','
                << row.command_producer_timestamp_unix_ms << ','
                << row.command_phase_confidence << ','
                << row.command_connection_generation << ','
                << row.command_age_ms << ','
                << (row.command_action_enabled ? 1 : 0) << ','
                << (row.command_terminate ? 1 : 0) << ','
                << (row.command_recovery_mode ? 1 : 0) << ','
                << row.command_mask_lr_majority << ','
                << row.command_phase_index << ','
                << row.command_model_y_m << ','
                << row.command_model_rz_deg << ','
                << row.command_desired_force_n;
        std::replace(row.command_hold_reason.begin(),
                     row.command_hold_reason.end(), ',', ';');
        stream_ << ',' << row.command_hold_reason << ','
                << row.servo_submitted_sequence << ','
                << row.servo_consumed_sequence << ','
                << row.servo_discarded_sequence << ','
                << row.servo_pending_sequence << ','
                << row.servo_sent_sequence << ','
                << row.servo_result_sequence << ','
                << static_cast<int>(row.servo_result_code) << ','
                << (row.servo_outstanding ? 1 : 0);
        for (int i = 0; i < 7; ++i) stream_ << ',' << row.actual_joints[i];
        for (int i = 0; i < 6; ++i) stream_ << ',' << row.actual_pose[i];
        for (int i = 0; i < 6; ++i) stream_ << ',' << row.raw_wrench[i];
        for (int i = 0; i < 6; ++i) stream_ << ',' << row.compensated_wrench[i];
        for (int i = 0; i < 6; ++i) stream_ << ',' << row.control_wrench[i];
        stream_ << ',' << (row.contact.valid ? 1 : 0) << ','
                << row.contact.point.x() << ',' << row.contact.point.y() << ','
                << row.contact.point.z() << ',' << row.contact.residual << ','
                << row.contact.point_error_m << ','
                << row.filtered_contact_point.x() << ','
                << row.filtered_contact_point.y() << ','
                << row.filtered_contact_point.z();
        for (int i = 0; i < 6; ++i) stream_ << ',' << row.requested_delta[i];
        for (int i = 0; i < 6; ++i) stream_ << ',' << row.desired_pose[i];
        stream_ << ',' << row.reference_model_position_error_mm << ','
                << row.reference_model_orientation_error_deg << ','
                << row.robot_model_position_error_mm << ','
                << row.robot_model_tool_y_error_mm << ','
                << (row.target_force_unloading ? 1 : 0) << ','
                << (row.target_force_recovering ? 1 : 0) << ','
                << (row.target_force_reference_rebased ? 1 : 0) << ','
                << (row.visual_y_tracking_paused ? 1 : 0) << ','
                << (row.visual_y_tracking_reference_rebased ? 1 : 0) << ','
                << (row.recovery_search_active ? 1 : 0) << ','
                << row.recovery_locked_mask_side << ','
                << row.recovery_tool_y_velocity_m_s << ','
                << row.recovery_search_distance_m << ','
                << (row.recovery_distance_limit_reached ? 1 : 0);
        for (int i = 0; i < 7; ++i) stream_ << ',' << row.target_joints[i];
        std::replace(row.fault.begin(), row.fault.end(), ',', ';');
        stream_ << ',' << row.fault << '\n';
    }
    stream_.flush();
}

nlohmann::json BuildRuntimeSummary(const RuntimeSummaryData& data) {
    const auto& calibration = data.calibration;
    const auto& control = data.control;
    const auto& config = control.config;
    const auto& safety = control.safety;
    const auto& tare = data.tare;
    const auto& timing = data.timing;
    const auto& servo = data.servo;

    nlohmann::json summary;
    summary["schema_version"] = kRuntimeSummarySchemaVersion;
    summary["controller"] = "main_rm75";
    summary["mode"] = data.mode;
    summary["simulated"] = data.simulated;
    summary["result"] = data.fatal_fault ? "fault" : "completed";
    summary["completion_reason"] = data.fatal_fault
        ? nlohmann::json(nullptr) : nlohmann::json(data.completion_reason);
    summary["fault_code"] = data.fault_code.empty()
        ? nlohmann::json(nullptr) : nlohmann::json(data.fault_code);
    summary["calibration"] = {
        {"sensor_id", calibration.sensor_id},
        {"probe_model", calibration.probe_model},
        {"probe_model_sha256", data.selected_probe_sha256},
        {"controller_pose_frame", "arm_tip"},
        {"control_tool_frame", "sensor"},
        {"legacy_rotation_field_interpretation",
         "rotation_arm_tip_from_tool_sensor"},
        {"rotation_arm_tip_from_tool_row_major", {
            config.rotation_pose_from_tool(0, 0),
            config.rotation_pose_from_tool(0, 1),
            config.rotation_pose_from_tool(0, 2),
            config.rotation_pose_from_tool(1, 0),
            config.rotation_pose_from_tool(1, 1),
            config.rotation_pose_from_tool(1, 2),
            config.rotation_pose_from_tool(2, 0),
            config.rotation_pose_from_tool(2, 1),
            config.rotation_pose_from_tool(2, 2)}},
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
        {"probe_tcp_arm_tip_m", {
            data.probe_tcp_arm_tip_m.x(), data.probe_tcp_arm_tip_m.y(),
            data.probe_tcp_arm_tip_m.z()}},
        {"tool_chain_verified", calibration.tool_chain_verified},
        {"residuals_verified", calibration.calibration_residuals_verified}};
    summary["provisional_force_control"] = data.provisional_force_control;
    summary["configuration_profile"] = data.configuration_profile;
    summary["control"] = {
        {"duration_s", control.duration_s},
        {"desired_force_n", config.desired_force_n},
        {"approach_speed_cm_s", config.approach_speed_m_s * 100.0},
        {"approach_direction_tool_z", config.approach_direction_tool_z},
        {"maximum_force_axis_speed_cm_s",
         config.max_force_axis_speed_m_s * 100.0},
        {"visual_y_control_mode", "proportional_error_velocity"},
        {"model_y_velocity_gain_per_s", config.model_y_velocity_gain_per_s},
        {"maximum_model_y_step_mm", config.maximum_model_y_step_m * 1000.0},
        {"visual_y_tracking_pause_error_mm",
         config.visual_y_tracking_pause_error_m * 1000.0},
        {"visual_y_tracking_resume_error_mm",
         config.visual_y_tracking_resume_error_m * 1000.0},
        {"maximum_no_contact_approach_distance_mm",
         safety.maximum_no_contact_approach_distance_m * 1000.0},
        {"phase_index", control.phase_index},
        {"scan_start_force_n", config.scan_start_force_n},
        {"visual_y_enable_force_n", config.visual_y_enable_force_n},
        {"visual_y_force_stable_duration_s",
         config.visual_y_force_stable_duration_s},
        {"model_y_direction", config.model_y_direction},
        {"scan_force_tolerance_n", config.scan_force_tolerance_n},
        {"scan_force_stable_duration_s", config.scan_force_stable_duration_s},
        {"scan_speed_cm_s", config.scan_speed_m_s * 100.0},
        {"scan_distance_mm", config.maximum_scan_distance_m * 1000.0},
        {"scan_direction_tool_x", config.scan_direction_tool_x},
        {"planned_scan_distance_mm", control.planned_scan_distance_m * 1000.0},
        {"maximum_reference_model_position_error_mm",
         control.maximum_reference_model_position_error_mm},
        {"maximum_reference_model_orientation_error_deg",
         control.maximum_reference_model_orientation_error_deg},
        {"maximum_robot_model_position_error_mm",
         control.maximum_robot_model_position_error_mm},
        {"maximum_robot_model_tool_y_error_mm",
         control.maximum_robot_model_tool_y_error_mm},
        {"visual_y_tracking_paused_cycles",
         control.visual_y_tracking_paused_cycles},
        {"recovery_search_cycles", control.recovery_search_cycles},
        {"recovery_tool_y_speed_mm_s",
         config.recovery_tool_y_speed_m_s * 1000.0},
        {"maximum_recovery_tool_y_distance_mm",
         config.maximum_recovery_tool_y_distance_m * 1000.0},
        {"visual_y_tracking_reference_rebases",
         control.visual_y_tracking_reference_rebases},
        {"target_force_unloading_cycles",
         control.target_force_unloading_cycles},
        {"target_force_recovering_cycles",
         control.target_force_recovering_cycles},
        {"target_force_reference_rebases",
         control.target_force_reference_rebases},
        {"robot_state_stale_hold_cycles",
         control.robot_state_stale_hold_cycles},
        {"robot_state_recoveries", control.robot_state_recoveries},
        {"retract_direction_tool_z", config.retract_direction_tool_z},
        {"retract_distance_mm", config.retract_distance_m * 1000.0},
        {"retract_speed_cm_s", config.retract_speed_m_s * 100.0},
        {"maximum_orientation_excursion_deg",
         safety.maximum_orientation_excursion_deg},
        {"force_retract_enabled", config.force_retract_enabled},
        {"force_limit_xy_n", config.force_limit_xy_n},
        {"force_limit_z_n", config.force_limit_z_n},
        {"torque_limit_nm", config.torque_limit_nm},
        {"raw_force_limit_n", safety.raw_force_limit_n},
        {"raw_torque_limit_nm", safety.raw_torque_limit_nm},
        {"contact_threshold_n", config.contact_threshold_n},
        {"force_virtual_mass", config.force_virtual_mass},
        {"force_virtual_damping", config.force_virtual_damping},
        {"target_force_unload_enabled", config.target_force_unload_enabled},
        {"target_force_unload_margin_n", config.target_force_unload_margin_n},
        {"target_force_unload_speed_cm_s",
         config.target_force_unload_speed_m_s * 100.0},
        {"target_force_unload_deceleration_band_n",
         config.target_force_unload_deceleration_band_n},
        {"target_force_unload_stop_acceleration_m_s2",
         config.target_force_unload_stop_acceleration_m_s2},
        {"target_force_reacquire_speed_cm_s",
         config.target_force_reacquire_speed_m_s * 100.0},
        {"target_force_recovery_stable_duration_s",
         config.target_force_recovery_stable_duration_s},
        {"maximum_force_axis_acceleration_m_s2",
         config.max_force_axis_acceleration_m_s2},
        {"legacy_six_axis_force_stage",
         config.legacy_wrench_filter_enabled
             && config.legacy_contact_filter_enabled
             && config.legacy_contact_roll_enabled},
        {"legacy_wrench_filter_enabled", config.legacy_wrench_filter_enabled},
        {"legacy_contact_filter_enabled", config.legacy_contact_filter_enabled},
        {"legacy_contact_roll_enabled", config.legacy_contact_roll_enabled},
        {"legacy_wrench_filter_measurement_noise",
         config.legacy_wrench_filter_measurement_noise},
        {"legacy_wrench_filter_process_noise",
         config.legacy_wrench_filter_process_noise},
        {"legacy_contact_filter_measurement_noise",
         config.legacy_contact_filter_measurement_noise},
        {"legacy_contact_filter_process_noise",
         config.legacy_contact_filter_process_noise},
        {"legacy_contact_roll_virtual_mass",
         config.legacy_contact_roll_virtual_mass},
        {"legacy_contact_roll_virtual_damping",
         config.legacy_contact_roll_virtual_damping},
        {"legacy_contact_roll_scale", config.legacy_contact_roll_scale},
        {"legacy_contact_roll_limits_enabled",
         config.legacy_contact_roll_limits_enabled},
        {"max_tracking_joint_error_deg",
         safety.max_tracking_joint_error_deg},
        {"max_tracking_position_error_mm",
         safety.max_tracking_position_error_mm},
        {"max_tracking_orientation_error_deg",
         safety.max_tracking_orientation_error_deg}};
    summary["runtime_tare"] = {
        {"applied", tare.applied},
        {"samples", tare.samples},
        {"sensor_offset", {
            tare.sensor_offset[0], tare.sensor_offset[1], tare.sensor_offset[2],
            tare.sensor_offset[3], tare.sensor_offset[4], tare.sensor_offset[5]}},
        {"maximum_force_norm_n", tare.maximum_force_norm_n},
        {"maximum_torque_norm_nm", tare.maximum_torque_norm_nm},
        {"maximum_force_deviation_n", tare.maximum_force_deviation_n},
        {"maximum_torque_deviation_nm", tare.maximum_torque_deviation_nm},
        {"maximum_joint_span_deg", tare.maximum_joint_span_deg},
        {"maximum_tcp_span_mm", tare.maximum_tcp_span_mm},
        {"maximum_orientation_span_deg", tare.maximum_orientation_span_deg}};
    summary["timing"] = {
        {"period_ms", timing.period_ms},
        {"cycles", timing.cycles},
        {"missed_periods", timing.missed_periods},
        {"periods_within_22ms_percent", timing.periods_within_22ms_percent},
        {"periods_over_22ms", timing.periods_over_22ms},
        {"periods_over_40ms", timing.periods_over_40ms},
        {"maximum_consecutive_missed_periods",
         timing.maximum_consecutive_missed_periods},
        {"work_p99_us", timing.work_p99_us},
        {"work_max_us", timing.work_max_us}};
    summary["runtime_csv"] = data.runtime_csv;
    summary["logger_dropped_rows"] = data.logger_dropped_rows;
    summary["redis"] = {
        {"enabled", data.redis_enabled},
        {"command_stale_ms", data.command_stale_ms},
        {"fail_closed", true},
        {"requires_new_command_after_reconnect", true}};
    summary["servo"] = {
        {"submitted_sequence", servo.submitted_sequence},
        {"consumed_sequence", servo.consumed_sequence},
        {"discarded_sequence", servo.discarded_sequence},
        {"pending_sequence", servo.pending_sequence},
        {"sent_sequence", servo.sent_sequence},
        {"result_sequence", servo.result_sequence},
        {"result_code", servo.result_sequence == 0
                            ? static_cast<int>(RMErrorCode::kNone)
                            : static_cast<int>(servo.result.code)},
        {"mailbox_pending", servo.pending()},
        {"outstanding", data.servo_outstanding}};
    return summary;
}

bool WriteRuntimeSummary(const std::string& runtime_csv_path,
                         const nlohmann::json& summary,
                         std::string* summary_path,
                         std::string* error) {
    std::filesystem::path output(runtime_csv_path);
    output.replace_extension(".summary.json");
    if (summary_path != nullptr) *summary_path = output.string();
    std::ofstream stream(output);
    if (!stream) {
        if (error != nullptr) {
            *error = "Unable to write runtime summary: " + output.string();
        }
        return false;
    }
    stream << std::setw(2) << summary << '\n';
    if (error != nullptr) error->clear();
    return true;
}

void PrintRuntimeCompletion(const RuntimeSummaryData& data,
                            const std::string& summary_path,
                            const std::string& logger_path) {
    const auto& control = data.control;
    const auto& timing = data.timing;
    std::cout << "cycles: " << timing.cycles << '\n'
              << "result: " << (data.fatal_fault ? "fault" : "completed")
              << '\n'
              << "completion_reason: "
              << (data.fatal_fault ? "none" : data.completion_reason) << '\n'
              << "planned_scan_distance_mm: "
              << control.planned_scan_distance_m * 1000.0 << '\n'
              << "maximum_reference_model_position_error_mm: "
              << control.maximum_reference_model_position_error_mm << '\n'
              << "maximum_reference_model_orientation_error_deg: "
              << control.maximum_reference_model_orientation_error_deg << '\n'
              << "maximum_robot_model_position_error_mm: "
              << control.maximum_robot_model_position_error_mm << '\n'
              << "visual_y_tracking_paused_cycles: "
              << control.visual_y_tracking_paused_cycles << '\n'
              << "target_force_unloading_cycles: "
              << control.target_force_unloading_cycles << '\n'
              << "target_force_recovering_cycles: "
              << control.target_force_recovering_cycles << '\n'
              << "target_force_reference_rebases: "
              << control.target_force_reference_rebases << '\n'
              << "robot_state_stale_hold_cycles: "
              << control.robot_state_stale_hold_cycles << '\n'
              << "robot_state_recoveries: "
              << control.robot_state_recoveries << '\n'
              << "fault_code: "
              << (data.fault_code.empty() ? "none" : data.fault_code) << '\n'
              << "missed_periods: " << timing.missed_periods << '\n'
              << "on_time_percent: " << timing.on_time_percent << '\n'
              << "periods_within_22ms_percent: "
              << timing.periods_within_22ms_percent << '\n'
              << "periods_over_22ms: " << timing.periods_over_22ms << '\n'
              << "periods_over_40ms: " << timing.periods_over_40ms << '\n'
              << "maximum_consecutive_missed_periods: "
              << timing.maximum_consecutive_missed_periods << '\n'
              << "work_p99_us: " << timing.work_p99_us << '\n'
              << "work_max_us: " << timing.work_max_us << '\n'
              << "logger_dropped_rows: " << data.logger_dropped_rows << '\n'
              << "runtime_log_written: " << logger_path << '\n'
              << "runtime_summary_written: " << summary_path << '\n';
}
