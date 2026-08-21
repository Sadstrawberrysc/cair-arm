#pragma once

#include <array>
#include <string_view>

inline constexpr int kRuntimeSummarySchemaVersion = 2;

inline constexpr std::array<std::string_view, 17>
    kRuntimeSummaryV2TopLevelKeys{{
        "calibration",
        "completion_reason",
        "configuration_profile",
        "control",
        "controller",
        "fault_code",
        "logger_dropped_rows",
        "mode",
        "provisional_force_control",
        "redis",
        "result",
        "runtime_csv",
        "runtime_tare",
        "schema_version",
        "servo",
        "simulated",
        "timing",
    }};

inline constexpr std::string_view kRuntimeCsvHeader =
    "cycle,monotonic_ns,work_us,cycle_interval_us,"
    "deadline_lateness_us,deadline_missed,state,"
    "robot_sequence,sensor_sequence,"
    "robot_received_monotonic_ns,sensor_received_monotonic_ns,"
    "sensor_source_unix_ns,robot_valid,wrench_valid,"
    "arm_error,system_error,robot_age_ms,sensor_age_ms,"
    "checksum_valid,sensor_io_status,sensor_io_error,"
    "redis_enabled,redis_subscriber_connected,"
    "redis_publisher_connected,command_present,command_valid,"
    "command_fresh,command_sequence,command_producer_sequence,"
    "command_protocol_version,command_session_id,"
    "command_producer_timestamp_unix_ms,command_phase_confidence,"
    "command_connection_generation,"
    "command_age_ms,command_action_enabled,command_terminate,"
    "command_recovery_mode,command_mask_lr_majority,"
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
    "reference_model_position_error_mm,"
    "reference_model_orientation_error_deg,"
    "robot_model_position_error_mm,robot_model_tool_y_error_mm,"
    "target_force_unloading,"
    "target_force_recovering,"
    "target_force_reference_rebased,"
    "visual_y_tracking_paused,"
    "visual_y_tracking_reference_rebased,"
    "recovery_search_active,recovery_locked_mask_side,"
    "recovery_tool_y_velocity_m_s,recovery_search_distance_m,"
    "recovery_distance_limit_reached,"
    "j1_rad,j2_rad,j3_rad,j4_rad,j5_rad,j6_rad,j7_rad,fault";
