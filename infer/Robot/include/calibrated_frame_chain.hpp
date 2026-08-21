#pragma once

#include <atomic>

#include <Eigen/Dense>

#include <force_calibration.hpp>
#include <force_sensor.hpp>
#include <realman_command.hpp>

// Immutable transforms loaded from one validated force-calibration file.
// Controller poses are Base -> Arm_Tip. Tool and Sensor axes are coincident
// on the current installation; their fixed mount rotation is expressed as
// Arm_Tip <- Tool/Sensor.
class CalibratedFrameChain {
public:
    explicit CalibratedFrameChain(const ForceCalibration& calibration);

    const Eigen::Matrix3d& RotationArmTipFromTool() const noexcept;
    const Eigen::Vector3d& TranslationSensorToToolM() const noexcept;
    const Eigen::Vector3d& ProbeTcpSensorM() const noexcept;
    const Eigen::Vector3d& ProbeTcpToolM() const noexcept;
    const Eigen::Vector3d& ProbeTcpArmTipM() const noexcept;

    Eigen::Matrix3d RotationBaseFromArmTip(
        const Eigen::Matrix<double, 6, 1>& controller_pose) const;
    Eigen::Matrix3d RotationBaseFromTool(
        const Eigen::Matrix<double, 6, 1>& controller_pose) const;
    Eigen::Vector3d ProbeTcpBase(
        const Eigen::Matrix<double, 6, 1>& controller_pose) const;
    Eigen::Vector3d ToolYAxisBase(
        const Eigen::Matrix<double, 6, 1>& controller_pose) const;
    Eigen::Matrix3d ArmTipOrientationDelta(
        const Eigen::Matrix<double, 6, 1>& current_pose,
        const Eigen::Matrix<double, 6, 1>& reference_pose) const;

private:
    Eigen::Matrix3d rotation_arm_tip_from_tool_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation_sensor_to_tool_m_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d probe_tcp_sensor_m_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d probe_tcp_tool_m_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d probe_tcp_arm_tip_m_ = Eigen::Vector3d::Zero();
};

double MaximumWrappedJointDeltaDeg(
    const Eigen::Matrix<double, 7, 1>& lhs,
    const Eigen::Matrix<double, 7, 1>& rhs);

RMResult StopAndConfirmStationary(
    RMCommand& command,
    RMStateReader& state_reader,
    const CalibratedFrameChain& frame_chain,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));

struct RuntimeTareConfig {
    double maximum_joint_span_deg = 0.02;
    double maximum_tcp_span_mm = 0.35;
    double maximum_orientation_span_deg = 0.10;
    double maximum_force_deviation_n = 0.25;
    double maximum_torque_deviation_nm = 0.02;
    int minimum_samples_per_second = 20;
};

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
    const CalibratedFrameChain& frame_chain,
    const std::atomic<bool>& stop_requested,
    int seconds,
    double raw_force_limit_n,
    double raw_torque_limit_nm,
    const RuntimeTareConfig& config = {});

void ApplyRuntimeTare(const Eigen::Matrix<double, 6, 1>& sensor_offset,
                      CompensatedWrench& compensated);
