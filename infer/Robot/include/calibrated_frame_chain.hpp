#pragma once

#include <Eigen/Dense>

#include <force_calibration.hpp>
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
