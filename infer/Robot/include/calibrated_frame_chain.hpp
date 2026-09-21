#pragma once

#include <Eigen/Dense>
#include <deque>

#include <force_calibration.hpp>
#include <realman_command.hpp>

// Immutable transforms loaded from one validated force-calibration file.
// Controller poses are Base -> Arm_Tip. Tool and Sensor axes are coincident
// on the current installation; their fixed mount rotation is expressed as
// Arm_Tip <- Tool/Sensor.
class CalibratedFrameChain {
public:
    explicit CalibratedFrameChain(const ForceCalibration& calibration);
    // Preview-only camera calibration; reading and hashing occur at startup.
    static Eigen::Matrix4d LoadCameraTransform(const std::string& path,
        const std::string& transform_name, const std::string& matrix_key,
        std::string& sha256, bool require_verified = false);
    Eigen::Matrix4d CameraPoseBase(const Eigen::Matrix<double, 6, 1>& pose,
                                  const Eigen::Matrix4d& arm_tip_camera) const;
    Eigen::Matrix4d ToolTcpPoseBase(const Eigen::Matrix<double,6,1>& pose) const;
    Eigen::Matrix<double,6,1> ArmTipPoseFromTcp(const Eigen::Matrix4d& tcp) const;
    static void CameraObservationBase(const Eigen::Matrix4d& camera_pose,
        const Eigen::Vector3d& point_camera, const Eigen::Vector3d& normal_camera,
        Eigen::Vector3d& point_base, Eigen::Vector3d& normal_base);
    struct CameraSample { std::int64_t time_ns; Eigen::Matrix4d pose; };
    static bool InterpolateCamera(const std::deque<CameraSample>& history,
                                  std::int64_t time_ns, Eigen::Matrix4d& pose, double& span_ms);

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
