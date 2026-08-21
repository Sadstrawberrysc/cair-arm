#include <calibrated_frame_chain.hpp>

#include <algorithm>
#include <cmath>

namespace {

double WrappedAngleDifference(double lhs, double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

}  // namespace

CalibratedFrameChain::CalibratedFrameChain(
    const ForceCalibration& calibration)
    : rotation_arm_tip_from_tool_(calibration.rotation_tool_from_sensor),
      translation_sensor_to_tool_m_(calibration.translation_sensor_to_tool_m),
      probe_tcp_sensor_m_(calibration.probe_tcp_sensor_m),
      probe_tcp_tool_m_(probe_tcp_sensor_m_ - translation_sensor_to_tool_m_),
      probe_tcp_arm_tip_m_(rotation_arm_tip_from_tool_ * probe_tcp_tool_m_) {}

const Eigen::Matrix3d& CalibratedFrameChain::RotationArmTipFromTool()
    const noexcept {
    return rotation_arm_tip_from_tool_;
}

const Eigen::Vector3d& CalibratedFrameChain::ProbeTcpToolM() const noexcept {
    return probe_tcp_tool_m_;
}

const Eigen::Vector3d& CalibratedFrameChain::TranslationSensorToToolM()
    const noexcept {
    return translation_sensor_to_tool_m_;
}

const Eigen::Vector3d& CalibratedFrameChain::ProbeTcpSensorM() const noexcept {
    return probe_tcp_sensor_m_;
}

const Eigen::Vector3d& CalibratedFrameChain::ProbeTcpArmTipM() const noexcept {
    return probe_tcp_arm_tip_m_;
}

Eigen::Matrix3d CalibratedFrameChain::RotationBaseFromArmTip(
    const Eigen::Matrix<double, 6, 1>& controller_pose) const {
    return RotationBaseFromControllerEuler(controller_pose.tail<3>());
}

Eigen::Matrix3d CalibratedFrameChain::RotationBaseFromTool(
    const Eigen::Matrix<double, 6, 1>& controller_pose) const {
    return RotationBaseFromArmTip(controller_pose)
        * rotation_arm_tip_from_tool_;
}

Eigen::Vector3d CalibratedFrameChain::ProbeTcpBase(
    const Eigen::Matrix<double, 6, 1>& controller_pose) const {
    return controller_pose.head<3>()
        + RotationBaseFromArmTip(controller_pose) * probe_tcp_arm_tip_m_;
}

Eigen::Vector3d CalibratedFrameChain::ToolYAxisBase(
    const Eigen::Matrix<double, 6, 1>& controller_pose) const {
    return RotationBaseFromArmTip(controller_pose)
        * rotation_arm_tip_from_tool_.col(1);
}

Eigen::Matrix3d CalibratedFrameChain::ArmTipOrientationDelta(
    const Eigen::Matrix<double, 6, 1>& current_pose,
    const Eigen::Matrix<double, 6, 1>& reference_pose) const {
    return RotationBaseFromArmTip(current_pose)
        * RotationBaseFromArmTip(reference_pose).transpose();
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

RMResult StopAndConfirmStationary(
    RMCommand& command,
    RMStateReader& state_reader,
    const CalibratedFrameChain& frame_chain,
    std::chrono::milliseconds timeout) {
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
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(
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
            (frame_chain.ProbeTcpBase(current.pose)
             - frame_chain.ProbeTcpBase(previous.pose)).norm()
            * 1000.0;
        const double window_tcp_mm =
            (frame_chain.ProbeTcpBase(current.pose)
             - frame_chain.ProbeTcpBase(window_anchor.pose)).norm()
            * 1000.0;
        const double adjacent_orientation_deg =
            Eigen::AngleAxisd(frame_chain.ArmTipOrientationDelta(
                current.pose, previous.pose)).angle()
            * 180.0 / M_PI;
        const double window_orientation_deg =
            Eigen::AngleAxisd(frame_chain.ArmTipOrientationDelta(
                current.pose, window_anchor.pose)).angle()
            * 180.0 / M_PI;
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
