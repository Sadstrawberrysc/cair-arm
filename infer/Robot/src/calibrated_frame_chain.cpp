#include <calibrated_frame_chain.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <thread>
#include <vector>

namespace {

double WrappedAngleDifference(double lhs, double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

Eigen::Matrix<double, 6, 1> WrenchArrayToEigen(
    const std::array<double, 6>& input) {
    Eigen::Matrix<double, 6, 1> output;
    for (int index = 0; index < 6; ++index) {
        output[index] = input[static_cast<std::size_t>(index)];
    }
    return output;
}

bool RawWrenchWithinLimits(const Eigen::Matrix<double, 6, 1>& wrench,
                           double force_limit_n,
                           double torque_limit_nm) {
    return wrench.array().isFinite().all()
        && wrench.head<3>().cwiseAbs().maxCoeff() <= force_limit_n
        && wrench.tail<3>().cwiseAbs().maxCoeff() <= torque_limit_nm;
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

RuntimeTareResult CollectRuntimeTare(
    ForceSensorReader& force_reader,
    RMStateReader& state_reader,
    const ForceCalibration& calibration,
    const CalibratedFrameChain& frame_chain,
    const std::atomic<bool>& stop_requested,
    int seconds,
    double raw_force_limit_n,
    double raw_torque_limit_nm,
    const RuntimeTareConfig& config) {
    RuntimeTareResult result;
    std::vector<Eigen::Matrix<double, 6, 1>> samples;
    samples.reserve(static_cast<std::size_t>(seconds * 50));
    std::uint64_t last_sensor_sequence = 0;
    RobotStateSnapshot anchor;
    bool have_anchor = false;
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::seconds(seconds);

    while (!stop_requested.load()
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
            WrenchArrayToEigen(sample.wrench_si);
        if (!RawWrenchWithinLimits(raw,
                                   raw_force_limit_n,
                                   raw_torque_limit_nm)) {
            result.error = "runtime tare rejected raw sensor overrange";
            return result;
        }
        const CompensatedWrench compensated = calibration.Compensate(
            raw, frame_chain.RotationBaseFromTool(robot.pose));
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
            (frame_chain.ProbeTcpBase(robot.pose)
             - frame_chain.ProbeTcpBase(anchor.pose)).norm()
                * 1000.0);
        result.maximum_orientation_span_deg = std::max(
            result.maximum_orientation_span_deg,
            Eigen::AngleAxisd(frame_chain.ArmTipOrientationDelta(
                robot.pose, anchor.pose)).angle()
                * 180.0 / M_PI);
        result.maximum_force_norm_n = std::max(
            result.maximum_force_norm_n,
            compensated.sensor.head<3>().norm());
        result.maximum_torque_norm_nm = std::max(
            result.maximum_torque_norm_nm,
            compensated.sensor.tail<3>().norm());
        samples.push_back(compensated.sensor);
    }

    result.samples = samples.size();
    const std::size_t minimum_samples = static_cast<std::size_t>(
        seconds * config.minimum_samples_per_second);
    if (stop_requested.load()) {
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
    if (result.maximum_joint_span_deg > config.maximum_joint_span_deg
        || result.maximum_tcp_span_mm > config.maximum_tcp_span_mm
        || result.maximum_orientation_span_deg
               > config.maximum_orientation_span_deg) {
        std::ostringstream message;
        message << "runtime tare requires a stationary robot"
                << " (joint_span_deg=" << result.maximum_joint_span_deg
                << ", limit=" << config.maximum_joint_span_deg
                << "; tcp_span_mm=" << result.maximum_tcp_span_mm
                << ", limit=" << config.maximum_tcp_span_mm
                << "; orientation_span_deg="
                << result.maximum_orientation_span_deg
                << ", limit=" << config.maximum_orientation_span_deg << ')';
        result.error = message.str();
        return result;
    }
    if (result.maximum_force_deviation_n
            > config.maximum_force_deviation_n
        || result.maximum_torque_deviation_nm
               > config.maximum_torque_deviation_nm) {
        std::ostringstream message;
        message << "runtime tare residual wrench is not stable"
                << " (force_deviation_n="
                << result.maximum_force_deviation_n
                << ", limit=" << config.maximum_force_deviation_n
                << "; torque_deviation_nm="
                << result.maximum_torque_deviation_nm
                << ", limit=" << config.maximum_torque_deviation_nm << ')';
        result.error = message.str();
        return result;
    }
    result.valid = true;
    return result;
}

void ApplyRuntimeTare(const Eigen::Matrix<double, 6, 1>& sensor_offset,
                      CompensatedWrench& compensated) {
    if (!compensated.valid) return;
    compensated.sensor -= sensor_offset;
}
