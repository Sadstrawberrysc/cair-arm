#include <calibrated_frame_chain.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <openssl/sha.h>
#include <json.hpp>

namespace {

double WrappedAngleDifference(double lhs, double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

}  // namespace

Eigen::Matrix4d CalibratedFrameChain::LoadCameraTransform(
    const std::string& path, const std::string& name, const std::string& key,
    std::string& sha256, bool require_verified) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("cannot read camera calibration: " + path);
    const std::string bytes((std::istreambuf_iterator<char>(file)), {});
    const auto data = nlohmann::json::parse(bytes);
    if (require_verified && !data.value("independent_validation_recorded", false))
        throw std::runtime_error("wrist execute requires independently validated camera calibration");
    if (data.at("schema_version") != 1 || data.at("translation_unit") != "m"
        || data.at("transform") != name
        || data.at("transform_convention") != "T_A_B maps coordinates from frame B to frame A")
        throw std::runtime_error("camera calibration schema/frame mismatch");
    Eigen::Matrix4d t;
    const auto& rows = data.at(key);
    if (!rows.is_array() || rows.size() != 4) throw std::runtime_error("camera matrix shape");
    for (int i=0; i<4; ++i) {
        if (!rows[i].is_array() || rows[i].size()!=4) throw std::runtime_error("camera matrix shape");
        for (int j=0; j<4; ++j) t(i,j)=rows[i][j].get<double>();
    }
    const Eigen::Matrix3d r=t.topLeftCorner<3,3>();
    if (!t.allFinite() || (t.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-8
        || (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()>2e-6
        || std::abs(r.determinant()-1)>2e-6)
        throw std::runtime_error("invalid camera rigid transform");
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), digest);
    std::ostringstream hex;
    for (auto byte : digest) hex << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
    sha256=hex.str();
    return t;
}

Eigen::Matrix4d CalibratedFrameChain::CameraPoseBase(
    const Eigen::Matrix<double,6,1>& pose, const Eigen::Matrix4d& arm_tip_camera) const {
    Eigen::Matrix4d base_arm=Eigen::Matrix4d::Identity();
    base_arm.topLeftCorner<3,3>()=RotationBaseFromArmTip(pose);
    base_arm.topRightCorner<3,1>()=pose.head<3>();
    return base_arm*arm_tip_camera;
}

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

Eigen::Matrix4d CalibratedFrameChain::ToolTcpPoseBase(const Eigen::Matrix<double,6,1>& pose) const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.topLeftCorner<3,3>() = RotationBaseFromTool(pose);
    result.topRightCorner<3,1>() = ProbeTcpBase(pose);
    return result;
}
Eigen::Matrix<double,6,1> CalibratedFrameChain::ArmTipPoseFromTcp(const Eigen::Matrix4d& tcp) const {
    const Eigen::Matrix3d r = tcp.topLeftCorner<3,3>() * rotation_arm_tip_from_tool_.transpose();
    Eigen::Matrix<double,6,1> result;
    result.head<3>() = tcp.topRightCorner<3,1>() - r*probe_tcp_arm_tip_m_;
    const double y = std::asin(std::clamp(-r(2,0), -1., 1.));
    double x = 0., z;
    if (std::abs(std::cos(y)) > 1e-9) { x=std::atan2(r(2,1),r(2,2)); z=std::atan2(r(1,0),r(0,0)); }
    else z=std::atan2(-r(0,1),r(1,1));
    result.tail<3>() << x,y,z;
    return result;
}
bool CalibratedFrameChain::InterpolateCamera(const std::deque<CameraSample>& history,
    std::int64_t time_ns, Eigen::Matrix4d& pose, double& span_ms) {
    for (std::size_t i=1; i<history.size(); ++i) {
        const auto& a=history[i-1]; const auto& b=history[i];
        const auto span=b.time_ns-a.time_ns;
        if (span<=0 || span>50000000 || time_ns<a.time_ns || time_ns>b.time_ns) continue;
        const double t=double(time_ns-a.time_ns)/double(span);
        pose=Eigen::Matrix4d::Identity();
        pose.topRightCorner<3,1>()=(1-t)*a.pose.topRightCorner<3,1>()+t*b.pose.topRightCorner<3,1>();
        const Eigen::Quaterniond qa(a.pose.topLeftCorner<3,3>()), qb(b.pose.topLeftCorner<3,3>());
        pose.topLeftCorner<3,3>()=qa.slerp(t,qb).normalized().toRotationMatrix();
        span_ms=span/1e6; return true;
    }
    return false;
}

void CalibratedFrameChain::CameraObservationBase(const Eigen::Matrix4d& camera_pose,
    const Eigen::Vector3d& point_camera, const Eigen::Vector3d& normal_camera,
    Eigen::Vector3d& point_base, Eigen::Vector3d& normal_base) {
    point_base = camera_pose.topLeftCorner<3,3>()*point_camera + camera_pose.topRightCorner<3,1>();
    normal_base = camera_pose.topLeftCorner<3,3>()*normal_camera;
}
