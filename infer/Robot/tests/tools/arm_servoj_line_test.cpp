#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <realman_command.hpp>
#include <realman_kinematics.hpp>

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kEulerSingularityEps = 1e-9;
constexpr double kQuinticSmoothPeakVelocityScale = 1.875;
constexpr double kControlFeedbackBlend = 0;
constexpr double kAsyncJointSpeedScale = 0.5;
constexpr std::chrono::milliseconds kMinimumAsyncServoSendGap{10};
std::atomic<bool> g_stop_requested{false};

void HandleSignal(int) {
    g_stop_requested.store(true);
}
const Eigen::Matrix<double, 7, 1> kRm75JointMaxSpeedDegS =
    (Eigen::Matrix<double, 7, 1>() << 180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0).finished();

struct Options {
    std::string ip = "192.168.50.254";
    int port = 8080;
    Eigen::Vector3d delta_cm = (Eigen::Vector3d() << 2.0, 0.0, 0.0).finished();
    Eigen::Vector3d delta_rotation_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_position_cm = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_rotation_deg = Eigen::Vector3d::Zero();
    bool use_delta = false;
    bool use_delta_rotation = false;
    bool use_target_position = false;
    bool use_target_rotation = false;
    double max_tcp_speed_cm_s = 3.0;
    int final_hold_ms = 2000;
    int period_ms = 20;
    int feedback_every = 2;
    bool follow = false;
    bool use_s_curve = false;
    double max_total_delta_cm = 100.0;
    double max_rotation_delta_deg = 180.0;
    double max_joint_step_deg = 1.0;
    bool use_official_joint_max_speed = true;
    double max_joint_accel_deg_s2 = 90.0;
    double joint_limit_warning_deg = 10.0;
    double joint_limit_stop_deg = 3.0;
    double damping = 0.001;
    double singularity_warning_deg = 5.0;
    std::string trajectory_log_path;
    int log_every = 1;
    bool allow_near_singularity = false;
    bool summary_only = false;
    bool execute = false;
};

std::string CurrentTimestampForFilename() {
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&now, &local_time);

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return stream.str();
}

std::string DefaultTrajectoryLogPath() {
    const std::string timestamp = CurrentTimestampForFilename();
    const std::filesystem::path run_dir =
        std::filesystem::current_path() / "logs"
        / ("rm75_servoj_diagnostic_" + timestamp);
    return (run_dir / "trajectory.csv").string();
}

void PrintUsage(const char* program) {
    std::cout
        << "Usage: " << program << " [--ip A.B.C.D] [--port PORT]\n"
        << "       [--delta-cm X,Y,Z]\n"
        << "       [--delta-rotation-deg RX,RY,RZ]\n"
        << "       [--target-position-cm X,Y,Z]\n"
        << "       [--target-rotation-deg RX,RY,RZ]\n"
        << "       [--max-tcp-speed-cm-s CM_PER_S]\n"
        << "       [--final-hold-ms MS]\n"
        << "       [--period-ms MS]\n"
        << "       [--feedback-every N] [--follow]\n"
        << "       [--interpolation linear|s-curve]\n"
        << "       [--max-total-delta-cm CM] [--max-rotation-delta-deg DEG]\n"
        << "       [--max-joint-step-deg DEG] [--max-joint-accel-deg-s2 DEG_PER_S2]\n"
        << "       [--joint-limit-warning-deg DEG] [--joint-limit-stop-deg DEG]\n"
        << "       [--damping L] [--singularity-warning-deg DEG]\n"
        << "       [--trajectory-log PATH] [--log-every N]\n"
        << "       [--allow-near-singularity] [--summary-only] [--execute]\n\n"
        << "RM75 fixed-period streaming ServoJ pose test.\n"
        << "Use --delta-cm for relative position-only motion.\n"
        << "Use --delta-rotation-deg for relative rotation-only or combined relative motion.\n"
        << "Use --target-position-cm/--target-rotation-deg for absolute 6D pose control.\n"
        << "The tool uses fixed-period streaming ServoJ only.\n"
        << "Default interpolation is linear, matching the stable ServoJ baseline.\n"
        << "Use --interpolation s-curve only for tuning.\n"
        << "Joint step is capped at 50% of official RM75 max speed; --max-joint-step-deg may lower it.\n"
        << "Joint acceleration defaults to a conservative 90 deg/s^2 envelope.\n"
        << "Duration is estimated automatically from joint and TCP speed limits.\n"
        << "During --execute, trajectory CSV/SVG are saved automatically under ./logs.\n"
        << "Use --trajectory-log during --execute to choose a custom CSV path.\n"
        << "Default max total delta is 100 cm.\n"
        << "Omitted absolute target fields use the current pose. Default mode is dry-run.\n";
}

bool ParseInt(const char* text, int& value) {
    try {
        size_t parsed = 0;
        int parsed_value = std::stoi(text, &parsed, 10);
        if (parsed != std::strlen(text) || !std::isfinite(parsed_value)) return false;
        value = parsed_value;
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseDouble(const char* text, double& value) {
    try {
        size_t parsed = 0;
        double parsed_value = std::stod(text, &parsed);
        if (parsed != std::strlen(text) || !std::isfinite(parsed_value)) return false;
        value = parsed_value;
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseVector3(const char* text, Eigen::Vector3d& values, const char* label) {
    std::string input = text;
    std::vector<double> parsed_values;
    size_t start = 0;
    while (start <= input.size()) {
        size_t comma = input.find(',', start);
        std::string token = input.substr(start, comma == std::string::npos
                                                  ? std::string::npos
                                                  : comma - start);
        double parsed = 0.0;
        if (!ParseDouble(token.c_str(), parsed)) return false;
        parsed_values.push_back(parsed);
        if (comma == std::string::npos) break;
        start = comma + 1;
    }

    if (parsed_values.size() != 3) {
        std::cerr << label << " must have exactly 3 values.\n";
        return false;
    }
    values << parsed_values[0], parsed_values[1], parsed_values[2];
    return true;
}

bool ParseOptions(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }

        auto need_value = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--ip") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.ip = value;
        } else if (arg == "--port") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.port)) return false;
        } else if (arg == "--delta-cm") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseVector3(value, options.delta_cm, "delta-cm")) return false;
            options.use_delta = true;
        } else if (arg == "--delta-rotation-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseVector3(value, options.delta_rotation_deg, "delta-rotation-deg")) return false;
            options.use_delta_rotation = true;
        } else if (arg == "--target-position-cm") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseVector3(value, options.target_position_cm, "target-position-cm")) return false;
            options.use_target_position = true;
        } else if (arg == "--target-rotation-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseVector3(value, options.target_rotation_deg, "target-rotation-deg")) return false;
            options.use_target_rotation = true;
        } else if (arg == "--max-tcp-speed-cm-s") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.max_tcp_speed_cm_s)) return false;
        } else if (arg == "--final-hold-ms") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.final_hold_ms)) return false;
        } else if (arg == "--period-ms") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.period_ms)) return false;
        } else if (arg == "--feedback-every") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.feedback_every)) return false;
        } else if (arg == "--follow") {
            options.follow = true;
        } else if (arg == "--interpolation") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            std::string profile = value;
            if (profile == "linear") {
                options.use_s_curve = false;
            } else if (profile == "s-curve" || profile == "s_curve" || profile == "scurve") {
                options.use_s_curve = true;
            } else {
                std::cerr << "interpolation must be linear or s-curve\n";
                return false;
            }
        } else if (arg == "--max-total-delta-cm") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.max_total_delta_cm)) return false;
        } else if (arg == "--max-rotation-delta-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.max_rotation_delta_deg)) return false;
        } else if (arg == "--max-joint-step-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.max_joint_step_deg)) return false;
            options.use_official_joint_max_speed = false;
        } else if (arg == "--max-joint-accel-deg-s2") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.max_joint_accel_deg_s2)) return false;
        } else if (arg == "--joint-limit-warning-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.joint_limit_warning_deg)) return false;
        } else if (arg == "--joint-limit-stop-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.joint_limit_stop_deg)) return false;
        } else if (arg == "--damping") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.damping)) return false;
        } else if (arg == "--singularity-warning-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.singularity_warning_deg)) return false;
        } else if (arg == "--trajectory-log") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.trajectory_log_path = value;
        } else if (arg == "--log-every") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.log_every)) return false;
        } else if (arg == "--allow-near-singularity") {
            options.allow_near_singularity = true;
        } else if (arg == "--summary-only") {
            options.summary_only = true;
        } else if (arg == "--execute") {
            options.execute = true;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return false;
        }
    }

    if (options.ip.size() > RMConnectionConfig::kMaximumIpv4TextLength) {
        std::cerr << "IP address is too long.\n";
        return false;
    }
    if (options.port <= 0 || options.port > 65535) {
        std::cerr << "Invalid port.\n";
        return false;
    }
    if ((options.use_target_position || options.use_target_rotation)
        && (options.use_delta || options.use_delta_rotation)) {
        std::cerr << "Use either relative delta options or absolute target pose options, not both.\n";
        return false;
    }
    if (options.final_hold_ms < 0 || options.final_hold_ms > 10000) {
        std::cerr << "final-hold-ms must be in 0..10000\n";
        return false;
    }
    if (options.period_ms < 20 || options.period_ms > 200) {
        std::cerr << "period-ms must be in 20..200\n";
        return false;
    }
    if (options.execute && options.period_ms != 20) {
        std::cerr << "--execute requires period-ms=20\n";
        return false;
    }
    if (options.feedback_every < 0 || options.feedback_every > 100) {
        std::cerr << "feedback-every must be in 0..100\n";
        return false;
    }
    if (options.max_total_delta_cm <= 0.0 || options.max_total_delta_cm > 100.0) {
        std::cerr << "max-total-delta-cm must be in 0..100\n";
        return false;
    }
    if (options.use_delta
        && (options.delta_cm.norm() <= 0.0 || options.delta_cm.norm() > options.max_total_delta_cm)) {
        std::cerr << "delta-cm norm must be non-zero and within max-total-delta-cm.\n";
        return false;
    }
    if (options.max_rotation_delta_deg <= 0.0 || options.max_rotation_delta_deg > 180.0) {
        std::cerr << "max-rotation-delta-deg must be in 0..180\n";
        return false;
    }
    if (options.use_delta_rotation && !options.use_delta && options.delta_rotation_deg.norm() <= 0.0) {
        std::cerr << "delta-rotation-deg must be non-zero for rotation-only relative motion.\n";
        return false;
    }
    if (options.max_tcp_speed_cm_s <= 0.0 || options.max_tcp_speed_cm_s > 100.0) {
        std::cerr << "max-tcp-speed-cm-s must be in 0..100\n";
        return false;
    }
    if (!options.use_official_joint_max_speed) {
        if (options.max_joint_step_deg <= 0.0 || options.max_joint_step_deg > 10.0) {
            std::cerr << "max-joint-step-deg must be in 0..10\n";
            return false;
        }
    }
    if (options.max_joint_accel_deg_s2 < 0.0 || options.max_joint_accel_deg_s2 > 90.0) {
        std::cerr << "max-joint-accel-deg-s2 must be in 0..90; 0 is dry-run only\n";
        return false;
    }
    if (options.execute && options.max_joint_accel_deg_s2 <= 0.0) {
        std::cerr << "--execute requires a positive joint acceleration limit\n";
        return false;
    }
    if (options.joint_limit_warning_deg < 0.0 || options.joint_limit_warning_deg > 60.0) {
        std::cerr << "joint-limit-warning-deg must be in 0..60\n";
        return false;
    }
    if (options.joint_limit_stop_deg < 0.0 || options.joint_limit_stop_deg > 60.0) {
        std::cerr << "joint-limit-stop-deg must be in 0..60\n";
        return false;
    }
    if (options.joint_limit_stop_deg > options.joint_limit_warning_deg) {
        std::cerr << "joint-limit-stop-deg must be no larger than joint-limit-warning-deg\n";
        return false;
    }
    if (options.damping <= 0.0 || options.damping > 0.1) {
        std::cerr << "damping must be in 0..0.1\n";
        return false;
    }
    if (options.singularity_warning_deg < 0.0 || options.singularity_warning_deg > 30.0) {
        std::cerr << "singularity-warning-deg must be in 0..30\n";
        return false;
    }
    if (options.log_every <= 0 || options.log_every > 1000) {
        std::cerr << "log-every must be in 1..1000\n";
        return false;
    }
    return true;
}

double AngleDelta(double lhs, double rhs) {
    double delta = lhs - rhs;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    return delta;
}

Eigen::Vector3d MatrixToControllerEuler(const Eigen::Matrix3d& rotation) {
    const double sy = std::max(-1.0, std::min(1.0, -rotation(2, 0)));
    const double ry = std::asin(sy);
    const double cy = std::cos(ry);

    double rx = 0.0;
    double rz = 0.0;
    if (std::abs(cy) > kEulerSingularityEps) {
        rx = std::atan2(rotation(2, 1), rotation(2, 2));
        rz = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        rx = 0.0;
        rz = std::atan2(-rotation(0, 1), rotation(1, 1));
    }

    Eigen::Vector3d euler;
    euler << rx, ry, rz;
    return euler;
}

Eigen::Matrix<double, 6, 1> ControllerPoseFromJoints(
    RMKinematics& kinematics,
    const Eigen::Matrix<double, 7, 1>& joints) {
    Eigen::Matrix<double, 7, 1> mutable_joints = joints;
    Eigen::Matrix4d transform;
    kinematics.GetKinematics(transform, mutable_joints);

    Eigen::Matrix<double, 6, 1> pose;
    pose.head<3>() = transform.block<3,1>(0,3);
    pose.tail<3>() = MatrixToControllerEuler(transform.block<3,3>(0,0));
    return pose;
}

Eigen::Matrix<double, 6, 1> PoseError(const Eigen::Matrix<double, 6, 1>& target,
                                      const Eigen::Matrix<double, 6, 1>& current) {
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target.head<3>() - current.head<3>();
    for (int axis = 0; axis < 3; ++axis) {
        error[3 + axis] = AngleDelta(target[3 + axis], current[3 + axis]);
    }
    return error;
}

double SmoothPhase(double phase) {
    phase = std::max(0.0, std::min(1.0, phase));
    return phase * phase * phase * (10.0 + phase * (-15.0 + 6.0 * phase));
}

Eigen::Matrix<double, 6, 1> InterpolatePose(const Eigen::Matrix<double, 6, 1>& start,
                                            const Eigen::Matrix<double, 6, 1>& target,
                                            double phase,
                                            bool use_s_curve) {
    phase = std::max(0.0, std::min(1.0, phase));
    if (use_s_curve) {
        phase = SmoothPhase(phase);
    }
    Eigen::Matrix<double, 6, 1> pose = start;
    pose.head<3>() = start.head<3>() + phase * (target.head<3>() - start.head<3>());
    for (int axis = 0; axis < 3; ++axis) {
        pose[3 + axis] = start[3 + axis] + phase * AngleDelta(target[3 + axis], start[3 + axis]);
    }
    return pose;
}

Eigen::Matrix<double, 7, 1> LimitJointAcceleration(
    const Options& options,
    const Eigen::Matrix<double, 7, 1>& requested_delta,
    const Eigen::Matrix<double, 7, 1>& previous_delta) {
    if (options.max_joint_accel_deg_s2 <= 0.0) {
        return requested_delta;
    }

    const double period_s = static_cast<double>(options.period_ms) / 1000.0;
    const double max_delta_delta =
        options.max_joint_accel_deg_s2
        * kAsyncJointSpeedScale * kAsyncJointSpeedScale
        * kDegToRad * period_s * period_s;
    Eigen::Matrix<double, 7, 1> limited_delta = requested_delta;

    for (int joint = 0; joint < 7; ++joint) {
        const double delta_change = requested_delta[joint] - previous_delta[joint];
        if (delta_change > max_delta_delta) {
            limited_delta[joint] = previous_delta[joint] + max_delta_delta;
        } else if (delta_change < -max_delta_delta) {
            limited_delta[joint] = previous_delta[joint] - max_delta_delta;
        }
    }

    return limited_delta;
}

Eigen::Matrix<double, 6, 7> ComputeControllerPoseJacobian(
    RMKinematics& kinematics,
    const Eigen::Matrix<double, 7, 1>& joints) {
    constexpr double eps = 1e-6;
    Eigen::Matrix<double, 6, 7> jacobian;
    jacobian.setZero();

    for (int joint = 0; joint < 7; ++joint) {
        Eigen::Matrix<double, 7, 1> joints_plus = joints;
        Eigen::Matrix<double, 7, 1> joints_minus = joints;
        joints_plus[joint] += eps;
        joints_minus[joint] -= eps;

        const Eigen::Matrix<double, 6, 1> plus_pose =
            ControllerPoseFromJoints(kinematics, joints_plus);
        const Eigen::Matrix<double, 6, 1> minus_pose =
            ControllerPoseFromJoints(kinematics, joints_minus);

        jacobian.col(joint).head<3>() =
            (plus_pose.head<3>() - minus_pose.head<3>()) / (2.0 * eps);
        for (int axis = 0; axis < 3; ++axis) {
            jacobian(3 + axis, joint) =
                AngleDelta(plus_pose[3 + axis], minus_pose[3 + axis]) / (2.0 * eps);
        }
    }
    return jacobian;
}

bool NearDeg(double value_deg, double target_deg, double threshold_deg) {
    return std::fabs(value_deg - target_deg) < threshold_deg;
}

bool NearZeroDeg(double value_deg, double threshold_deg) {
    return NearDeg(value_deg, 0.0, threshold_deg);
}

bool NearPlusOrMinus90Deg(double value_deg, double threshold_deg) {
    return NearDeg(value_deg, 90.0, threshold_deg)
        || NearDeg(value_deg, -90.0, threshold_deg);
}

bool PrintSingularityWarning(const char* label,
                             const Eigen::Matrix<double, 7, 1>& joints,
                             double threshold_deg) {
    if (threshold_deg <= 0.0) return false;

    const double j2_deg = joints[1] * kRadToDeg;
    const double j3_deg = joints[2] * kRadToDeg;
    const double j4_deg = joints[3] * kRadToDeg;
    const double j5_deg = joints[4] * kRadToDeg;
    const double j6_deg = joints[5] * kRadToDeg;

    const bool type1 = NearZeroDeg(j2_deg, threshold_deg)
                    && NearZeroDeg(j6_deg, threshold_deg);
    const bool type2 = NearZeroDeg(j4_deg, threshold_deg);
    const bool type3 = NearZeroDeg(j2_deg, threshold_deg)
                    && NearPlusOrMinus90Deg(j3_deg, threshold_deg);
    const bool type4 = NearZeroDeg(j6_deg, threshold_deg)
                    && NearPlusOrMinus90Deg(j5_deg, threshold_deg);
    if (!type1 && !type2 && !type3 && !type4) return false;

    std::cout << "rm75_singularity_warning: " << label
              << " joints are near an official RM75 singular pattern.\n";
    return true;
}

bool WithinJointLimits(const RMKinematics& kinematics,
                       const Eigen::Matrix<double, 7, 1>& joints) {
    for (int i = 0; i < 7; ++i) {
        if (joints[i] < kinematics.JointMinimums()[i]
            || joints[i] > kinematics.JointMaximums()[i]) {
            std::cerr << "Target joint J" << (i + 1) << " exceeds RM75 joint limits.\n";
            return false;
        }
    }
    return true;
}

double JointLimitMarginDeg(const RMKinematics& kinematics,
                           const Eigen::Matrix<double, 7, 1>& joints,
                           int joint) {
    const double lower_margin =
        joints[joint] - kinematics.JointMinimums()[joint];
    const double upper_margin =
        kinematics.JointMaximums()[joint] - joints[joint];
    return std::min(lower_margin, upper_margin) * kRadToDeg;
}

bool CheckJointLimitMargins(const char* label,
                            const RMKinematics& kinematics,
                            const Eigen::Matrix<double, 7, 1>& joints,
                            double warning_deg,
                            double stop_deg,
                            bool print_warning,
                            bool* warning_printed) {
    for (int i = 0; i < 7; ++i) {
        const double margin_deg = JointLimitMarginDeg(kinematics, joints, i);
        if (stop_deg > 0.0 && margin_deg < stop_deg) {
            std::cerr << label << " joint J" << (i + 1)
                      << " is too close to RM75 joint limit: margin="
                      << margin_deg << " deg, stop_threshold=" << stop_deg << " deg.\n";
            return false;
        }
        const bool already_printed = warning_printed != nullptr && *warning_printed;
        if (print_warning
            && warning_deg > 0.0
            && margin_deg < warning_deg
            && !already_printed) {
            std::cout << "rm75_joint_limit_warning: " << label
                      << " joint J" << (i + 1)
                      << " is near limit, margin_deg=" << margin_deg
                      << ", warning_threshold_deg=" << warning_deg << "\n";
            if (warning_printed != nullptr) {
                *warning_printed = true;
            }
        }
    }
    return true;
}

bool PlanPoseStep(RMKinematics& kinematics,
                  const Options& options,
                  const Eigen::Matrix<double, 7, 1>& current_joints,
                  const Eigen::Matrix<double, 6, 1>& current_pose,
                  const Eigen::Matrix<double, 6, 1>& target_pose,
                  Eigen::Matrix<double, 7, 1>& target_joints,
                  bool print_joint_limit_warning = false,
                  bool* joint_limit_warning_printed = nullptr) {
    Eigen::Matrix<double, 6, 7> jacobian =
        ComputeControllerPoseJacobian(kinematics, current_joints);
    Eigen::Matrix<double, 6, 1> error = PoseError(target_pose, current_pose);
    Eigen::Matrix<double, 7, 6> damped_inverse =
        jacobian.transpose()
        * ((jacobian * jacobian.transpose()
            + options.damping * Eigen::Matrix<double, 6, 6>::Identity()).inverse());
    Eigen::Matrix<double, 7, 1> joint_delta = damped_inverse * error;

    const double period_s = static_cast<double>(options.period_ms) / 1000.0;
    const Eigen::Matrix<double, 7, 1> safe_official_step_rad =
        kAsyncJointSpeedScale
        * kRm75JointMaxSpeedDegS * period_s * kDegToRad;
    Eigen::Matrix<double, 7, 1> max_step_rad;
    if (options.use_official_joint_max_speed) {
        max_step_rad = safe_official_step_rad;
    } else {
        max_step_rad.setConstant(options.max_joint_step_deg * kDegToRad);
        max_step_rad = max_step_rad.cwiseMin(safe_official_step_rad);
    }

    double scale = 1.0;
    for (int i = 0; i < 7; ++i) {
        const double abs_step = std::fabs(joint_delta[i]);
        if (abs_step > max_step_rad[i]) {
            scale = std::min(scale, max_step_rad[i] / abs_step);
        }
    }
    if (scale < 1.0) {
        joint_delta *= scale;
    }

    target_joints = current_joints + joint_delta;
    if (!WithinJointLimits(kinematics, target_joints)) {
        return false;
    }
    return CheckJointLimitMargins("planned_target",
                                  kinematics,
                                  target_joints,
                                  options.joint_limit_warning_deg,
                                  options.joint_limit_stop_deg,
                                  print_joint_limit_warning,
                                  joint_limit_warning_printed);
}

bool SimulateStreamingCycles(RMKinematics& kinematics,
                             const Options& options,
                             const Eigen::Matrix<double, 7, 1>& start_joints,
                             const Eigen::Matrix<double, 6, 1>& start_pose,
                             const Eigen::Matrix<double, 6, 1>& target_pose,
                             int cycles,
                             double& final_position_error_cm,
                             double& final_rotation_error_deg) {
    Eigen::Matrix<double, 7, 1> model_joints = start_joints;
    Eigen::Matrix<double, 6, 1> model_pose = start_pose;
    Eigen::Matrix<double, 7, 1> previous_joint_delta;
    previous_joint_delta.setZero();

    for (int cycle = 1; cycle <= cycles; ++cycle) {
        const double phase = static_cast<double>(cycle) / static_cast<double>(cycles);
        Eigen::Matrix<double, 6, 1> desired_pose =
            InterpolatePose(start_pose, target_pose, phase, options.use_s_curve);

        Eigen::Matrix<double, 7, 1> target_joints;
        if (!PlanPoseStep(kinematics,
                          options,
                          model_joints,
                          model_pose,
                          desired_pose,
                          target_joints)) {
            return false;
        }
        Eigen::Matrix<double, 7, 1> requested_joint_delta = target_joints - model_joints;
        Eigen::Matrix<double, 7, 1> limited_joint_delta =
            LimitJointAcceleration(options, requested_joint_delta, previous_joint_delta);
        target_joints = model_joints + limited_joint_delta;
        if (!WithinJointLimits(kinematics, target_joints)) {
            return false;
        }
        model_joints = target_joints;
        previous_joint_delta = limited_joint_delta;
        model_pose = ControllerPoseFromJoints(kinematics, model_joints);
    }

    const Eigen::Matrix<double, 6, 1> error = PoseError(target_pose, model_pose);
    final_position_error_cm = error.head<3>().norm() * 100.0;
    final_rotation_error_deg = error.tail<3>().cwiseAbs().maxCoeff() * kRadToDeg;
    return true;
}

int EstimateFastestStreamingCycles(RMKinematics& kinematics,
                                   const Options& options,
                                   const Eigen::Matrix<double, 7, 1>& start_joints,
                                   const Eigen::Matrix<double, 6, 1>& start_pose,
                                   const Eigen::Matrix<double, 6, 1>& target_pose) {
    const int min_cycles = std::max(1, 200 / options.period_ms);
    const int max_cycles = std::max(min_cycles, 30000 / options.period_ms);
    constexpr double kPositionToleranceCm = 0.05;
    constexpr double kRotationToleranceDeg = 0.2;

    auto feasible = [&](int cycles) -> bool {
        double position_error_cm = 0.0;
        double rotation_error_deg = 0.0;
        if (!SimulateStreamingCycles(kinematics,
                                     options,
                                     start_joints,
                                     start_pose,
                                     target_pose,
                                     cycles,
                                     position_error_cm,
                                     rotation_error_deg)) {
            return false;
        }
        return position_error_cm <= kPositionToleranceCm
            && rotation_error_deg <= kRotationToleranceDeg;
    };

    int high = min_cycles;
    while (high < max_cycles && !feasible(high)) {
        high = std::min(max_cycles, high * 2);
    }
    if (!feasible(high)) return max_cycles;

    int low = min_cycles;
    while (low < high) {
        const int mid = low + (high - low) / 2;
        if (feasible(mid)) {
            high = mid;
        } else {
            low = mid + 1;
        }
    }
    return low;
}

double LateralErrorCm(const Eigen::Vector3d& start_m,
                      const Eigen::Vector3d& end_m,
                      const Eigen::Vector3d& point_m) {
    const Eigen::Vector3d line = end_m - start_m;
    const double line_norm2 = line.squaredNorm();
    if (line_norm2 <= std::numeric_limits<double>::epsilon()) {
        return (point_m - start_m).norm() * 100.0;
    }
    const double t = std::max(0.0, std::min(1.0, (point_m - start_m).dot(line) / line_norm2));
    const Eigen::Vector3d projection = start_m + t * line;
    return (point_m - projection).norm() * 100.0;
}

void PrintPose(const char* label, const Eigen::Matrix<double, 6, 1>& pose) {
    std::cout << label << "\n";
    std::cout << "position_cm: ["
              << pose[0] * 100.0 << ", "
              << pose[1] * 100.0 << ", "
              << pose[2] * 100.0 << "]\n";
    std::cout << "rotation_deg: ["
              << pose[3] * kRadToDeg << ", "
              << pose[4] * kRadToDeg << ", "
              << pose[5] * kRadToDeg << "]\n";
}

struct TrajectorySample {
    int t_ms = 0;
    Eigen::Vector3d desired_mm = Eigen::Vector3d::Zero();
    Eigen::Vector3d model_mm = Eigen::Vector3d::Zero();
    Eigen::Vector3d actual_mm = Eigen::Vector3d::Zero();
    double model_lateral_error_mm = 0.0;
    double actual_lateral_error_mm = 0.0;
    bool actual_valid = false;
};

std::vector<std::string> SplitCsvLine(const std::string& line) {
    std::vector<std::string> columns;
    size_t start = 0;
    while (start <= line.size()) {
        const size_t comma = line.find(',', start);
        columns.push_back(line.substr(start,
                                      comma == std::string::npos
                                          ? std::string::npos
                                          : comma - start));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return columns;
}

bool ParseCsvDouble(const std::string& text, double& value) {
    if (text.empty()) return false;
    try {
        size_t parsed = 0;
        const double parsed_value = std::stod(text, &parsed);
        if (parsed != text.size() || !std::isfinite(parsed_value)) return false;
        value = parsed_value;
        return true;
    } catch (...) {
        return false;
    }
}

bool ReadTrajectoryCsv(const std::string& csv_path,
                       std::vector<TrajectorySample>& samples) {
    std::ifstream stream(csv_path);
    if (!stream) {
        std::cerr << "Failed to read trajectory CSV: " << csv_path << "\n";
        return false;
    }

    std::string line;
    bool skipped_header = false;
    while (std::getline(stream, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (!skipped_header) {
            skipped_header = true;
            continue;
        }

        const std::vector<std::string> columns = SplitCsvLine(line);
        if (columns.size() < 17) continue;

        TrajectorySample sample;
        double parsed = 0.0;
        if (!ParseCsvDouble(columns[2], parsed)) continue;
        sample.t_ms = static_cast<int>(std::round(parsed));

        bool valid = true;
        for (int axis = 0; axis < 3; ++axis) {
            valid = valid && ParseCsvDouble(columns[4 + axis], parsed);
            sample.desired_mm[axis] = parsed * 10.0;
        }
        for (int axis = 0; axis < 3; ++axis) {
            valid = valid && ParseCsvDouble(columns[7 + axis], parsed);
            sample.model_mm[axis] = parsed * 10.0;
        }
        if (!valid) continue;

        sample.actual_valid = columns[10] == "1";
        if (sample.actual_valid) {
            for (int axis = 0; axis < 3; ++axis) {
                if (!ParseCsvDouble(columns[11 + axis], parsed)) {
                    sample.actual_valid = false;
                    break;
                }
                sample.actual_mm[axis] = parsed * 10.0;
            }
        }
        if (!ParseCsvDouble(columns[14], sample.actual_lateral_error_mm)) {
            sample.actual_lateral_error_mm = 0.0;
        } else {
            sample.actual_lateral_error_mm *= 10.0;
        }
        if (!ParseCsvDouble(columns[15], sample.model_lateral_error_mm)) {
            sample.model_lateral_error_mm = 0.0;
        } else {
            sample.model_lateral_error_mm *= 10.0;
        }
        samples.push_back(sample);
    }

    if (samples.empty()) {
        std::cerr << "Trajectory CSV contains no plottable samples: "
                  << csv_path << "\n";
        return false;
    }
    return true;
}

Eigen::Vector2d ProjectIsoMm(const Eigen::Vector3d& point_mm) {
    // Approximate matplotlib mplot3d's default view (elev=30, azim=-60)
    // for a static SVG projection.
    constexpr double kAzimuth = -60.0 * kDegToRad;
    constexpr double kElevation = 30.0 * kDegToRad;
    const double cos_az = std::cos(kAzimuth);
    const double sin_az = std::sin(kAzimuth);
    const double cos_el = std::cos(kElevation);
    const double sin_el = std::sin(kElevation);

    const double x_rot = cos_az * point_mm.x() - sin_az * point_mm.y();
    const double y_rot = sin_az * point_mm.x() + cos_az * point_mm.y();

    return Eigen::Vector2d(x_rot,
                           sin_el * y_rot - cos_el * point_mm.z());
}

struct PlotBounds {
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    void Include(const Eigen::Vector2d& point) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
    }
};

struct PlotBounds3d {
    Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

    void Include(const Eigen::Vector3d& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }
};

Eigen::Vector2d MapToBox(const Eigen::Vector2d& point,
                         const PlotBounds& bounds,
                         double x,
                         double y,
                         double width,
                         double height) {
    const double span_x = std::max(bounds.max_x - bounds.min_x, 1e-6);
    const double span_y = std::max(bounds.max_y - bounds.min_y, 1e-6);
    const double span = std::max(span_x, span_y);
    const double center_x = 0.5 * (bounds.min_x + bounds.max_x);
    const double center_y = 0.5 * (bounds.min_y + bounds.max_y);
    const double padded_span = span * 1.15;
    const double normalized_x = (point.x() - center_x) / padded_span + 0.5;
    const double normalized_y = (point.y() - center_y) / padded_span + 0.5;
    return Eigen::Vector2d(x + normalized_x * width,
                           y + (1.0 - normalized_y) * height);
}

std::string SvgPolyline(const std::vector<Eigen::Vector2d>& points,
                        const std::string& stroke,
                        double stroke_width,
                        const std::string& dash = "",
                        double opacity = 1.0) {
    if (points.empty()) return "";
    std::ostringstream stream;
    stream << "<polyline fill=\"none\" stroke=\"" << stroke
           << "\" stroke-width=\"" << stroke_width
           << "\" stroke-linecap=\"round\" stroke-linejoin=\"round"
           << "\" opacity=\"" << opacity << "\"";
    if (!dash.empty()) {
        stream << " stroke-dasharray=\"" << dash << "\"";
    }
    stream << " points=\"";
    for (const Eigen::Vector2d& point : points) {
        stream << point.x() << "," << point.y() << " ";
    }
    stream << "\" />\n";
    return stream.str();
}

std::string SvgLine(const Eigen::Vector2d& start,
                    const Eigen::Vector2d& end,
                    const std::string& stroke,
                    double stroke_width,
                    const std::string& dash = "",
                    double opacity = 1.0) {
    std::ostringstream stream;
    stream << "<line x1=\"" << start.x() << "\" y1=\"" << start.y()
           << "\" x2=\"" << end.x() << "\" y2=\"" << end.y()
           << "\" stroke=\"" << stroke
           << "\" stroke-width=\"" << stroke_width
           << "\" opacity=\"" << opacity << "\"";
    if (!dash.empty()) {
        stream << " stroke-dasharray=\"" << dash << "\"";
    }
    stream << " />\n";
    return stream.str();
}

std::string SvgPolygon(const std::vector<Eigen::Vector2d>& points,
                       const std::string& fill,
                       const std::string& stroke,
                       double stroke_width) {
    if (points.empty()) return "";
    std::ostringstream stream;
    stream << "<polygon fill=\"" << fill
           << "\" stroke=\"" << stroke
           << "\" stroke-width=\"" << stroke_width
           << "\" points=\"";
    for (const Eigen::Vector2d& point : points) {
        stream << point.x() << "," << point.y() << " ";
    }
    stream << "\" />\n";
    return stream.str();
}

std::string SvgText(double x,
                    double y,
                    const std::string& text,
                    int size = 14,
                    const std::string& anchor = "start") {
    std::ostringstream stream;
    stream << "<text x=\"" << x << "\" y=\"" << y
           << "\" font-size=\"" << size
           << "\" text-anchor=\"" << anchor
           << "\" font-family=\"Arial, sans-serif\">" << text << "</text>\n";
    return stream.str();
}

std::string FormatTick(double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(0) << value;
    return stream.str();
}

std::string FormatDeviationTick(double value_mm, double max_value_mm) {
    std::ostringstream stream;
    if (max_value_mm < 1.0) {
        stream << std::fixed << std::setprecision(3) << value_mm;
    } else if (max_value_mm < 10.0) {
        stream << std::fixed << std::setprecision(2) << value_mm;
    } else {
        stream << std::fixed << std::setprecision(1) << value_mm;
    }
    return stream.str();
}

bool WriteTrajectorySvg(const std::string& csv_path) {
    std::vector<TrajectorySample> samples;
    if (!ReadTrajectoryCsv(csv_path, samples)) return false;

    std::filesystem::path output_path(csv_path);
    output_path.replace_extension(".svg");
    std::ofstream svg(output_path);
    if (!svg) {
        std::cerr << "Failed to write trajectory SVG: "
                  << output_path.string() << "\n";
        return false;
    }

    constexpr double kWidth = 1400.0;
    constexpr double kHeight = 720.0;
    constexpr double kLeftX = 60.0;
    constexpr double kTopY = 85.0;
    constexpr double kPanelW = 610.0;
    constexpr double kPanelH = 520.0;
    constexpr double kRightX = 760.0;
    constexpr double kErrorW = 560.0;
    constexpr double kErrorH = 500.0;

    PlotBounds3d raw_3d_bounds;
    for (const TrajectorySample& sample : samples) {
        raw_3d_bounds.Include(sample.desired_mm);
        raw_3d_bounds.Include(sample.model_mm);
        if (sample.actual_valid) raw_3d_bounds.Include(sample.actual_mm);
    }

    const Eigen::Vector3d center_mm = 0.5 * (raw_3d_bounds.min + raw_3d_bounds.max);
    const Eigen::Vector3d range_mm = raw_3d_bounds.max - raw_3d_bounds.min;
    const double equal_span_mm =
        std::max({range_mm.x(), range_mm.y(), range_mm.z(), 1.0}) * 1.2;
    const Eigen::Vector3d cube_min =
        center_mm - Eigen::Vector3d::Constant(0.5 * equal_span_mm);
    const Eigen::Vector3d cube_max =
        center_mm + Eigen::Vector3d::Constant(0.5 * equal_span_mm);

    PlotBounds bounds;
    for (const double x : {cube_min.x(), cube_max.x()}) {
        for (const double y : {cube_min.y(), cube_max.y()}) {
            for (const double z : {cube_min.z(), cube_max.z()}) {
                bounds.Include(ProjectIsoMm((Eigen::Vector3d() << x, y, z).finished()));
            }
        }
    }

    std::vector<Eigen::Vector2d> desired_points;
    std::vector<Eigen::Vector2d> model_points;
    std::vector<Eigen::Vector2d> actual_points;
    std::vector<Eigen::Vector2d> model_jitter_points;
    std::vector<Eigen::Vector2d> actual_jitter_points;
    desired_points.reserve(samples.size());
    model_points.reserve(samples.size());

    double max_time_ms = 1.0;
    double max_jitter_mm = 1.0;
    for (const TrajectorySample& sample : samples) {
        max_time_ms = std::max(max_time_ms, static_cast<double>(sample.t_ms));
        max_jitter_mm = std::max(max_jitter_mm, sample.model_lateral_error_mm);
        if (sample.actual_valid) {
            max_jitter_mm = std::max(max_jitter_mm, sample.actual_lateral_error_mm);
        }
    }
    max_jitter_mm *= 1.08;

    auto map_3d = [&](const Eigen::Vector3d& point_mm) {
        return MapToBox(ProjectIsoMm(point_mm), bounds, kLeftX, kTopY, kPanelW, kPanelH);
    };
    auto map_jitter = [&](double t_ms, double jitter_mm) {
        const double x = kRightX + (t_ms / max_time_ms) * kErrorW;
        const double y = kTopY + (1.0 - jitter_mm / max_jitter_mm) * kErrorH;
        return Eigen::Vector2d(x, y);
    };

    for (const TrajectorySample& sample : samples) {
        desired_points.push_back(map_3d(sample.desired_mm));
        model_points.push_back(map_3d(sample.model_mm));
        if (sample.actual_valid) {
            actual_points.push_back(map_3d(sample.actual_mm));
        }
        model_jitter_points.push_back(
            map_jitter(sample.t_ms, sample.model_lateral_error_mm));
        if (sample.actual_valid) {
            actual_jitter_points.push_back(
                map_jitter(sample.t_ms, sample.actual_lateral_error_mm));
        }
    }

    svg << std::fixed << std::setprecision(3);
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << kWidth
        << "\" height=\"" << kHeight << "\" viewBox=\"0 0 " << kWidth
        << " " << kHeight << "\">\n";
    svg << "<style>\n"
        << ".plot-title{font-family:Arial,sans-serif;font-size:20px;fill:#111;}\n"
        << ".axis-label{font-family:Arial,sans-serif;font-size:14px;fill:#222;}\n"
        << ".tick-label{font-family:Arial,sans-serif;font-size:10px;fill:#333;}\n"
        << ".legend-text{font-family:Arial,sans-serif;font-size:14px;fill:#111;}\n"
        << "</style>\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"white\" />\n";
    svg << SvgText(kWidth / 2.0, 35.0, output_path.stem().string(), 24, "middle");
    svg << SvgText(kLeftX + kPanelW / 2.0, 70.0, "3D Trajectory (mm)", 20, "middle");
    svg << SvgText(kRightX + kErrorW / 2.0, 70.0, "Lateral Deviation", 20, "middle");

    constexpr int kGridTicks = 5;

    auto point_3d = [](double x, double y, double z) {
        return (Eigen::Vector3d() << x, y, z).finished();
    };
    auto draw_grid_line = [&](const Eigen::Vector3d& start,
                              const Eigen::Vector3d& end,
                              const std::string& color = "#d0d0d0") {
        svg << SvgLine(map_3d(start), map_3d(end), color, 0.8, "", 0.58);
    };

    const Eigen::Vector3d p000 = point_3d(cube_min.x(), cube_min.y(), cube_min.z());
    const Eigen::Vector3d p100 = point_3d(cube_max.x(), cube_min.y(), cube_min.z());
    const Eigen::Vector3d p010 = point_3d(cube_min.x(), cube_max.y(), cube_min.z());
    const Eigen::Vector3d p110 = point_3d(cube_max.x(), cube_max.y(), cube_min.z());
    const Eigen::Vector3d p001 = point_3d(cube_min.x(), cube_min.y(), cube_max.z());
    const Eigen::Vector3d p101 = point_3d(cube_max.x(), cube_min.y(), cube_max.z());
    const Eigen::Vector3d p011 = point_3d(cube_min.x(), cube_max.y(), cube_max.z());
    const Eigen::Vector3d p111 = point_3d(cube_max.x(), cube_max.y(), cube_max.z());

    svg << SvgPolygon({map_3d(p000), map_3d(p100), map_3d(p110), map_3d(p010)},
                      "#f6f6f6",
                      "#c8c8c8",
                      1.0);
    svg << SvgPolygon({map_3d(p000), map_3d(p100), map_3d(p101), map_3d(p001)},
                      "#fbfbfb",
                      "#c8c8c8",
                      1.0);
    svg << SvgPolygon({map_3d(p000), map_3d(p010), map_3d(p011), map_3d(p001)},
                      "#fbfbfb",
                      "#c8c8c8",
                      1.0);

    for (int tick = 0; tick <= kGridTicks; ++tick) {
        const double ratio = static_cast<double>(tick) / static_cast<double>(kGridTicks);
        const double x = cube_min.x() + ratio * (cube_max.x() - cube_min.x());
        const double y = cube_min.y() + ratio * (cube_max.y() - cube_min.y());
        const double z = cube_min.z() + ratio * (cube_max.z() - cube_min.z());

        draw_grid_line(point_3d(x, cube_min.y(), cube_min.z()),
                       point_3d(x, cube_max.y(), cube_min.z()),
                       "#d6d6d6");
        draw_grid_line(point_3d(cube_min.x(), y, cube_min.z()),
                       point_3d(cube_max.x(), y, cube_min.z()),
                       "#d6d6d6");

        draw_grid_line(point_3d(x, cube_min.y(), cube_min.z()),
                       point_3d(x, cube_min.y(), cube_max.z()),
                       "#dedede");
        draw_grid_line(point_3d(cube_min.x(), cube_min.y(), z),
                       point_3d(cube_max.x(), cube_min.y(), z),
                       "#dedede");

        draw_grid_line(point_3d(cube_min.x(), y, cube_min.z()),
                       point_3d(cube_min.x(), y, cube_max.z()),
                       "#dedede");
        draw_grid_line(point_3d(cube_min.x(), cube_min.y(), z),
                       point_3d(cube_min.x(), cube_max.y(), z),
                       "#dedede");

        if (tick > 0 && tick < kGridTicks) {
            const Eigen::Vector2d x_tick = map_3d(point_3d(x, cube_min.y(), cube_max.z()));
            const Eigen::Vector2d y_tick = map_3d(point_3d(cube_max.x(), y, cube_max.z()));
            const Eigen::Vector2d z_tick = map_3d(point_3d(cube_min.x(), cube_min.y(), z));
            svg << SvgText(x_tick.x(), x_tick.y() + 18.0, FormatTick(x), 10, "middle");
            svg << SvgText(y_tick.x() + 8.0, y_tick.y() + 16.0, FormatTick(y), 10, "start");
            svg << SvgText(z_tick.x() - 8.0, z_tick.y() + 4.0, FormatTick(z), 10, "end");
        }
    }

    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> frame_edges = {
        {p000, p100}, {p000, p010}, {p000, p001},
        {p100, p110}, {p010, p110}, {p100, p101},
        {p010, p011}, {p001, p101}, {p001, p011},
        {p111, p101}, {p111, p011}, {p111, p110},
    };
    for (const auto& edge : frame_edges) {
        svg << SvgLine(map_3d(edge.first), map_3d(edge.second), "#b7b7b7", 1.0, "", 0.88);
    }

    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> axis_edges = {
        {p001, p101}, {p101, p111}, {p000, p001},
    };
    for (const auto& edge : axis_edges) {
        svg << SvgLine(map_3d(edge.first), map_3d(edge.second), "#555555", 2.3);
    }

    const Eigen::Vector2d z_axis_mid = 0.5 * (map_3d(p000) + map_3d(p001));
    const Eigen::Vector2d x_axis_mid = 0.5 * (map_3d(p001) + map_3d(p101));
    const Eigen::Vector2d y_axis_mid = 0.5 * (map_3d(p101) + map_3d(p111));
    svg << SvgText(z_axis_mid.x() - 22.0,
                   z_axis_mid.y() + 4.0,
                   "Z (mm)",
                   13,
                   "end");
    svg << SvgText(x_axis_mid.x(),
                   x_axis_mid.y() + 48.0,
                   "X (mm)",
                   13,
                   "middle");
    svg << SvgText(y_axis_mid.x() + 36.0,
                   y_axis_mid.y() + 32.0,
                   "Y (mm)",
                   13,
                   "start");

    svg << SvgPolyline(desired_points, "#111111", 2.2, "8 6");
    svg << SvgPolyline(model_points, "#1f77b4", 2.8);
    svg << SvgPolyline(actual_points, "#d62728", 3.2);
    if (!actual_points.empty()) {
        const Eigen::Vector2d start_marker = actual_points.front();
        const Eigen::Vector2d end_marker = actual_points.back();
        svg << "<circle cx=\"" << start_marker.x() << "\" cy=\"" << start_marker.y()
            << "\" r=\"5\" fill=\"#2ca02c\" stroke=\"white\" stroke-width=\"1.5\" />\n";
        svg << "<circle cx=\"" << end_marker.x() << "\" cy=\"" << end_marker.y()
            << "\" r=\"5\" fill=\"#d62728\" stroke=\"white\" stroke-width=\"1.5\" />\n";
        svg << SvgText(start_marker.x() + 8.0, start_marker.y() - 8.0, "start", 12);
        svg << SvgText(end_marker.x() + 8.0, end_marker.y() - 8.0, "end", 12);
    }
    svg << "<rect x=\"" << kLeftX + kPanelW - 160.0 << "\" y=\"" << kTopY + 14.0
        << "\" width=\"140\" height=\"78\" rx=\"3\" ry=\"3\" fill=\"white\" "
        << "stroke=\"#d0d0d0\" opacity=\"0.94\" />\n";
    svg << SvgText(kLeftX + kPanelW - 145.0, kTopY + 37.0, "desired", 14);
    svg << "<line x1=\"" << kLeftX + kPanelW - 68.0 << "\" y1=\"" << kTopY + 32.0
        << "\" x2=\"" << kLeftX + kPanelW - 22.0 << "\" y2=\"" << kTopY + 32.0
        << "\" stroke=\"#111111\" stroke-width=\"2\" stroke-dasharray=\"8 6\" />\n";
    svg << SvgText(kLeftX + kPanelW - 145.0, kTopY + 61.0, "model", 14);
    svg << "<line x1=\"" << kLeftX + kPanelW - 68.0 << "\" y1=\"" << kTopY + 56.0
        << "\" x2=\"" << kLeftX + kPanelW - 22.0 << "\" y2=\"" << kTopY + 56.0
        << "\" stroke=\"#1f77b4\" stroke-width=\"2\" />\n";
    svg << SvgText(kLeftX + kPanelW - 145.0, kTopY + 85.0, "actual", 14);
    svg << "<line x1=\"" << kLeftX + kPanelW - 68.0 << "\" y1=\"" << kTopY + 80.0
        << "\" x2=\"" << kLeftX + kPanelW - 22.0 << "\" y2=\"" << kTopY + 80.0
        << "\" stroke=\"#d62728\" stroke-width=\"2.5\" />\n";

    svg << "<rect x=\"" << kRightX << "\" y=\"" << kTopY << "\" width=\""
        << kErrorW << "\" height=\"" << kErrorH
        << "\" fill=\"none\" stroke=\"#cccccc\" />\n";
    for (int tick = 0; tick <= 5; ++tick) {
        const double ratio = static_cast<double>(tick) / 5.0;
        const double y = kTopY + (1.0 - ratio) * kErrorH;
        const double error = ratio * max_jitter_mm;
        svg << "<line x1=\"" << kRightX << "\" y1=\"" << y
            << "\" x2=\"" << kRightX + kErrorW << "\" y2=\"" << y
            << "\" stroke=\"#eeeeee\" />\n";
        svg << SvgText(kRightX - 8.0,
                       y + 4.0,
                       FormatDeviationTick(error, max_jitter_mm),
                       12,
                       "end");
    }
    for (int tick = 0; tick <= 5; ++tick) {
        const double ratio = static_cast<double>(tick) / 5.0;
        const double x = kRightX + ratio * kErrorW;
        const double time_s = ratio * max_time_ms / 1000.0;
        svg << "<line x1=\"" << x << "\" y1=\"" << kTopY
            << "\" x2=\"" << x << "\" y2=\"" << kTopY + kErrorH
            << "\" stroke=\"#eeeeee\" />\n";
        std::ostringstream tick_label;
        tick_label << std::fixed << std::setprecision(1) << time_s;
        svg << SvgText(x, kTopY + kErrorH + 20.0, tick_label.str(), 12, "middle");
    }
    svg << SvgPolyline(model_jitter_points, "#1f77b4", 2.0);
    svg << SvgPolyline(actual_jitter_points, "#d62728", 2.5);
    svg << SvgText(kRightX + kErrorW / 2.0, kTopY + kErrorH + 42.0, "time (s)", 14, "middle");
    svg << SvgText(kRightX - 82.0, kTopY + kErrorH / 2.0, "lateral deviation (mm)", 14, "middle");
    svg << SvgText(kRightX + 20.0, kTopY + 28.0, "model", 14);
    svg << "<line x1=\"" << kRightX + 95.0 << "\" y1=\"" << kTopY + 23.0
        << "\" x2=\"" << kRightX + 145.0 << "\" y2=\"" << kTopY + 23.0
        << "\" stroke=\"#1f77b4\" stroke-width=\"2\" />\n";
    svg << SvgText(kRightX + 20.0, kTopY + 52.0, "actual", 14);
    svg << "<line x1=\"" << kRightX + 95.0 << "\" y1=\"" << kTopY + 47.0
        << "\" x2=\"" << kRightX + 145.0 << "\" y2=\"" << kTopY + 47.0
        << "\" stroke=\"#d62728\" stroke-width=\"2.5\" />\n";
    svg << "</svg>\n";

    std::cout << "trajectory_plot_written: " << output_path.string() << "\n";
    return true;
}

class TrajectoryLogger {
public:
    bool Open(const std::string& path) {
        if (path.empty()) return true;
        path_ = path;

        const std::filesystem::path output_path(path);
        const std::filesystem::path parent = output_path.parent_path();
        if (!parent.empty()) {
            std::error_code ec;
            std::filesystem::create_directories(parent, ec);
            if (ec) {
                std::cerr << "Failed to create trajectory log directory: "
                          << parent.string() << "\n";
                return false;
            }
        }

        stream_.open(path);
        if (!stream_) {
            std::cerr << "Failed to open trajectory log: " << path << "\n";
            return false;
        }

        stream_ << std::fixed << std::setprecision(6);
        stream_ << "stage,cycle,t_ms,phase,"
                << "desired_x_cm,desired_y_cm,desired_z_cm,"
                << "model_x_cm,model_y_cm,model_z_cm,"
                << "actual_valid,actual_x_cm,actual_y_cm,actual_z_cm,"
                << "actual_lateral_error_cm,model_lateral_error_cm,"
                << "j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg,j7_deg\n";
        return true;
    }

    bool Enabled() const {
        return stream_.is_open();
    }

    const std::string& Path() const {
        return path_;
    }

    void Close() {
        if (stream_.is_open()) {
            stream_.close();
        }
    }

    void Record(const std::string& stage,
                int cycle,
                int t_ms,
                double phase,
                const Eigen::Matrix<double, 6, 1>& desired_pose,
                const Eigen::Matrix<double, 6, 1>& model_pose,
                const Eigen::Matrix<double, 7, 1>& model_joints,
                bool actual_valid,
                const Eigen::Matrix<double, 6, 1>& actual_pose,
                const Eigen::Vector3d& start_position,
                const Eigen::Vector3d& target_position) {
        if (!Enabled()) return;

        const double model_lateral_cm =
            LateralErrorCm(start_position, target_position, model_pose.head<3>());
        const double actual_lateral_cm =
            actual_valid ? LateralErrorCm(start_position, target_position, actual_pose.head<3>())
                         : 0.0;

        stream_ << stage << ","
                << cycle << ","
                << t_ms << ","
                << phase << ","
                << desired_pose[0] * 100.0 << ","
                << desired_pose[1] * 100.0 << ","
                << desired_pose[2] * 100.0 << ","
                << model_pose[0] * 100.0 << ","
                << model_pose[1] * 100.0 << ","
                << model_pose[2] * 100.0 << ","
                << (actual_valid ? 1 : 0) << ",";
        if (actual_valid) {
            stream_ << actual_pose[0] * 100.0 << ","
                    << actual_pose[1] * 100.0 << ","
                    << actual_pose[2] * 100.0 << ",";
        } else {
            stream_ << ",,,";
        }
        stream_ << (actual_valid ? actual_lateral_cm : 0.0) << ","
                << model_lateral_cm;

        for (int joint = 0; joint < 7; ++joint) {
            stream_ << "," << model_joints[joint] * kRadToDeg;
        }
        stream_ << "\n";
    }

    void WriteSummary(double final_position_error_cm,
                      double final_rotation_error_deg,
                      double max_lateral_error_cm,
                      int missed_periods) {
        if (!Enabled()) return;
        stream_ << "# summary,final_position_error_cm,"
                << final_position_error_cm << "\n";
        stream_ << "# summary,final_rotation_error_deg,"
                << final_rotation_error_deg << "\n";
        stream_ << "# summary,max_lateral_error_cm,"
                << max_lateral_error_cm << "\n";
        stream_ << "# summary,missed_periods,"
                << missed_periods << "\n";
        stream_.flush();
    }

private:
    std::string path_;
    std::ofstream stream_;
};

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        PrintUsage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);
    if (options.execute && options.trajectory_log_path.empty()) {
        options.trajectory_log_path = DefaultTrajectoryLogPath();
    }

    RMCommand command(RMConnectionConfig{options.ip, options.port});
    command.SetQuiet(true);
    const RMResult connect_result = command.TryConnectTCPSocket();
    if (!connect_result) {
        std::cerr << "Failed to connect to the robot: "
                  << connect_result.message << "\n";
        return 3;
    }
    if (options.execute) {
        const RMResult startup_stop = command.TryStopMotion(1000);
        if (!startup_stop) {
            std::cerr << "Failed to stop inherited motion before state read: "
                      << startup_stop.message << "\n";
            return 6;
        }
    }

    Eigen::Matrix<double, 7, 1> current_joints;
    Eigen::Matrix<double, 6, 1> current_pose;
    int arm_err = 0;
    int sys_err = 0;
    const RMResult initial_state_result =
        command.TryReadArmState(current_joints,
                                current_pose,
                                arm_err,
                                sys_err);
    if (!initial_state_result) {
        std::cerr << "Failed to read the initial robot state: "
                  << initial_state_result.message << "\n";
        return 3;
    }
    if (arm_err != 0 || sys_err != 0) {
        std::cerr << "Robot reports an error. Refusing to plan motion.\n";
        return 3;
    }

    RMKinematics kinematics;
    PrintSingularityWarning("current", current_joints, options.singularity_warning_deg);
    CheckJointLimitMargins("current",
                           kinematics,
                           current_joints,
                           options.joint_limit_warning_deg,
                           0.0,
                           true,
                           nullptr);

    const Eigen::Matrix<double, 6, 1> start_pose = current_pose;
    Eigen::Matrix<double, 6, 1> target_pose = start_pose;
    if (options.use_target_position) {
        target_pose.head<3>() = options.target_position_cm / 100.0;
    } else if (options.use_delta) {
        target_pose.head<3>() += options.delta_cm / 100.0;
    } else if (!(options.use_target_rotation || options.use_delta_rotation)) {
        target_pose.head<3>() += options.delta_cm / 100.0;
    }
    if (options.use_target_rotation) {
        target_pose.tail<3>() = options.target_rotation_deg * kDegToRad;
    } else if (options.use_delta_rotation) {
        target_pose.tail<3>() += options.delta_rotation_deg * kDegToRad;
    }

    const Eigen::Matrix<double, 6, 1> requested_delta = PoseError(target_pose, start_pose);
    const double requested_position_delta_cm = requested_delta.head<3>().norm() * 100.0;
    const double requested_rotation_delta_deg =
        requested_delta.tail<3>().cwiseAbs().maxCoeff() * kRadToDeg;
    if (requested_position_delta_cm > options.max_total_delta_cm) {
        std::cerr << "target position delta exceeds max-total-delta-cm.\n";
        return 2;
    }
    if (requested_rotation_delta_deg > options.max_rotation_delta_deg) {
        std::cerr << "target rotation delta exceeds max-rotation-delta-deg.\n";
        return 2;
    }

    const int joint_limited_cycles = EstimateFastestStreamingCycles(kinematics,
                                                                    options,
                                                                    current_joints,
                                                                    current_pose,
                                                                    target_pose);
    const int tcp_limited_cycles =
        std::max(1,
                 static_cast<int>(std::ceil(requested_position_delta_cm
                                            * (options.use_s_curve
                                                   ? kQuinticSmoothPeakVelocityScale
                                                   : 1.0)
                                            / options.max_tcp_speed_cm_s
                                            * 1000.0
                                            / options.period_ms)));
    const int cycle_count = std::max(joint_limited_cycles, tcp_limited_cycles);
    const int duration_ms = cycle_count * options.period_ms;
    if (duration_ms < 200 || duration_ms > 30000) {
        std::cerr << "Estimated duration is outside 200..30000 ms.\n";
        return 2;
    }

    PrintPose("current_pose", start_pose);
    PrintPose("target_pose", target_pose);
    std::cout << "mode: streaming\n";
    if (options.use_target_position || options.use_target_rotation) {
        std::cout << "target_mode: absolute_pose\n";
        std::cout << "requested_position_delta_cm: " << requested_position_delta_cm << "\n";
        std::cout << "requested_rotation_delta_deg: " << requested_rotation_delta_deg << "\n";
    } else {
        std::cout << "target_mode: relative_delta\n";
        std::cout << "delta_cm: ["
                  << (options.use_delta ? options.delta_cm[0] : 0.0) << ", "
                  << (options.use_delta ? options.delta_cm[1] : 0.0) << ", "
                  << (options.use_delta ? options.delta_cm[2] : 0.0) << "]\n";
        std::cout << "delta_rotation_deg: ["
                  << (options.use_delta_rotation ? options.delta_rotation_deg[0] : 0.0) << ", "
                  << (options.use_delta_rotation ? options.delta_rotation_deg[1] : 0.0) << ", "
                  << (options.use_delta_rotation ? options.delta_rotation_deg[2] : 0.0) << "]\n";
    }
    std::cout << "max_total_delta_cm: " << options.max_total_delta_cm << "\n";
    std::cout << "max_rotation_delta_deg: " << options.max_rotation_delta_deg << "\n";
    std::cout << "interpolation_profile: "
              << (options.use_s_curve ? "quintic_s_curve" : "linear") << "\n";
    if (options.use_s_curve) {
        std::cout << "interpolation_peak_velocity_scale: "
                  << kQuinticSmoothPeakVelocityScale << "\n";
    }
    std::cout << "period_ms: " << options.period_ms << "\n";
    std::cout << "async_joint_speed_scale: " << kAsyncJointSpeedScale << "\n";
    std::cout << "minimum_servoj_send_gap_ms: "
              << kMinimumAsyncServoSendGap.count() << "\n";
    if (options.use_official_joint_max_speed) {
        std::cout << "joint_speed_limit: scaled_official_rm75_max\n";
        std::cout << "joint_max_speed_deg_s: [90, 90, 112.5, 112.5, 112.5, 112.5, 112.5]\n";
    } else {
        std::cout << "joint_speed_limit: manual_step_capped_by_scaled_official_max\n";
        std::cout << "max_joint_step_deg: " << options.max_joint_step_deg << "\n";
    }
    if (options.max_joint_accel_deg_s2 > 0.0) {
        std::cout << "max_joint_accel_deg_s2: " << options.max_joint_accel_deg_s2 << "\n";
    } else {
        std::cout << "max_joint_accel_deg_s2: disabled\n";
    }
    std::cout << "duration_ms: " << duration_ms << "\n";
    std::cout << "duration_mode: fastest_joint_and_tcp_limited\n";
    std::cout << "max_tcp_speed_cm_s: " << options.max_tcp_speed_cm_s << "\n";
    std::cout << "planned_main_speed_cm_s: "
              << (requested_position_delta_cm * 1000.0 / duration_ms) << "\n";
    std::cout << "planned_rotation_speed_deg_s: "
              << (requested_rotation_delta_deg * 1000.0 / duration_ms) << "\n";
    std::cout << "final_hold_ms: " << options.final_hold_ms << "\n";
    std::cout << "cycle_count: " << cycle_count << "\n";
    std::cout << "feedback_every: " << options.feedback_every << "\n";
    if (!options.trajectory_log_path.empty()) {
        std::cout << "trajectory_log: " << options.trajectory_log_path << "\n";
        std::cout << "log_every: " << options.log_every << "\n";
    }
    std::cout << "joint_limit_warning_deg: " << options.joint_limit_warning_deg << "\n";
    std::cout << "joint_limit_stop_deg: " << options.joint_limit_stop_deg << "\n";

    if (!options.execute) {
        std::cout << "\nDry-run only. No motion command was sent.\n";
        std::cout << "Add --execute after confirming the pose path is clear.\n";
        return 0;
    }

    TrajectoryLogger trajectory_logger;
    if (!trajectory_logger.Open(options.trajectory_log_path)) {
        return 2;
    }

    // One background owner receives every JSON response on this same TCP
    // connection. The 20 ms control path below only sends ServoJ and reads a
    // latest snapshot; it never performs a blocking ReadArmState call.
    const int state_poll_ms =
        options.feedback_every > 0
            ? std::max(40, options.feedback_every * options.period_ms)
            : 100;
    const int state_stale_ms = std::max(250, state_poll_ms * 4);
    RMStateReader state_reader(command,
                               std::chrono::milliseconds(state_poll_ms),
                               std::chrono::milliseconds(state_stale_ms));
    const RMResult state_reader_result = state_reader.Start();
    if (!state_reader_result) {
        std::cerr << "Failed to start the asynchronous robot state reader: "
                  << state_reader_result.message << "\n";
        return 6;
    }
    const std::uint64_t initial_reader_sequence =
        state_reader.Latest().sequence;
    RobotStateSnapshot primed_state;
    if (!state_reader.WaitForUpdate(initial_reader_sequence,
                                    std::chrono::milliseconds(1500),
                                    primed_state)
        || !primed_state.valid || primed_state.stale) {
        const RMResult reader_error = state_reader.LastResult();
        std::cerr << "Asynchronous robot feedback did not become ready before motion";
        if (!reader_error) std::cerr << ": " << reader_error.message;
        std::cerr << "\n";
        return 6;
    }
    if (primed_state.arm_err != 0 || primed_state.sys_err != 0) {
        std::cerr << "Asynchronous robot feedback reports an error before motion.\n";
        return 6;
    }
    std::uint64_t last_feedback_sequence = primed_state.sequence;
    auto stop_motion = [&]() {
        const RMResult stop_result = command.TryStopMotion(1000);
        if (!stop_result) {
            std::cerr << "StopMotion was not confirmed: "
                      << stop_result.message << "\n";
        }
        return stop_result;
    };
    if (trajectory_logger.Enabled()) {
        trajectory_logger.Record("start",
                                 0,
                                 0,
                                 0.0,
                                 start_pose,
                                 current_pose,
                                 current_joints,
                                 true,
                                 current_pose,
                                 start_pose.head<3>(),
                                 target_pose.head<3>());
    }

    Eigen::Matrix<double, 7, 1> model_joints = current_joints;
    Eigen::Matrix<double, 6, 1> model_pose = current_pose;
    double max_lateral_error_cm = 0.0;
    int missed_periods = 0;
    bool joint_limit_warning_printed = false;
    Eigen::Matrix<double, 7, 1> previous_joint_delta;
    previous_joint_delta.setZero();

    auto next_tick = std::chrono::steady_clock::now();
    auto wait_for_next_tick = [&]() {
        next_tick += std::chrono::milliseconds(options.period_ms);
        const auto now = std::chrono::steady_clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
        } else {
            ++missed_periods;
            next_tick = now;
        }
    };

    const int final_hold_cycles =
        std::max(0, options.final_hold_ms / options.period_ms);

    auto run_servo_cycle = [&](const char* stage,
                               int cycle,
                               int total_cycles,
                               int t_ms,
                               double phase,
                               const Eigen::Matrix<double, 6, 1>& desired_pose) -> int {
        if (g_stop_requested.load()) return 130;
        const bool is_hold = std::strcmp(stage, "hold") == 0;
        const char* target_label = is_hold ? "final_hold_target" : "planned_target";

        Eigen::Matrix<double, 7, 1> target_joints;
        if (!PlanPoseStep(kinematics,
                          options,
                          model_joints,
                          model_pose,
                          desired_pose,
                          target_joints,
                          true,
                          &joint_limit_warning_printed)) {
            return 4;
        }
        Eigen::Matrix<double, 7, 1> requested_joint_delta = target_joints - model_joints;
        Eigen::Matrix<double, 7, 1> limited_joint_delta =
            LimitJointAcceleration(options, requested_joint_delta, previous_joint_delta);
        target_joints = model_joints + limited_joint_delta;
        if (!WithinJointLimits(kinematics, target_joints)) {
            return 4;
        }
        if (!CheckJointLimitMargins(target_label,
                                    kinematics,
                                    target_joints,
                                    options.joint_limit_warning_deg,
                                    options.joint_limit_stop_deg,
                                    true,
                                    &joint_limit_warning_printed)) {
            return 4;
        }

        const bool near_singularity =
            PrintSingularityWarning(target_label, target_joints, options.singularity_warning_deg);
        if (near_singularity && !options.allow_near_singularity) {
            std::cerr << (is_hold ? "Final hold target" : "Planned target")
                      << " is near an official RM75 singular pattern.\n";
            return 5;
        }

        // Reject stale or faulted feedback before sending the next target.
        // Holding uses the previous model target, which is the last target
        // accepted by the control loop.
        const RobotStateSnapshot latest_state = state_reader.Latest();
        const RMResult reader_health = state_reader.LastResult();
        const ServoSendSnapshot servo_status = command.ServoStatus();
        if (!reader_health) {
            std::cerr << (is_hold ? "ServoJ final hold" : "ServoJ line")
                      << " asynchronous I/O fault: "
                      << reader_health.message << "\n";
            return 6;
        }
        if (servo_status.result_sequence != 0 && !servo_status.result) {
            std::cerr << (is_hold ? "ServoJ final hold" : "ServoJ line")
                      << " asynchronous ServoJ failure: "
                      << servo_status.result.message << "\n";
            return 6;
        }
        if (servo_status.submitted_sequence
            > std::max(servo_status.sent_sequence,
                       servo_status.discarded_sequence)) {
            std::cerr << (is_hold ? "ServoJ final hold" : "ServoJ line")
                      << " previous target is still pending or in flight; "
                         "refusing to queue fixed-period targets.\n";
            return 6;
        }
        const auto servo_check_time = std::chrono::steady_clock::now();
        if (servo_status.sent_sequence != 0
            && servo_status.sent_at
                   != std::chrono::steady_clock::time_point{}
            && servo_check_time - servo_status.sent_at
                   < kMinimumAsyncServoSendGap) {
            std::cerr << (is_hold ? "ServoJ final hold" : "ServoJ line")
                      << " previous socket write was less than "
                      << kMinimumAsyncServoSendGap.count()
                      << " ms ago; refusing unsafe dispatch cadence.\n";
            return 6;
        }
        if (latest_state.stale) {
            std::cerr << (is_hold ? "ServoJ final hold" : "ServoJ line")
                      << " feedback is stale; holding the last safe target.\n";
            return 6;
        }
        if (latest_state.arm_err != 0 || latest_state.sys_err != 0) {
            std::cerr << (is_hold ? "ServoJ final hold" : "ServoJ line")
                      << " feedback reports robot error state.\n";
            return 6;
        }

        if (g_stop_requested.load()) return 130;
        const RMResult servo_result =
            command.TryServoJ(target_joints, options.follow);
        if (!servo_result) {
            std::cerr << (is_hold ? "ServoJ final hold" : "ServoJ line")
                      << " send failed: " << servo_result.message << "\n";
            return 6;
        }
        model_joints = target_joints;
        previous_joint_delta = limited_joint_delta;
        model_pose = ControllerPoseFromJoints(kinematics, model_joints);

        bool actual_valid_for_log = false;
        Eigen::Matrix<double, 6, 1> actual_pose_for_log = current_pose;
        if (options.feedback_every > 0 && cycle % options.feedback_every == 0) {
            if (latest_state.valid
                && latest_state.sequence > last_feedback_sequence) {
                current_joints = latest_state.joints;
                current_pose = latest_state.pose;
                arm_err = latest_state.arm_err;
                sys_err = latest_state.sys_err;
                last_feedback_sequence = latest_state.sequence;
                actual_valid_for_log = true;
                actual_pose_for_log = current_pose;
            }
            model_joints = (1.0 - kControlFeedbackBlend) * model_joints
                         + kControlFeedbackBlend * current_joints;
            model_pose = (1.0 - kControlFeedbackBlend) * model_pose
                       + kControlFeedbackBlend * current_pose;

            if (actual_valid_for_log) {
                const double lateral_cm = LateralErrorCm(start_pose.head<3>(),
                                                         target_pose.head<3>(),
                                                         current_pose.head<3>());
                max_lateral_error_cm = std::max(max_lateral_error_cm, lateral_cm);
                if (!options.summary_only
                    && cycle % (options.feedback_every * 5) == 0) {
                    std::cout << (is_hold ? "final_hold_cycle: " : "cycle: ")
                              << cycle << "/" << total_cycles
                              << " lateral_error_cm=" << lateral_cm << "\n";
                }
            }
        }

        if (trajectory_logger.Enabled()
            && (cycle % options.log_every == 0 || actual_valid_for_log)) {
            trajectory_logger.Record(stage,
                                     cycle,
                                     t_ms,
                                     phase,
                                     desired_pose,
                                     model_pose,
                                     model_joints,
                                     actual_valid_for_log,
                                     actual_pose_for_log,
                                     start_pose.head<3>(),
                                     target_pose.head<3>());
        }

        wait_for_next_tick();
        return 0;
    };

    for (int cycle = 1; cycle <= cycle_count; ++cycle) {
        const double phase = static_cast<double>(cycle) / static_cast<double>(cycle_count);
        const Eigen::Matrix<double, 6, 1> desired_pose =
            InterpolatePose(start_pose, target_pose, phase, options.use_s_curve);
        const int result = run_servo_cycle("move",
                                           cycle,
                                           cycle_count,
                                           cycle * options.period_ms,
                                           phase,
                                           desired_pose);
        if (result != 0) {
            (void)stop_motion();
            return result;
        }
    }

    for (int hold_cycle = 1; hold_cycle <= final_hold_cycles; ++hold_cycle) {
        const int result = run_servo_cycle("hold",
                                           hold_cycle,
                                           final_hold_cycles,
                                           duration_ms + hold_cycle * options.period_ms,
                                           1.0,
                                           target_pose);
        if (result != 0) {
            (void)stop_motion();
            return result;
        }
    }

    if (!stop_motion()) {
        return 6;
    }

    RobotStateSnapshot final_state;
    state_reader.WaitForUpdate(last_feedback_sequence,
                               std::chrono::milliseconds(
                                   std::max(500, state_poll_ms * 3)),
                               final_state);
    if (!final_state.valid) final_state = state_reader.Latest();
    state_reader.Stop();
    if (!final_state.valid || final_state.stale) {
        std::cerr << "ServoJ line finished without fresh robot feedback.\n";
        return 6;
    }
    current_joints = final_state.joints;
    current_pose = final_state.pose;
    arm_err = final_state.arm_err;
    sys_err = final_state.sys_err;
    if (arm_err != 0 || sys_err != 0) {
        std::cerr << "ServoJ line finished with robot error state.\n";
        return 6;
    }

    const Eigen::Matrix<double, 6, 1> final_error = PoseError(target_pose, current_pose);
    const double final_position_error_cm = final_error.head<3>().norm() * 100.0;
    const double final_rotation_error_deg =
        final_error.tail<3>().cwiseAbs().maxCoeff() * kRadToDeg;
    max_lateral_error_cm =
        std::max(max_lateral_error_cm,
                 LateralErrorCm(start_pose.head<3>(),
                                target_pose.head<3>(),
                                current_pose.head<3>()));
    if (trajectory_logger.Enabled()) {
        trajectory_logger.Record("final",
                                 cycle_count + final_hold_cycles,
                                 duration_ms + final_hold_cycles * options.period_ms,
                                 1.0,
                                 target_pose,
                                 current_pose,
                                 current_joints,
                                 true,
                                 current_pose,
                                 start_pose.head<3>(),
                                 target_pose.head<3>());
        trajectory_logger.WriteSummary(final_position_error_cm,
                                       final_rotation_error_deg,
                                       max_lateral_error_cm,
                                       missed_periods);
    }

    PrintPose("final_pose", current_pose);
    std::cout << "final_position_error_cm: " << final_position_error_cm << "\n";
    std::cout << "final_rotation_error_deg: " << final_rotation_error_deg << "\n";
    std::cout << "max_lateral_error_cm: " << max_lateral_error_cm << "\n";
    std::cout << "missed_periods: " << missed_periods << "\n";
    if (trajectory_logger.Enabled()) {
        std::cout << "trajectory_log_written: " << trajectory_logger.Path() << "\n";
        const std::string trajectory_log_path = trajectory_logger.Path();
        trajectory_logger.Close();
        WriteTrajectorySvg(trajectory_log_path);
    }
    return 0;
}
