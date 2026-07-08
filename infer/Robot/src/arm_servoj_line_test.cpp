#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
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
const Eigen::Matrix<double, 7, 1> kRm75JointMaxSpeedDegS =
    (Eigen::Matrix<double, 7, 1>() << 180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0).finished();

struct Options {
    std::string ip = "192.168.50.254";
    int port = 8080;
    Eigen::Vector3d delta_cm = (Eigen::Vector3d() << 2.0, 0.0, 0.0).finished();
    Eigen::Vector3d target_position_cm = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_rotation_deg = Eigen::Vector3d::Zero();
    bool use_delta = false;
    bool use_target_position = false;
    bool use_target_rotation = false;
    double max_tcp_speed_cm_s = 5.0;
    int final_hold_ms = 2000;
    int period_ms = 20;
    int feedback_every = 10;
    bool follow = false;
    bool use_s_curve = false;
    double max_total_delta_cm = 100.0;
    double max_rotation_delta_deg = 180.0;
    double max_joint_step_deg = 0.0;
    bool use_official_joint_max_speed = true;
    double max_joint_accel_deg_s2 = 0.0;
    double joint_limit_warning_deg = 10.0;
    double joint_limit_stop_deg = 3.0;
    double damping = 0.0005;
    double singularity_warning_deg = 5.0;
    bool allow_near_singularity = false;
    bool summary_only = false;
    bool execute = false;
};

void PrintUsage(const char* program) {
    std::cout
        << "Usage: " << program << " [--delta-cm X,Y,Z]\n"
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
        << "       [--allow-near-singularity] [--summary-only] [--execute]\n\n"
        << "RM75 fixed-period streaming ServoJ pose test.\n"
        << "Use --delta-cm for relative position-only motion.\n"
        << "Use --target-position-cm/--target-rotation-deg for absolute 6D pose control.\n"
        << "The tool uses fixed-period streaming ServoJ only.\n"
        << "Default interpolation is linear, matching the stable ServoJ baseline.\n"
        << "Use --interpolation s-curve only for tuning.\n"
        << "Joint step is limited by official RM75 max joint speeds unless --max-joint-step-deg is set.\n"
        << "Joint acceleration is limited only when --max-joint-accel-deg-s2 is greater than 0.\n"
        << "Duration is estimated automatically from joint and TCP speed limits.\n"
        << "Default max total delta is 100 cm.\n"
        << "Omitted absolute target fields use the current pose. Default mode is dry-run.\n";
}

bool ParseInt(const char* text, int& value) {
    try {
        size_t parsed = 0;
        int parsed_value = std::stoi(text, &parsed, 10);
        if (parsed != std::strlen(text)) return false;
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
        if (parsed != std::strlen(text)) return false;
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

    if (options.ip.size() >= sizeof(RMCommand().rlm_ip)) {
        std::cerr << "IP address is too long.\n";
        return false;
    }
    if (options.port <= 0 || options.port > 65535) {
        std::cerr << "Invalid port.\n";
        return false;
    }
    if ((options.use_target_position || options.use_target_rotation) && options.use_delta) {
        std::cerr << "Use either --delta-cm or absolute target pose options, not both.\n";
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
    if (options.feedback_every < 0 || options.feedback_every > 100) {
        std::cerr << "feedback-every must be in 0..100\n";
        return false;
    }
    if (options.max_total_delta_cm <= 0.0 || options.max_total_delta_cm > 100.0) {
        std::cerr << "max-total-delta-cm must be in 0..100\n";
        return false;
    }
    if (!(options.use_target_position || options.use_target_rotation)
        && (options.delta_cm.norm() <= 0.0 || options.delta_cm.norm() > options.max_total_delta_cm)) {
        std::cerr << "delta-cm norm must be non-zero and within max-total-delta-cm.\n";
        return false;
    }
    if (options.max_rotation_delta_deg <= 0.0 || options.max_rotation_delta_deg > 180.0) {
        std::cerr << "max-rotation-delta-deg must be in 0..180\n";
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
    if (options.max_joint_accel_deg_s2 < 0.0 || options.max_joint_accel_deg_s2 > 5000.0) {
        std::cerr << "max-joint-accel-deg-s2 must be in 0..5000; 0 disables it\n";
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
        options.max_joint_accel_deg_s2 * kDegToRad * period_s * period_s;
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
        if (joints[i] < kinematics.joint_min[i] || joints[i] > kinematics.joint_max[i]) {
            std::cerr << "Target joint J" << (i + 1) << " exceeds RM75 joint limits.\n";
            return false;
        }
    }
    return true;
}

double JointLimitMarginDeg(const RMKinematics& kinematics,
                           const Eigen::Matrix<double, 7, 1>& joints,
                           int joint) {
    const double lower_margin = joints[joint] - kinematics.joint_min[joint];
    const double upper_margin = kinematics.joint_max[joint] - joints[joint];
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

    Eigen::Matrix<double, 7, 1> max_step_rad;
    if (options.use_official_joint_max_speed) {
        const double period_s = static_cast<double>(options.period_ms) / 1000.0;
        max_step_rad = kRm75JointMaxSpeedDegS * period_s * kDegToRad;
    } else {
        max_step_rad.setConstant(options.max_joint_step_deg * kDegToRad);
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

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        PrintUsage(argv[0]);
        return 2;
    }

    RMCommand command;
    command.quiet = true;
    command.rlm_port = options.port;
    std::strncpy(command.rlm_ip, options.ip.c_str(), sizeof(command.rlm_ip) - 1);
    command.rlm_ip[sizeof(command.rlm_ip) - 1] = '\0';
    command.ConnectTCPSocket();

    Eigen::Matrix<double, 7, 1> current_joints;
    Eigen::Matrix<double, 6, 1> current_pose;
    int arm_err = 0;
    int sys_err = 0;
    command.ReadArmState(current_joints, current_pose, arm_err, sys_err);
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
    } else if (!options.use_target_rotation) {
        target_pose.head<3>() += options.delta_cm / 100.0;
    }
    if (options.use_target_rotation) {
        target_pose.tail<3>() = options.target_rotation_deg * kDegToRad;
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
        std::cout << "target_mode: relative_position_delta\n";
        std::cout << "delta_cm: [" << options.delta_cm[0] << ", "
                  << options.delta_cm[1] << ", " << options.delta_cm[2] << "]\n";
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
    if (options.use_official_joint_max_speed) {
        std::cout << "joint_speed_limit: official_rm75_max\n";
        std::cout << "joint_max_speed_deg_s: [180, 180, 225, 225, 225, 225, 225]\n";
    } else {
        std::cout << "joint_speed_limit: manual_step\n";
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
    std::cout << "joint_limit_warning_deg: " << options.joint_limit_warning_deg << "\n";
    std::cout << "joint_limit_stop_deg: " << options.joint_limit_stop_deg << "\n";

    if (!options.execute) {
        std::cout << "\nDry-run only. No motion command was sent.\n";
        std::cout << "Add --execute after confirming the pose path is clear.\n";
        return 0;
    }

    Eigen::Matrix<double, 7, 1> model_joints = current_joints;
    Eigen::Matrix<double, 6, 1> model_pose = current_pose;
    double max_lateral_error_cm = 0.0;
    int missed_periods = 0;
    bool joint_limit_warning_printed = false;
    Eigen::Matrix<double, 7, 1> previous_joint_delta;
    previous_joint_delta.setZero();

    auto next_tick = std::chrono::steady_clock::now();
    for (int cycle = 1; cycle <= cycle_count; ++cycle) {
        const double phase = static_cast<double>(cycle) / static_cast<double>(cycle_count);
        Eigen::Matrix<double, 6, 1> desired_pose =
            InterpolatePose(start_pose, target_pose, phase, options.use_s_curve);

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
        if (!CheckJointLimitMargins("planned_target",
                                    kinematics,
                                    target_joints,
                                    options.joint_limit_warning_deg,
                                    options.joint_limit_stop_deg,
                                    true,
                                    &joint_limit_warning_printed)) {
            return 4;
        }

        const bool near_singularity =
            PrintSingularityWarning("planned_target", target_joints, options.singularity_warning_deg);
        if (near_singularity && !options.allow_near_singularity) {
            std::cerr << "Planned target is near an official RM75 singular pattern.\n";
            return 5;
        }

        command.ServoJ(target_joints, options.follow);
        model_joints = target_joints;
        previous_joint_delta = limited_joint_delta;
        model_pose = ControllerPoseFromJoints(kinematics, model_joints);

        if (options.feedback_every > 0 && cycle % options.feedback_every == 0) {
            command.ReadArmState(current_joints, current_pose, arm_err, sys_err);
            if (arm_err != 0 || sys_err != 0) {
                std::cerr << "ServoJ line feedback reports robot error state.\n";
                return 6;
            }
            model_joints = current_joints;
            model_pose = current_pose;

            const double lateral_cm = LateralErrorCm(start_pose.head<3>(),
                                                     target_pose.head<3>(),
                                                     current_pose.head<3>());
            max_lateral_error_cm = std::max(max_lateral_error_cm, lateral_cm);
            if (!options.summary_only && cycle % (options.feedback_every * 5) == 0) {
                std::cout << "cycle: " << cycle << "/" << cycle_count
                          << " lateral_error_cm=" << lateral_cm << "\n";
            }
        }

        next_tick += std::chrono::milliseconds(options.period_ms);
        const auto now = std::chrono::steady_clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
        } else {
            ++missed_periods;
            next_tick = now;
        }
    }

    const int final_hold_cycles =
        std::max(0, options.final_hold_ms / options.period_ms);
    for (int hold_cycle = 1; hold_cycle <= final_hold_cycles; ++hold_cycle) {
        Eigen::Matrix<double, 7, 1> target_joints;
        if (!PlanPoseStep(kinematics,
                          options,
                          model_joints,
                          model_pose,
                          target_pose,
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
        if (!CheckJointLimitMargins("final_hold_target",
                                    kinematics,
                                    target_joints,
                                    options.joint_limit_warning_deg,
                                    options.joint_limit_stop_deg,
                                    true,
                                    &joint_limit_warning_printed)) {
            return 4;
        }

        const bool near_singularity =
            PrintSingularityWarning("final_hold_target",
                                    target_joints,
                                    options.singularity_warning_deg);
        if (near_singularity && !options.allow_near_singularity) {
            std::cerr << "Final hold target is near an official RM75 singular pattern.\n";
            return 5;
        }

        command.ServoJ(target_joints, options.follow);
        model_joints = target_joints;
        previous_joint_delta = limited_joint_delta;
        model_pose = ControllerPoseFromJoints(kinematics, model_joints);

        if (options.feedback_every > 0 && hold_cycle % options.feedback_every == 0) {
            command.ReadArmState(current_joints, current_pose, arm_err, sys_err);
            if (arm_err != 0 || sys_err != 0) {
                std::cerr << "ServoJ final hold feedback reports robot error state.\n";
                return 6;
            }
            model_joints = current_joints;
            model_pose = current_pose;

            const double lateral_cm = LateralErrorCm(start_pose.head<3>(),
                                                     target_pose.head<3>(),
                                                     current_pose.head<3>());
            max_lateral_error_cm = std::max(max_lateral_error_cm, lateral_cm);
            if (!options.summary_only && hold_cycle % (options.feedback_every * 5) == 0) {
                std::cout << "final_hold_cycle: " << hold_cycle << "/"
                          << final_hold_cycles
                          << " lateral_error_cm=" << lateral_cm << "\n";
            }
        }

        next_tick += std::chrono::milliseconds(options.period_ms);
        const auto now = std::chrono::steady_clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
        } else {
            ++missed_periods;
            next_tick = now;
        }
    }

    command.ReadArmState(current_joints, current_pose, arm_err, sys_err);
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

    PrintPose("final_pose", current_pose);
    std::cout << "final_position_error_cm: " << final_position_error_cm << "\n";
    std::cout << "final_rotation_error_deg: " << final_rotation_error_deg << "\n";
    std::cout << "max_lateral_error_cm: " << max_lateral_error_cm << "\n";
    std::cout << "missed_periods: " << missed_periods << "\n";
    return 0;
}
