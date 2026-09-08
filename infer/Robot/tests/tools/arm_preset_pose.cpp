#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <json.hpp>

#include <calibrated_frame_chain.hpp>
#include <force_calibration.hpp>
#include <realman_command.hpp>
#include <rm75_control.hpp>

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kArteryStandoffM = 0.050;
constexpr double kArteryJointLimitWarningDeg = 5.0;
constexpr double kMaximumControllerModelPositionErrorM = 0.025;
constexpr double kMaximumControllerModelOrientationErrorDeg = 5.0;
constexpr int kMaximumIkIterations = 10000;

struct Preset {
    const char* name;
    Eigen::Matrix<double, 7, 1> joints_deg;
    const char* note;
};

struct Options {
    std::string ip = "192.168.50.254";
    int port = 8080;
    int velocity = 5;
    double max_joint_delta_deg = 90.0;
    double max_final_error_deg = 1.0;
    std::string preset;
    std::string target_deg_text;
    std::string artery_path_base;
    std::string target_calibration;
    std::string tool_calibration;
    double max_final_position_error_mm = 2.0;
    bool list_presets = false;
    bool allow_multistep = false;
    bool inspect_target_only = false;
    bool confirm_single_movej = false;
    bool execute = false;
};

struct ArteryTarget {
    Eigen::Vector3d normal_base = Eigen::Vector3d::Zero();
    Eigen::Vector3d surface_point_base_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d probe_tcp_target_base_m = Eigen::Vector3d::Zero();
    std::string camera_serial;
    std::string path_sha256;
    std::string calibration_sha256;
};

const std::vector<Preset>& Presets() {
    static const std::vector<Preset> presets = {
        {
            "ready_verified",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                -28.429, -7.429, 1.023).finished(),
            "Pose observed after successful RM75 10-degree joint-space tests."
        },
        {
            "calib_v4_1",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                 15.000, -5.000, 1.023).finished(),
            "Force calibration v4 pose 1/8; moderate wrist range."
        },
        {
            "calib_v4_2",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                 15.000, 25.000, 1.023).finished(),
            "Force calibration v4 pose 2/8; moderate wrist range."
        },
        {
            "calib_v4_3",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                  0.000, 40.000, 1.023).finished(),
            "Force calibration v4 pose 3/8; moderate wrist range."
        },
        {
            "calib_v4_4",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                -25.000, 40.000, 1.023).finished(),
            "Force calibration v4 pose 4/8; moderate wrist range."
        },
        {
            "calib_v4_5",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                -45.000, 25.000, 1.023).finished(),
            "Force calibration v4 pose 5/8; replaces the cable-sensitive v3 extreme."
        },
        {
            "calib_v4_6",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                -45.000, -5.000, 1.023).finished(),
            "Force calibration v4 pose 6/8; moderate wrist range."
        },
        {
            "calib_v4_7",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                -35.000, -35.000, 1.023).finished(),
            "Force calibration v4 pose 7/8; moderate wrist range."
        },
        {
            "calib_v4_8",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                 -5.000, -35.000, 31.023).finished(),
            "Force calibration v4 pose 8/8; replaces the cable-sensitive v3 extreme."
        },
    };
    return presets;
}

void PrintUsage(const char* program) {
    std::cout
        << "Usage: " << program << " [--ip A.B.C.D] [--port PORT]\n"
        << "       [--list-presets]\n"
        << "       [--preset NAME | --target-deg d1,d2,d3,d4,d5,d6,d7]\n"
        << "       [--velocity V] [--max-joint-delta-deg DEG]\n"
        << "       [--max-final-error-deg DEG]\n"
        << "       [--allow-multistep] [--execute]\n\n"
        << "       --artery-path-base PATH --target-calibration PATH\n"
        << "       --tool-calibration PATH [--inspect-target-only]\n"
        << "       [--max-final-position-error-mm MM]\n"
        << "       [--execute --confirm-single-movej]\n\n"
        << "RM75 joint-pose target tool.\n"
        << "Default controller address is 192.168.50.254:8080.\n"
        << "The program first reads the current joint pose as the initial state,\n"
        << "then plans to the target joint pose from --preset or --target-deg.\n"
        << "Artery mode always selects the first point, adds 50 mm along the\n"
        << "stored outward normal, preserves the current Probe TCP orientation,\n"
        << "and plans exactly one MoveJ. Default mode is dry-run. Artery motion\n"
        << "requires both --execute and --confirm-single-movej.\n";
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

std::string StripJointText(std::string text) {
    text.erase(std::remove_if(text.begin(), text.end(), [](unsigned char c) {
        return std::isspace(c) || c == '[' || c == ']';
    }), text.end());
    return text;
}

bool ParseJointDegList(const std::string& text, Eigen::Matrix<double, 7, 1>& joints_deg) {
    std::stringstream stream(StripJointText(text));
    std::string item;
    std::vector<double> values;
    while (std::getline(stream, item, ',')) {
        if (item.empty()) return false;
        try {
            size_t parsed = 0;
            double value = std::stod(item, &parsed);
            if (parsed != item.size() || !std::isfinite(value)) return false;
            values.push_back(value);
        } catch (...) {
            return false;
        }
    }

    if (values.size() != 7) return false;
    for (int i = 0; i < 7; ++i) {
        joints_deg[i] = values[i];
    }
    return true;
}

const Preset* FindPreset(const std::string& name) {
    for (const auto& preset : Presets()) {
        if (preset.name == name) {
            return &preset;
        }
    }
    return nullptr;
}

void PrintPresets() {
    std::cout << "Available presets:\n";
    std::cout << "  home_current\n";
    std::cout << "    joints_deg: dynamically read from the robot at startup\n";
    std::cout << "    note: Uses the current RM75 joint state as the target pose.\n";
    for (const auto& preset : Presets()) {
        std::cout << "  " << preset.name << "\n";
        std::cout << "    joints_deg: [";
        for (int i = 0; i < preset.joints_deg.size(); ++i) {
            if (i > 0) std::cout << ", ";
            std::cout << preset.joints_deg[i];
        }
        std::cout << "]\n";
        std::cout << "    note: " << preset.note << "\n";
    }
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
        } else if (arg == "--velocity") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.velocity)) return false;
        } else if (arg == "--max-joint-delta-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.max_joint_delta_deg)) return false;
        } else if (arg == "--max-final-error-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.max_final_error_deg)) return false;
        } else if (arg == "--max-final-position-error-mm") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.max_final_position_error_mm)) return false;
        } else if (arg == "--preset") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.preset = value;
        } else if (arg == "--target-deg") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.target_deg_text = value;
        } else if (arg == "--artery-path-base") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.artery_path_base = value;
        } else if (arg == "--target-calibration") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.target_calibration = value;
        } else if (arg == "--tool-calibration") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.tool_calibration = value;
        } else if (arg == "--list-presets") {
            options.list_presets = true;
        } else if (arg == "--allow-multistep") {
            options.allow_multistep = true;
        } else if (arg == "--inspect-target-only") {
            options.inspect_target_only = true;
        } else if (arg == "--confirm-single-movej") {
            options.confirm_single_movej = true;
        } else if (arg == "--execute") {
            options.execute = true;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return false;
        }
    }

    if (options.ip.size() > RMConnectionConfig::kMaximumIpv4TextLength) {
        std::cerr << "IP address is too long: " << options.ip << "\n";
        return false;
    }
    if (options.port <= 0 || options.port > 65535) {
        std::cerr << "Invalid port: " << options.port << "\n";
        return false;
    }
    if (options.velocity <= 0 || options.velocity > 20) {
        std::cerr << "velocity must be in 1..20 for this preset tool\n";
        return false;
    }
    if (options.max_joint_delta_deg <= 0.0 || options.max_joint_delta_deg > 90.0) {
        std::cerr << "max-joint-delta-deg must be in 0..90\n";
        return false;
    }
    if (options.max_final_error_deg <= 0.0 || options.max_final_error_deg > 10.0) {
        std::cerr << "max-final-error-deg must be in 0..10\n";
        return false;
    }
    if (options.max_final_position_error_mm <= 0.0
        || options.max_final_position_error_mm > 5.0) {
        std::cerr << "max-final-position-error-mm must be in 0..5\n";
        return false;
    }
    if (!options.preset.empty() && !options.target_deg_text.empty()) {
        std::cerr << "Use either --preset or --target-deg, not both\n";
        return false;
    }
    const bool artery_mode = !options.artery_path_base.empty()
        || !options.target_calibration.empty() || !options.tool_calibration.empty();
    if (artery_mode
        && (options.artery_path_base.empty()
            || options.target_calibration.empty()
            || options.tool_calibration.empty())) {
        std::cerr << "artery mode requires --artery-path-base, "
                     "--target-calibration and --tool-calibration\n";
        return false;
    }
    if (artery_mode && (!options.preset.empty()
                        || !options.target_deg_text.empty()
                        || options.allow_multistep)) {
        std::cerr << "artery mode is mutually exclusive with preset/manual "
                     "targets and --allow-multistep\n";
        return false;
    }
    if (options.inspect_target_only && (!artery_mode || options.execute)) {
        std::cerr << "--inspect-target-only requires artery mode and forbids --execute\n";
        return false;
    }
    if (options.confirm_single_movej && !artery_mode) {
        std::cerr << "--confirm-single-movej is only valid in artery mode\n";
        return false;
    }
    if (artery_mode && options.execute && !options.confirm_single_movej) {
        std::cerr << "artery execution requires --execute --confirm-single-movej\n";
        return false;
    }
    if (artery_mode && options.velocity > 5) {
        std::cerr << "artery mode velocity must be in 1..5\n";
        return false;
    }
    return true;
}

template <int Size>
void PrintVectorRadDeg(const char* label, const Eigen::Matrix<double, Size, 1>& values_rad) {
    std::cout << label << "_rad: [";
    for (int i = 0; i < values_rad.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << values_rad[i];
    }
    std::cout << "]\n";

    std::cout << label << "_deg: [";
    for (int i = 0; i < values_rad.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << values_rad[i] * kRadToDeg;
    }
    std::cout << "]\n";
}

Eigen::Matrix<double, 7, 1> DegToRad(const Eigen::Matrix<double, 7, 1>& values_deg) {
    return values_deg * kDegToRad;
}

void PrintTargetDeg(const Eigen::Matrix<double, 7, 1>& target_deg) {
    std::cout << "target_joints7_deg: [";
    for (int i = 0; i < target_deg.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << target_deg[i];
    }
    std::cout << "]\n";
}

std::string FormatJointDegList(const Eigen::Matrix<double, 7, 1>& joints_deg) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3);
    for (int i = 0; i < joints_deg.size(); ++i) {
        if (i > 0) stream << ",";
        stream << joints_deg[i];
    }
    return stream.str();
}

void PrintWaypointPlan(const Eigen::Matrix<double, 7, 1>& current_deg,
                       const Eigen::Matrix<double, 7, 1>& delta_deg,
                       int step_count) {
    if (step_count <= 1) {
        return;
    }

    std::cout << "\nPlanned waypoints\n";
    for (int step = 1; step <= step_count; ++step) {
        Eigen::Matrix<double, 7, 1> waypoint_deg =
            current_deg + delta_deg * (static_cast<double>(step) / step_count);
        std::cout << "waypoint " << step << "/" << step_count << "\n";
        PrintTargetDeg(waypoint_deg);
    }
}

bool ParseVector3Line(const std::string& line, Eigen::Vector3d& value) {
    std::istringstream stream(line);
    std::string trailing;
    return static_cast<bool>(stream >> value.x() >> value.y() >> value.z())
        && !(stream >> trailing) && value.array().isFinite().all();
}

bool LoadJsonObject(const std::string& path,
                    nlohmann::json& output,
                    std::string& error) {
    std::ifstream stream(path);
    if (!stream) {
        error = "cannot open JSON: " + path;
        return false;
    }
    output = nlohmann::json::parse(stream, nullptr, false);
    if (output.is_discarded() || !output.is_object()) {
        error = "invalid JSON object: " + path;
        return false;
    }
    return true;
}

bool LoadArteryTarget(const Options& options,
                      ArteryTarget& target,
                      std::string& error) {
    std::ifstream path_stream(options.artery_path_base);
    if (!path_stream) {
        error = "cannot open Base path: " + options.artery_path_base;
        return false;
    }
    std::string line;
    if (!std::getline(path_stream, line)
        || !ParseVector3Line(line, target.normal_base)) {
        error = "Base path first line must be one finite 3D normal";
        return false;
    }
    const double normal_norm = target.normal_base.norm();
    if (!std::isfinite(normal_norm) || normal_norm < 1e-12) {
        error = "Base path normal is zero";
        return false;
    }
    target.normal_base /= normal_norm;

    std::vector<Eigen::Vector3d> points;
    while (std::getline(path_stream, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        Eigen::Vector3d point;
        if (!ParseVector3Line(line, point)) {
            error = "Base path contains a malformed point line";
            return false;
        }
        points.push_back(point);
    }
    if (points.empty()) {
        error = "Base path must contain at least one 3D point after the normal";
        return false;
    }
    target.surface_point_base_m = points.front();
    target.probe_tcp_target_base_m = target.surface_point_base_m
        + kArteryStandoffM * target.normal_base;

    if (!ComputeFileSha256(options.artery_path_base,
                           target.path_sha256, &error)) {
        return false;
    }
    if (!ComputeFileSha256(options.target_calibration,
                           target.calibration_sha256, &error)) {
        return false;
    }

    nlohmann::json calibration;
    if (!LoadJsonObject(options.target_calibration, calibration, error)) {
        return false;
    }
    try {
        if (calibration.value("schema_version", 0) != 1
            || !calibration.value("accepted", false)
            || calibration.value("transform", std::string())
                != "d455_color_optical_to_rm75_base") {
            error = "camera calibration is not an accepted camera-to-Base result";
            return false;
        }
        target.camera_serial = calibration.at("camera").at("serial").get<std::string>();
        if (target.camera_serial.empty()) {
            error = "camera calibration serial is empty";
            return false;
        }
    } catch (const std::exception& exception) {
        error = std::string("camera calibration field error: ") + exception.what();
        return false;
    }

    nlohmann::json metadata;
    const std::string metadata_path = options.artery_path_base + ".json";
    if (!LoadJsonObject(metadata_path, metadata, error)) return false;
    try {
        if (metadata.value("schema_version", 0) != 1
            || metadata.value("frame", std::string()) != "rm75_base"
            || metadata.value("translation_unit", std::string()) != "m"
            || metadata.value("normal_semantics", std::string())
                != "rotation_only_and_unit_normalized") {
            error = "Base path metadata frame, unit or normal semantics is invalid";
            return false;
        }
        if (metadata.at("point_count").get<std::size_t>() != points.size()) {
            error = "Base path point count does not match metadata";
            return false;
        }
        if (metadata.at("source_camera_serial").get<std::string>()
                != target.camera_serial
            || metadata.at("calibration_sha256").get<std::string>()
                != target.calibration_sha256) {
            error = "Base path provenance does not match camera calibration";
            return false;
        }
        const std::string recorded_path_hash =
            metadata.value("output_path_sha256", std::string());
        if (recorded_path_hash.empty() || recorded_path_hash != target.path_sha256) {
            error = "Base path content hash is absent or does not match metadata; rerun calibrate.py convert";
            return false;
        }
    } catch (const std::exception& exception) {
        error = std::string("Base path metadata field error: ") + exception.what();
        return false;
    }
    return true;
}

double RotationDifferenceDeg(const Eigen::Matrix3d& lhs,
                             const Eigen::Matrix3d& rhs) {
    const Eigen::AngleAxisd difference(lhs * rhs.transpose());
    return std::abs(difference.angle()) * kRadToDeg;
}

bool SolveFixedOrientationTarget(
    Rm75ServoPlanner& planner,
    const Eigen::Matrix<double, 7, 1>& initial_joints,
    const Eigen::Matrix<double, 6, 1>& desired_model_pose,
    Eigen::Matrix<double, 7, 1>& target_joints,
    Eigen::Matrix<double, 6, 1>& target_model_pose,
    int& iterations,
    double& minimum_joint_margin_deg,
    std::string& error) {
    target_joints = initial_joints;
    target_model_pose = planner.PoseFromJoints(target_joints);
    Eigen::Matrix<double, 7, 1> previous_delta =
        Eigen::Matrix<double, 7, 1>::Zero();
    minimum_joint_margin_deg = std::numeric_limits<double>::infinity();
    for (iterations = 0; iterations < kMaximumIkIterations; ++iterations) {
        const double position_error_m =
            (desired_model_pose.head<3>() - target_model_pose.head<3>()).norm();
        const double orientation_error_deg = RotationDifferenceDeg(
            RotationBaseFromControllerEuler(desired_model_pose.tail<3>()),
            RotationBaseFromControllerEuler(target_model_pose.tail<3>()));
        if (position_error_m <= 0.0005 && orientation_error_deg <= 0.1) {
            return true;
        }
        const Rm75ServoPlan plan = planner.Plan(
            target_joints, target_model_pose, desired_model_pose, previous_delta);
        if (!plan.valid) {
            error = std::string("IK rejected: ")
                + Rm75PlanErrorString(plan.error) + ": " + plan.detail;
            return false;
        }
        if (plan.near_joint_limit) {
            int minimum_margin_joint = 0;
            double minimum_margin_deg = std::numeric_limits<double>::infinity();
            for (int joint = 0; joint < 7; ++joint) {
                const double margin_deg = std::min(
                    plan.target_joints[joint]
                        - planner.Kinematics().JointMinimums()[joint],
                    planner.Kinematics().JointMaximums()[joint]
                        - plan.target_joints[joint]) * kRadToDeg;
                if (margin_deg < minimum_margin_deg) {
                    minimum_margin_deg = margin_deg;
                    minimum_margin_joint = joint;
                }
            }
            std::ostringstream detail;
            detail << "IK entered joint-limit warning region: J"
                   << (minimum_margin_joint + 1) << " target="
                   << plan.target_joints[minimum_margin_joint] * kRadToDeg
                   << " deg, limits=["
                   << planner.Kinematics().JointMinimums()[minimum_margin_joint]
                          * kRadToDeg
                   << ", "
                   << planner.Kinematics().JointMaximums()[minimum_margin_joint]
                          * kRadToDeg
                   << "] deg, margin=" << minimum_margin_deg
                   << " deg is below warning threshold="
                   << planner.Config().joint_limit_warning_deg << " deg";
            error = detail.str();
            return false;
        }
        if (plan.near_singularity) {
            error = "IK entered singularity warning region";
            return false;
        }
        minimum_joint_margin_deg = std::min(
            minimum_joint_margin_deg, plan.minimum_joint_margin_deg);
        target_joints = plan.target_joints;
        target_model_pose = plan.model_pose;
        previous_delta = plan.joint_delta;
    }
    error = "IK did not converge within the fixed iteration limit";
    return false;
}

bool ValidateMoveJSegment(
    Rm75ServoPlanner& planner,
    const Eigen::Matrix<double, 7, 1>& current_joints,
    const Eigen::Matrix<double, 7, 1>& target_joints,
    double& minimum_joint_margin_deg,
    std::string& error) {
    constexpr int kSegmentSamples = 200;
    minimum_joint_margin_deg = std::numeric_limits<double>::infinity();
    for (int sample = 0; sample <= kSegmentSamples; ++sample) {
        const double fraction = static_cast<double>(sample) / kSegmentSamples;
        const Eigen::Matrix<double, 7, 1> joints =
            current_joints + fraction * (target_joints - current_joints);
        const Eigen::Matrix<double, 6, 1> pose = planner.PoseFromJoints(joints);
        const Rm75ServoPlan check = planner.Plan(joints, pose, pose);
        if (!check.valid || check.near_joint_limit || check.near_singularity) {
            std::ostringstream detail;
            detail << "MoveJ joint-space segment rejected at "
                   << 100.0 * fraction << "%: "
                   << Rm75PlanErrorString(check.error) << ": " << check.detail;
            error = detail.str();
            return false;
        }
        minimum_joint_margin_deg = std::min(
            minimum_joint_margin_deg, check.minimum_joint_margin_deg);
    }
    return true;
}

void PrintVector3(const char* label, const Eigen::Vector3d& value) {
    std::cout << label << ": [" << value.x() << ", " << value.y()
              << ", " << value.z() << "]\n";
}

int RunArteryMode(const Options& options,
                  const ArteryTarget& artery,
                  const ForceCalibration& tool_calibration,
                  RMCommand& command,
                  const Eigen::Matrix<double, 7, 1>& current_joints,
                  const Eigen::Matrix<double, 6, 1>& current_controller_pose) {
    const CalibratedFrameChain frame_chain(tool_calibration);
    Rm75ServoPlannerConfig planner_config;
    planner_config.joint_limit_warning_deg = kArteryJointLimitWarningDeg;
    Rm75ServoPlanner planner(planner_config);
    const Eigen::Matrix<double, 6, 1> current_model_pose =
        planner.PoseFromJoints(current_joints);
    const double model_position_difference_m =
        (current_controller_pose.head<3>() - current_model_pose.head<3>()).norm();
    const double model_orientation_difference_deg = RotationDifferenceDeg(
        frame_chain.RotationBaseFromArmTip(current_controller_pose),
        frame_chain.RotationBaseFromArmTip(current_model_pose));

    std::cout << "\nArtery single-point plan\n";
    std::cout << "point_index: 0\n";
    std::cout << "frame: rm75_base\n";
    std::cout << "translation_unit: m\n";
    std::cout << "joint_limit_warning_deg: "
              << planner.Config().joint_limit_warning_deg << "\n";
    std::cout << "joint_limit_stop_deg: "
              << planner.Config().joint_limit_stop_deg << "\n";
    std::cout << "camera_serial: " << artery.camera_serial << "\n";
    std::cout << "path_sha256: " << artery.path_sha256 << "\n";
    std::cout << "camera_calibration_sha256: "
              << artery.calibration_sha256 << "\n";
    PrintVector3("surface_point_base_m", artery.surface_point_base_m);
    PrintVector3("outward_normal_base", artery.normal_base);
    std::cout << "standoff_m: " << kArteryStandoffM << "\n";
    PrintVector3("target_probe_tcp_base_m",
                 artery.probe_tcp_target_base_m);
    std::cout << "tool_chain_verified: "
              << (tool_calibration.tool_chain_verified ? "true" : "false")
              << "\n";
    std::cout << "controller_model_position_difference_mm: "
              << 1000.0 * model_position_difference_m << "\n";
    std::cout << "controller_model_orientation_difference_deg: "
              << model_orientation_difference_deg << "\n";
    if (model_position_difference_m > kMaximumControllerModelPositionErrorM
        || model_orientation_difference_deg
            > kMaximumControllerModelOrientationErrorDeg) {
        std::cerr << "Controller pose and local RM75 model disagree beyond the "
                     "fixed safety gate. Refusing IK.\n";
        return 4;
    }

    const Eigen::Vector3d current_probe_tcp_base_m =
        frame_chain.ProbeTcpBase(current_controller_pose);
    const double requested_travel_m =
        (artery.probe_tcp_target_base_m - current_probe_tcp_base_m).norm();
    PrintVector3("current_probe_tcp_base_m", current_probe_tcp_base_m);
    std::cout << "requested_probe_tcp_travel_mm: "
              << 1000.0 * requested_travel_m << "\n";
    std::cout << "distance_gate_enabled: false\n";
    std::cout << "WARNING: no Cartesian distance gate; this tool has no "
                 "environment collision model.\n";

    // Preserve the current orientation. A bounded local translation offset
    // aligns the controller-reported pose with the local FK model at the
    // measured start state; rotations must already agree via the gate above.
    const Eigen::Vector3d controller_minus_model =
        current_controller_pose.head<3>() - current_model_pose.head<3>();
    const Eigen::Matrix3d current_rotation =
        frame_chain.RotationBaseFromArmTip(current_controller_pose);
    const Eigen::Vector3d desired_controller_armtip =
        artery.probe_tcp_target_base_m
        - current_rotation * frame_chain.ProbeTcpArmTipM();
    Eigen::Matrix<double, 6, 1> desired_model_pose = current_model_pose;
    desired_model_pose.head<3>() =
        desired_controller_armtip - controller_minus_model;

    Eigen::Matrix<double, 7, 1> target_joints;
    Eigen::Matrix<double, 6, 1> target_model_pose;
    int ik_iterations = 0;
    double minimum_joint_margin_deg = 0.0;
    std::string ik_error;
    if (!SolveFixedOrientationTarget(planner, current_joints,
                                     desired_model_pose, target_joints,
                                     target_model_pose, ik_iterations,
                                     minimum_joint_margin_deg, ik_error)) {
        std::cerr << ik_error << "\n";
        return 4;
    }

    const double maximum_joint_delta_deg =
        MaximumWrappedJointDeltaDeg(target_joints, current_joints);
    const Eigen::Vector3d planned_probe_tcp_base_m =
        target_model_pose.head<3>() + controller_minus_model
        + frame_chain.RotationBaseFromArmTip(target_model_pose)
            * frame_chain.ProbeTcpArmTipM();
    const double planned_position_error_m =
        (planned_probe_tcp_base_m - artery.probe_tcp_target_base_m).norm();
    std::cout << "ik_iterations: " << ik_iterations << "\n";
    std::cout << "minimum_joint_margin_deg: "
              << minimum_joint_margin_deg << "\n";
    std::cout << "maximum_joint_delta_deg_actual: "
              << maximum_joint_delta_deg << "\n";
    std::cout << "joint_delta_gate_enabled: false\n";
    PrintVector3("planned_probe_tcp_base_m", planned_probe_tcp_base_m);
    std::cout << "planned_probe_tcp_error_mm: "
              << 1000.0 * planned_position_error_m << "\n";
    PrintVectorRadDeg("target_joints7", target_joints);
    if (planned_position_error_m > 0.0006) {
        std::cerr << "IK target exceeds the fixed 0.6 mm planning residual.\n";
        return 4;
    }
    double movej_minimum_joint_margin_deg = 0.0;
    std::string movej_segment_error;
    if (!ValidateMoveJSegment(planner, current_joints, target_joints,
                              movej_minimum_joint_margin_deg,
                              movej_segment_error)) {
        std::cerr << movej_segment_error << "\n";
        return 4;
    }
    std::cout << "movej_segment_samples: 201\n";
    std::cout << "movej_minimum_joint_margin_deg: "
              << movej_minimum_joint_margin_deg << "\n";

    if (!options.execute) {
        std::cout << "\nDry-run only. Exactly zero motion commands were sent.\n";
        if (!tool_calibration.tool_chain_verified) {
            std::cout << "BLOCKED_FOR_EXECUTION: tool_chain_verified=false\n";
        } else {
            std::cout << "Execution still requires onsite checks plus both "
                         "--execute and --confirm-single-movej.\n";
        }
        return 0;
    }

    std::cout << "\nExecuting exactly one MoveJ to the validated IK target.\n";
    const RMResult move_result = command.TryMoveJ(target_joints, options.velocity);
    if (!move_result) {
        std::cerr << "MoveJ failed: " << move_result.message << "\n";
        (void)command.TryStopMotion(1000);
        return 5;
    }

    Eigen::Matrix<double, 7, 1> final_joints;
    Eigen::Matrix<double, 6, 1> final_pose;
    int arm_err = 0;
    int sys_err = 0;
    const RMResult final_result = command.TryReadArmState(
        final_joints, final_pose, arm_err, sys_err);
    if (!final_result || arm_err != 0 || sys_err != 0) {
        std::cerr << "Final state is unavailable or reports a robot error.\n";
        (void)command.TryStopMotion(1000);
        return 5;
    }
    const Eigen::Vector3d final_probe_tcp_base_m =
        frame_chain.ProbeTcpBase(final_pose);
    const double final_position_error_mm = 1000.0
        * (final_probe_tcp_base_m - artery.probe_tcp_target_base_m).norm();
    const double final_joint_error_deg =
        MaximumWrappedJointDeltaDeg(final_joints, target_joints);
    std::cout << "\nFinal state\n";
    PrintVector3("final_probe_tcp_base_m", final_probe_tcp_base_m);
    std::cout << "final_probe_tcp_error_mm: "
              << final_position_error_mm << "\n";
    std::cout << "maximum_final_joint_error_deg: "
              << final_joint_error_deg << "\n";
    if (final_position_error_mm > options.max_final_position_error_mm
        || final_joint_error_deg > options.max_final_error_deg) {
        std::cerr << "Final Probe TCP or joint error exceeds its acceptance gate.\n";
        (void)command.TryStopMotion(1000);
        return 6;
    }
    std::cout << "target_reached: true\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        PrintUsage(argv[0]);
        return 2;
    }

    if (options.list_presets) {
        PrintPresets();
        return 0;
    }

    const bool artery_mode = !options.artery_path_base.empty();
    ArteryTarget artery_target;
    ForceCalibration tool_calibration;
    if (artery_mode) {
        std::string error;
        if (!LoadArteryTarget(options, artery_target, error)) {
            std::cerr << "Invalid artery target: " << error << "\n";
            return 2;
        }
        if (!tool_calibration.LoadJson(options.tool_calibration, &error)) {
            std::cerr << "Invalid Probe TCP/tool calibration: " << error << "\n";
            return 2;
        }
        std::string tool_calibration_sha256;
        if (!ComputeFileSha256(options.tool_calibration,
                               tool_calibration_sha256, &error)) {
            std::cerr << "Cannot hash Probe TCP/tool calibration: "
                      << error << "\n";
            return 2;
        }
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "artery_target_valid: true\n";
        std::cout << "selected_point_index: 0\n";
        PrintVector3("surface_point_base_m",
                     artery_target.surface_point_base_m);
        PrintVector3("outward_normal_base", artery_target.normal_base);
        PrintVector3("target_probe_tcp_base_m",
                     artery_target.probe_tcp_target_base_m);
        std::cout << "standoff_m: " << kArteryStandoffM << "\n";
        std::cout << "tool_calibration_sha256: "
                  << tool_calibration_sha256 << "\n";
        std::cout << "tool_chain_verified: "
                  << (tool_calibration.tool_chain_verified ? "true" : "false")
                  << "\n";
        if (options.inspect_target_only) {
            std::cout << "offline_inspection_only: true\n";
            std::cout << "robot_connection_attempted: false\n";
            return 0;
        }
        if (options.execute && !tool_calibration.tool_chain_verified) {
            std::cerr << "BLOCKED: Probe TCP/tool chain has not been independently "
                         "verified; execution is forbidden.\n";
            return 4;
        }
    }

    // Reject malformed or unknown targets before opening the robot socket.
    // In particular, an invalid --execute request must not have any observable
    // effect on a controller that happens to be reachable.
    if (!options.target_deg_text.empty()) {
        Eigen::Matrix<double, 7, 1> validated_target_deg;
        if (!ParseJointDegList(options.target_deg_text,
                               validated_target_deg)) {
            std::cerr << "Invalid --target-deg. Expected 7 finite, "
                         "comma-separated degree values.\n";
            return 2;
        }
    }
    if (!options.preset.empty() && options.preset != "home_current"
        && FindPreset(options.preset) == nullptr) {
        std::cerr << "Unknown preset: " << options.preset << "\n";
        PrintPresets();
        return 2;
    }

    RMCommand command(RMConnectionConfig{options.ip, options.port});

    std::cout << "Connecting to Realman controller at "
              << options.ip << ":" << options.port << "\n";
    const RMResult connect_result = command.TryConnectTCPSocket();
    if (!connect_result) {
        std::cerr << "Failed to connect to the Realman controller: "
                  << connect_result.message << "\n";
        return 3;
    }
    if (options.execute) {
        const RMResult startup_stop = command.TryStopMotion(1000);
        if (!startup_stop) {
            std::cerr << "Failed to stop inherited motion before planning: "
                      << startup_stop.message << "\n";
            return 3;
        }
    }

    Eigen::Matrix<double, 7, 1> current_joints =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 1> current_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    int arm_err = 0;
    int sys_err = 0;
    const RMResult initial_state_result = command.TryReadArmState(
        current_joints, current_pose, arm_err, sys_err);
    if (!initial_state_result) {
        std::cerr << "Failed to read the initial robot state: "
                  << initial_state_result.message << "\n";
        return 3;
    }

    std::cout << "arm_err: " << arm_err << "\n";
    std::cout << "sys_err: " << sys_err << "\n";
    PrintVectorRadDeg("current_joints7", current_joints);
    Eigen::Matrix<double, 7, 1> current_deg = current_joints * kRadToDeg;
    std::string current_deg_text = FormatJointDegList(current_deg);
    std::cout << "current_as_target_deg: \"" << current_deg_text << "\"\n";
    std::cout << "copyable_dry_run_command: " << argv[0]
              << " --target-deg \"" << current_deg_text << "\"\n";

    if (arm_err != 0 || sys_err != 0) {
        std::cerr << "Robot reports an error. Refusing to plan motion.\n";
        return 3;
    }

    if (artery_mode) {
        return RunArteryMode(options, artery_target, tool_calibration,
                             command, current_joints, current_pose);
    }

    if (options.preset.empty() && options.target_deg_text.empty()) {
        std::cout << "\nNo target selected. Initial state was read; no motion command was sent.\n";
        std::cout << "Use --list-presets, --preset NAME, or --target-deg d1,d2,d3,d4,d5,d6,d7.\n";
        return 0;
    }

    Eigen::Matrix<double, 7, 1> target_deg;
    if (!options.preset.empty()) {
        if (options.preset == "home_current") {
            target_deg = current_joints * kRadToDeg;
            std::cout << "\nSelected preset: home_current\n";
            std::cout << "Using current RM75 joint state as the target pose.\n";
        } else {
            const Preset* preset = FindPreset(options.preset);
            if (preset == nullptr) {
                std::cerr << "Unknown preset: " << options.preset << "\n";
                PrintPresets();
                return 2;
            }
            target_deg = preset->joints_deg;
            std::cout << "\nSelected preset: " << preset->name << "\n";
            std::cout << preset->note << "\n";
        }
    } else if (!ParseJointDegList(options.target_deg_text, target_deg)) {
        std::cerr << "Invalid --target-deg. Expected 7 comma-separated degree values.\n";
        return 2;
    }

    Eigen::Matrix<double, 7, 1> delta_deg = target_deg - current_deg;
    double max_abs_delta = delta_deg.cwiseAbs().maxCoeff();

    std::cout << "\nMove plan\n";
    std::cout << "initial_state_source: current robot joint pose\n";
    std::cout << "target_state_source: "
              << (!options.preset.empty() ? ("preset " + options.preset) : "manual --target-deg")
              << "\n";
    std::cout << "velocity: " << options.velocity << "\n";
    std::cout << "max_joint_delta_deg_limit: " << options.max_joint_delta_deg << "\n";
    std::cout << "max_final_error_deg_limit: " << options.max_final_error_deg << "\n";
    std::cout << "max_joint_delta_deg_actual: " << max_abs_delta << "\n";
    PrintTargetDeg(target_deg);

    int step_count = 1;
    if (max_abs_delta > options.max_joint_delta_deg && !options.allow_multistep) {
        std::cerr << "Target exceeds max-joint-delta-deg. Refusing to send motion command.\n";
        std::cerr << "Use a nearer intermediate target or add --allow-multistep after checking safety.\n";
        return 4;
    }
    if (max_abs_delta > options.max_joint_delta_deg) {
        step_count = static_cast<int>(std::ceil(max_abs_delta / options.max_joint_delta_deg));
    }

    std::cout << "allow_multistep: " << (options.allow_multistep ? "true" : "false") << "\n";
    std::cout << "planned_steps: " << step_count << "\n";
    if (step_count > 1) {
        std::cout << "planned_max_step_delta_deg: " << (max_abs_delta / step_count) << "\n";
    }
    PrintWaypointPlan(current_deg, delta_deg, step_count);

    if (!options.execute) {
        std::cout << "\nDry-run only. No motion command was sent.\n";
        std::cout << "Add --execute after confirming the robot workspace is clear.\n";
        return 0;
    }

    std::cout << "\nExecuting MoveJ to target preset/pose";
    if (step_count > 1) {
        std::cout << " in " << step_count << " joint-space steps";
    }
    std::cout << ".\n";

    for (int step = 1; step <= step_count; ++step) {
        Eigen::Matrix<double, 7, 1> waypoint_deg =
            current_deg + delta_deg * (static_cast<double>(step) / step_count);
        std::cout << "\nStep " << step << "/" << step_count << "\n";
        PrintTargetDeg(waypoint_deg);
        Eigen::Matrix<double, 7, 1> waypoint_rad = DegToRad(waypoint_deg);
        const RMResult move_result = command.TryMoveJ(
            waypoint_rad, options.velocity);
        if (!move_result) {
            std::cerr << "MoveJ step " << step << " failed: "
                      << move_result.message << "\n";
            (void)command.TryStopMotion(1000);
            return 5;
        }
    }

    Eigen::Matrix<double, 7, 1> final_joints =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 1> final_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    const RMResult final_state_result = command.TryReadArmState(
        final_joints, final_pose, arm_err, sys_err);
    if (!final_state_result) {
        std::cerr << "Failed to read the final robot state: "
                  << final_state_result.message << "\n";
        return 5;
    }
    std::cout << "\nFinal state\n";
    std::cout << "arm_err: " << arm_err << "\n";
    std::cout << "sys_err: " << sys_err << "\n";
    PrintVectorRadDeg("final_joints7", final_joints);
    Eigen::Matrix<double, 7, 1> final_error_deg = target_deg - final_joints * kRadToDeg;
    double max_final_error = final_error_deg.cwiseAbs().maxCoeff();
    std::cout << "target_error_deg: [";
    for (int i = 0; i < final_error_deg.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << final_error_deg[i];
    }
    std::cout << "]\n";
    std::cout << "max_target_error_deg: " << max_final_error << "\n";
    if (arm_err != 0 || sys_err != 0) {
        std::cerr << "Target execution finished with robot error state.\n";
        return 5;
    }
    if (max_final_error > options.max_final_error_deg) {
        std::cerr << "Target execution error exceeds max-final-error-deg.\n";
        return 6;
    }
    std::cout << "target_reached: true\n";
    return 0;
}
