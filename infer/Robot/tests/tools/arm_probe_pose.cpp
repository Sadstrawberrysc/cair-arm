#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
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
constexpr double kEulerSingularityEpsilon = 1e-9;
constexpr double kMaximumControllerModelPositionErrorM = 0.025;
constexpr double kMaximumControllerModelOrientationErrorDeg = 5.0;
constexpr double kIkPositionToleranceM = 0.0005;
constexpr double kIkOrientationToleranceDeg = 0.1;
constexpr double kArteryStandoffM = 0.050;
constexpr int kMaximumIkIterations = 10000;

struct Options {
    std::string ip = "192.168.50.254";
    int port = 8080;
    int velocity = 1;
    std::string target_pose_text;
    std::string target_position_text;
    std::string artery_path_base;
    std::string tool_calibration;
    double max_joint_delta_deg = 60.0;
    double max_final_position_error_mm = 2.0;
    double max_final_orientation_error_deg = 1.0;
    double max_final_joint_error_deg = 1.0;
    bool inspect_transform_only = false;
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

void PrintUsage(const char* program) {
    std::cout
        << "Usage: " << program << " --target-pose-m-rad x,y,z,rx,ry,rz\n"
        << "   or: " << program << " --target-position-base-m x,y,z\n"
        << "   or: " << program << " --artery-path-base PATH\n"
        << "       --tool-calibration PATH [--ip A.B.C.D] [--port PORT]\n"
        << "       [--velocity 1..5] [--max-joint-delta-deg DEG]\n"
        << "       [--inspect-transform-only]\n"
        << "       [--execute --confirm-single-movej]\n\n"
        << "Moves the RM75 so the Probe TCP reaches one complete pose. The target\n"
        << "is Base->Probe_TCP: x/y/z in metres and controller ZYX rx/ry/rz in\n"
        << "radians. Probe TCP axes are the calibrated physical Tool axes.\n"
        << "Base-position mode preserves the startup Probe TCP orientation and\n"
        << "uses the supplied x/y/z directly, without a position offset.\n"
        << "Artery-path mode implicitly selects the first point, offsets 50 mm\n"
        << "along its outward normal, and preserves the startup Probe TCP\n"
        << "orientation. No point, offset, or orientation option is exposed.\n"
        << "Default mode connects and plans but sends no motion command. Execution\n"
        << "requires a verified tool chain and both explicit execution flags.\n";
}

bool ParseInt(const char* text, int& value) {
    try {
        std::size_t parsed = 0;
        const int result = std::stoi(text, &parsed, 10);
        if (parsed != std::strlen(text)) return false;
        value = result;
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseDouble(const char* text, double& value) {
    try {
        std::size_t parsed = 0;
        const double result = std::stod(text, &parsed);
        if (parsed != std::strlen(text) || !std::isfinite(result)) return false;
        value = result;
        return true;
    } catch (...) {
        return false;
    }
}

bool ParsePose(const std::string& text,
               Eigen::Matrix<double, 6, 1>& pose_m_rad) {
    std::string compact = text;
    compact.erase(std::remove_if(compact.begin(), compact.end(),
                                 [](unsigned char c) {
                                     return std::isspace(c) || c == '[' || c == ']';
                                 }),
                  compact.end());
    std::stringstream stream(compact);
    std::string item;
    std::vector<double> values;
    while (std::getline(stream, item, ',')) {
        double value = 0.0;
        if (item.empty() || !ParseDouble(item.c_str(), value)) return false;
        values.push_back(value);
    }
    if (values.size() != 6) return false;
    for (int i = 0; i < 3; ++i) pose_m_rad[i] = values[i];
    for (int i = 3; i < 6; ++i) pose_m_rad[i] = values[i];
    return pose_m_rad.array().isFinite().all();
}

bool ParseVector3(const std::string& text, Eigen::Vector3d& value) {
    std::string compact = text;
    compact.erase(std::remove_if(compact.begin(), compact.end(),
                                 [](unsigned char c) {
                                     return std::isspace(c) || c == '[' || c == ']';
                                 }),
                  compact.end());
    std::stringstream stream(compact);
    std::string item;
    std::vector<double> values;
    while (std::getline(stream, item, ',')) {
        double parsed = 0.0;
        if (item.empty() || !ParseDouble(item.c_str(), parsed)) return false;
        values.push_back(parsed);
    }
    if (values.size() != 3) return false;
    value << values[0], values[1], values[2];
    return value.array().isFinite().all();
}

bool ParseOptions(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
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
            if (!value) return false;
            options.ip = value;
        } else if (arg == "--port") {
            const char* value = need_value(arg);
            if (!value || !ParseInt(value, options.port)) return false;
        } else if (arg == "--velocity") {
            const char* value = need_value(arg);
            if (!value || !ParseInt(value, options.velocity)) return false;
        } else if (arg == "--target-pose-m-rad") {
            const char* value = need_value(arg);
            if (!value) return false;
            options.target_pose_text = value;
        } else if (arg == "--target-position-base-m") {
            const char* value = need_value(arg);
            if (!value) return false;
            options.target_position_text = value;
        } else if (arg == "--artery-path-base") {
            const char* value = need_value(arg);
            if (!value) return false;
            options.artery_path_base = value;
        } else if (arg == "--tool-calibration") {
            const char* value = need_value(arg);
            if (!value) return false;
            options.tool_calibration = value;
        } else if (arg == "--max-joint-delta-deg") {
            const char* value = need_value(arg);
            if (!value || !ParseDouble(value, options.max_joint_delta_deg)) return false;
        } else if (arg == "--max-final-position-error-mm") {
            const char* value = need_value(arg);
            if (!value || !ParseDouble(value, options.max_final_position_error_mm)) return false;
        } else if (arg == "--max-final-orientation-error-deg") {
            const char* value = need_value(arg);
            if (!value || !ParseDouble(value, options.max_final_orientation_error_deg)) return false;
        } else if (arg == "--max-final-joint-error-deg") {
            const char* value = need_value(arg);
            if (!value || !ParseDouble(value, options.max_final_joint_error_deg)) return false;
        } else if (arg == "--inspect-transform-only") {
            options.inspect_transform_only = true;
        } else if (arg == "--confirm-single-movej") {
            options.confirm_single_movej = true;
        } else if (arg == "--execute") {
            options.execute = true;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return false;
        }
    }

    const bool explicit_pose = !options.target_pose_text.empty();
    const bool base_position_mode = !options.target_position_text.empty();
    const bool artery_mode = !options.artery_path_base.empty();
    const int selected_modes = static_cast<int>(explicit_pose)
        + static_cast<int>(base_position_mode) + static_cast<int>(artery_mode);
    if (selected_modes != 1 || options.tool_calibration.empty()) {
        std::cerr << "Select exactly one of --target-pose-m-rad, "
                     "--target-position-base-m or --artery-path-base, and "
                     "provide --tool-calibration\n";
        return false;
    }
    Eigen::Matrix<double, 6, 1> parsed_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    if (explicit_pose && !ParsePose(options.target_pose_text, parsed_pose)) {
        std::cerr << "Target must contain 6 finite comma-separated values\n";
        return false;
    }
    Eigen::Vector3d parsed_position = Eigen::Vector3d::Zero();
    if (base_position_mode
        && !ParseVector3(options.target_position_text, parsed_position)) {
        std::cerr << "Base target position must contain 3 finite, "
                     "comma-separated metre values\n";
        return false;
    }
    if (options.ip.size() > RMConnectionConfig::kMaximumIpv4TextLength
        || options.port <= 0 || options.port > 65535) {
        std::cerr << "Invalid controller address\n";
        return false;
    }
    if (options.velocity < 1 || options.velocity > 5) {
        std::cerr << "velocity must be in 1..5\n";
        return false;
    }
    if (options.max_joint_delta_deg <= 0.0
        || options.max_joint_delta_deg > 90.0) {
        std::cerr << "max-joint-delta-deg must be in 0..90\n";
        return false;
    }
    if (options.max_final_position_error_mm <= 0.0
        || options.max_final_position_error_mm > 5.0
        || options.max_final_orientation_error_deg <= 0.0
        || options.max_final_orientation_error_deg > 5.0
        || options.max_final_joint_error_deg <= 0.0
        || options.max_final_joint_error_deg > 10.0) {
        std::cerr << "Invalid final-error acceptance gate\n";
        return false;
    }
    if (options.inspect_transform_only && options.execute) {
        std::cerr << "--inspect-transform-only forbids --execute\n";
        return false;
    }
    if (options.confirm_single_movej && !options.execute) {
        std::cerr << "--confirm-single-movej is only valid with --execute\n";
        return false;
    }
    if (options.execute && !options.confirm_single_movej) {
        std::cerr << "execution requires --execute --confirm-single-movej\n";
        return false;
    }
    return true;
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

bool LoadArteryTarget(const std::string& path,
                      ArteryTarget& target,
                      std::string& error) {
    std::ifstream path_stream(path);
    if (!path_stream) {
        error = "cannot open Base path: " + path;
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
        error = "Base path must contain at least one point after the normal";
        return false;
    }
    target.surface_point_base_m = points.front();
    target.probe_tcp_target_base_m = target.surface_point_base_m
        + kArteryStandoffM * target.normal_base;
    if (!ComputeFileSha256(path, target.path_sha256, &error)) return false;

    const std::string metadata_path = path + ".json";
    nlohmann::json metadata;
    if (!LoadJsonObject(metadata_path, metadata, error)) return false;
    std::string calibration_path;
    try {
        if (metadata.value("schema_version", 0) != 1
            || metadata.value("frame", std::string()) != "rm75_base"
            || metadata.value("translation_unit", std::string()) != "m"
            || metadata.value("normal_semantics", std::string())
                != "rotation_only_and_unit_normalized"
            || metadata.at("point_count").get<std::size_t>() != points.size()
            || metadata.value("output_path_sha256", std::string())
                != target.path_sha256) {
            error = "Base path metadata or content hash is invalid";
            return false;
        }
        target.camera_serial =
            metadata.at("source_camera_serial").get<std::string>();
        target.calibration_sha256 =
            metadata.at("calibration_sha256").get<std::string>();
        calibration_path = metadata.at("calibration_path").get<std::string>();
    } catch (const std::exception& exception) {
        error = std::string("Base path metadata field error: ") + exception.what();
        return false;
    }
    if (target.camera_serial.empty() || target.calibration_sha256.empty()
        || calibration_path.empty()) {
        error = "Base path provenance is incomplete";
        return false;
    }

    std::filesystem::path resolved_calibration(calibration_path);
    if (resolved_calibration.is_relative()
        && !std::filesystem::exists(resolved_calibration)) {
        resolved_calibration = std::filesystem::path(metadata_path).parent_path()
            / resolved_calibration;
    }
    std::string actual_calibration_sha256;
    if (!ComputeFileSha256(resolved_calibration.string(),
                           actual_calibration_sha256, &error)
        || actual_calibration_sha256 != target.calibration_sha256) {
        if (error.empty()) error = "camera calibration hash does not match path metadata";
        return false;
    }
    nlohmann::json calibration;
    if (!LoadJsonObject(resolved_calibration.string(), calibration, error)) return false;
    try {
        if (calibration.value("schema_version", 0) != 1
            || !calibration.value("accepted", false)
            || calibration.value("transform", std::string())
                != "d455_color_optical_to_rm75_base"
            || calibration.at("camera").at("serial").get<std::string>()
                != target.camera_serial) {
            error = "camera calibration is not an accepted matching camera-to-Base result";
            return false;
        }
    } catch (const std::exception& exception) {
        error = std::string("camera calibration field error: ") + exception.what();
        return false;
    }
    return true;
}

Eigen::Vector3d ControllerEuler(const Eigen::Matrix3d& rotation) {
    const double sy = std::clamp(-rotation(2, 0), -1.0, 1.0);
    const double ry = std::asin(sy);
    const double cy = std::cos(ry);
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    euler.y() = ry;
    if (std::abs(cy) > kEulerSingularityEpsilon) {
        euler.x() = std::atan2(rotation(2, 1), rotation(2, 2));
        euler.z() = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        euler.z() = std::atan2(-rotation(0, 1), rotation(1, 1));
    }
    return euler;
}

double RotationDifferenceDeg(const Eigen::Matrix3d& lhs,
                             const Eigen::Matrix3d& rhs) {
    return std::abs(Eigen::AngleAxisd(lhs * rhs.transpose()).angle()) * kRadToDeg;
}

void PrintVector(const char* label, const Eigen::Vector3d& value) {
    std::cout << label << ": [" << value.x() << ", " << value.y()
              << ", " << value.z() << "]\n";
}

void PrintPose(const char* label,
               const Eigen::Matrix<double, 6, 1>& pose_m_rad) {
    std::cout << label << "_m_rad: [";
    for (int i = 0; i < 6; ++i) {
        if (i) std::cout << ", ";
        std::cout << pose_m_rad[i];
    }
    std::cout << "]\n";
    std::cout << label << "_m_deg: [";
    for (int i = 0; i < 6; ++i) {
        if (i) std::cout << ", ";
        std::cout << (i < 3 ? pose_m_rad[i] : pose_m_rad[i] * kRadToDeg);
    }
    std::cout << "]\n";
}

void PrintJoints(const char* label,
                 const Eigen::Matrix<double, 7, 1>& joints_rad) {
    std::cout << label << "_deg: [";
    for (int i = 0; i < 7; ++i) {
        if (i) std::cout << ", ";
        std::cout << joints_rad[i] * kRadToDeg;
    }
    std::cout << "]\n";
}

Eigen::Matrix<double, 6, 1> ArmTipPoseForProbeTarget(
    const Eigen::Matrix<double, 6, 1>& target_probe_pose,
    const CalibratedFrameChain& frame_chain) {
    const Eigen::Matrix3d rotation_base_from_tool =
        RotationBaseFromControllerEuler(target_probe_pose.tail<3>());
    const Eigen::Matrix3d rotation_base_from_armtip =
        rotation_base_from_tool
        * frame_chain.RotationArmTipFromTool().transpose();
    Eigen::Matrix<double, 6, 1> arm_tip_pose;
    arm_tip_pose.head<3>() = target_probe_pose.head<3>()
        - rotation_base_from_armtip * frame_chain.ProbeTcpArmTipM();
    arm_tip_pose.tail<3>() = ControllerEuler(rotation_base_from_armtip);
    return arm_tip_pose;
}

bool SolveTarget(Rm75ServoPlanner& planner,
                 const Eigen::Matrix<double, 7, 1>& initial_joints,
                 const Eigen::Matrix<double, 6, 1>& desired_pose,
                 Eigen::Matrix<double, 7, 1>& target_joints,
                 Eigen::Matrix<double, 6, 1>& target_pose,
                 int& iterations,
                 std::string& error) {
    target_joints = initial_joints;
    target_pose = planner.PoseFromJoints(target_joints);
    Eigen::Matrix<double, 7, 1> previous_delta =
        Eigen::Matrix<double, 7, 1>::Zero();
    for (iterations = 0; iterations < kMaximumIkIterations; ++iterations) {
        if ((desired_pose.head<3>() - target_pose.head<3>()).norm()
                <= kIkPositionToleranceM
            && RotationDifferenceDeg(
                   RotationBaseFromControllerEuler(desired_pose.tail<3>()),
                   RotationBaseFromControllerEuler(target_pose.tail<3>()))
                <= kIkOrientationToleranceDeg) {
            return true;
        }
        const Rm75ServoPlan plan = planner.Plan(
            target_joints, target_pose, desired_pose, previous_delta);
        if (!plan.valid) {
            std::ostringstream detail;
            detail << "IK rejected: " << Rm75PlanErrorString(plan.error)
                   << ": " << plan.detail;
            error = detail.str();
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
        target_joints = plan.target_joints;
        target_pose = plan.model_pose;
        previous_delta = plan.joint_delta;
    }
    error = "IK did not converge within the fixed iteration limit";
    return false;
}

bool ValidateMoveJSegment(Rm75ServoPlanner& planner,
                          const Eigen::Matrix<double, 7, 1>& start,
                          const Eigen::Matrix<double, 7, 1>& target,
                          double& minimum_margin_deg,
                          std::string& error) {
    constexpr int kSamples = 200;
    minimum_margin_deg = std::numeric_limits<double>::infinity();
    for (int sample = 0; sample <= kSamples; ++sample) {
        const double fraction = static_cast<double>(sample) / kSamples;
        const Eigen::Matrix<double, 7, 1> joints =
            start + fraction * (target - start);
        const Eigen::Matrix<double, 6, 1> pose = planner.PoseFromJoints(joints);
        const Rm75ServoPlan check = planner.Plan(joints, pose, pose);
        if (!check.valid || check.near_joint_limit || check.near_singularity) {
            std::ostringstream detail;
            detail << "MoveJ segment rejected at " << fraction * 100.0
                   << "%: " << Rm75PlanErrorString(check.error)
                   << ": " << check.detail;
            error = detail.str();
            return false;
        }
        minimum_margin_deg = std::min(minimum_margin_deg,
                                      check.minimum_joint_margin_deg);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        PrintUsage(argv[0]);
        return 2;
    }

    const bool artery_mode = !options.artery_path_base.empty();
    const bool base_position_mode = !options.target_position_text.empty();
    const bool preserve_startup_orientation = artery_mode || base_position_mode;
    Eigen::Matrix<double, 6, 1> target_probe_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    ArteryTarget artery_target;
    std::string error;
    if (artery_mode) {
        if (!LoadArteryTarget(options.artery_path_base,
                              artery_target, error)) {
            std::cerr << "Invalid artery Base path: " << error << "\n";
            return 2;
        }
        target_probe_pose.head<3>() = artery_target.probe_tcp_target_base_m;
    } else if (base_position_mode) {
        Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
        (void)ParseVector3(options.target_position_text, target_position);
        target_probe_pose.head<3>() = target_position;
    } else {
        (void)ParsePose(options.target_pose_text, target_probe_pose);
    }
    ForceCalibration calibration;
    if (!calibration.LoadJson(options.tool_calibration, &error)) {
        std::cerr << "Invalid Probe TCP/tool calibration: " << error << "\n";
        return 2;
    }
    const CalibratedFrameChain frame_chain(calibration);
    Eigen::Matrix<double, 6, 1> target_controller_armtip_pose =
        Eigen::Matrix<double, 6, 1>::Zero();
    if (!preserve_startup_orientation) {
        target_controller_armtip_pose =
            ArmTipPoseForProbeTarget(target_probe_pose, frame_chain);
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "target_frame: rm75_base\n";
    std::cout << "target_orientation_axes: calibrated_tool\n";
    if (artery_mode) {
        std::cout << "target_source: artery_path_base\n";
        std::cout << "point_index: 0\n";
        PrintVector("surface_point_base_m",
                    artery_target.surface_point_base_m);
        PrintVector("outward_normal_base", artery_target.normal_base);
        std::cout << "standoff_m: " << kArteryStandoffM << "\n";
        PrintVector("target_probe_tcp_base_m",
                    artery_target.probe_tcp_target_base_m);
        std::cout << "target_orientation_source: startup_probe_tcp\n";
        std::cout << "camera_serial: " << artery_target.camera_serial << "\n";
        std::cout << "path_sha256: " << artery_target.path_sha256 << "\n";
        std::cout << "camera_calibration_sha256: "
                  << artery_target.calibration_sha256 << "\n";
    } else if (base_position_mode) {
        std::cout << "target_source: explicit_base_position\n";
        PrintVector("target_probe_tcp_base_m", target_probe_pose.head<3>());
        std::cout << "target_orientation_source: startup_probe_tcp\n";
    } else {
        std::cout << "target_source: explicit_pose\n";
        PrintPose("target_probe_tcp", target_probe_pose);
        PrintPose("target_controller_armtip", target_controller_armtip_pose);
    }
    PrintVector("probe_tcp_armtip_m", frame_chain.ProbeTcpArmTipM());
    std::cout << "tool_chain_verified: "
              << (calibration.tool_chain_verified ? "true" : "false") << "\n";

    if (options.inspect_transform_only) {
        if (preserve_startup_orientation) {
            std::cout << "target_orientation_available: false\n";
            std::cout << "note: startup Probe TCP orientation requires one "
                         "read-only robot state snapshot\n";
        }
        std::cout << "offline_inspection_only: true\n";
        std::cout << "robot_connection_attempted: false\n";
        return 0;
    }
    if (options.execute && !calibration.tool_chain_verified) {
        std::cerr << "BLOCKED: Probe TCP/tool chain is not independently verified\n";
        return 4;
    }

    RMCommand command(RMConnectionConfig{options.ip, options.port});
    std::cout << "Connecting to Realman controller at " << options.ip
              << ":" << options.port << "\n";
    const RMResult connect_result = command.TryConnectTCPSocket();
    if (!connect_result) {
        std::cerr << "Connection failed: " << connect_result.message << "\n";
        return 3;
    }
    if (options.execute) {
        const RMResult stop_result = command.TryStopMotion(1000);
        if (!stop_result) {
            std::cerr << "Failed to stop inherited motion: "
                      << stop_result.message << "\n";
            return 3;
        }
    }

    Eigen::Matrix<double, 7, 1> current_joints;
    Eigen::Matrix<double, 6, 1> current_controller_pose;
    int arm_err = 0;
    int sys_err = 0;
    const RMResult state_result = command.TryReadArmState(
        current_joints, current_controller_pose, arm_err, sys_err);
    if (!state_result || arm_err != 0 || sys_err != 0) {
        std::cerr << "Initial state is unavailable or reports a robot error\n";
        return 3;
    }
    PrintJoints("current_joints7", current_joints);
    PrintPose("current_controller_armtip", current_controller_pose);
    PrintVector("current_probe_tcp_base_m",
                frame_chain.ProbeTcpBase(current_controller_pose));

    if (preserve_startup_orientation) {
        target_probe_pose.tail<3>() = ControllerEuler(
            frame_chain.RotationBaseFromTool(current_controller_pose));
        target_controller_armtip_pose =
            ArmTipPoseForProbeTarget(target_probe_pose, frame_chain);
        PrintPose("target_probe_tcp", target_probe_pose);
        PrintPose("target_controller_armtip", target_controller_armtip_pose);
    }

    Rm75ServoPlannerConfig planner_config;
    // Maintenance-only warning margin; production defaults stay unchanged.
    planner_config.joint_limit_warning_deg = 3.0;
    Rm75ServoPlanner planner(planner_config);
    std::cout << "joint_limit_warning_deg: "
              << planner.Config().joint_limit_warning_deg << "\n";
    const Eigen::Matrix<double, 6, 1> current_model_pose =
        planner.PoseFromJoints(current_joints);
    const Eigen::Matrix3d current_controller_rotation =
        frame_chain.RotationBaseFromArmTip(current_controller_pose);
    const Eigen::Matrix3d current_model_rotation =
        frame_chain.RotationBaseFromArmTip(current_model_pose);
    const double model_position_error =
        (current_controller_pose.head<3>() - current_model_pose.head<3>()).norm();
    const double model_orientation_error = RotationDifferenceDeg(
        current_controller_rotation, current_model_rotation);
    std::cout << "controller_model_position_difference_mm: "
              << model_position_error * 1000.0 << "\n";
    std::cout << "controller_model_orientation_difference_deg: "
              << model_orientation_error << "\n";
    if (model_position_error > kMaximumControllerModelPositionErrorM
        || model_orientation_error > kMaximumControllerModelOrientationErrorDeg) {
        std::cerr << "Controller pose and local model disagree beyond the safety gate\n";
        return 4;
    }

    const Eigen::Vector3d controller_minus_model =
        current_controller_pose.head<3>() - current_model_pose.head<3>();
    const Eigen::Matrix3d controller_from_model_rotation =
        current_controller_rotation * current_model_rotation.transpose();
    Eigen::Matrix<double, 6, 1> desired_model_pose;
    desired_model_pose.head<3>() =
        target_controller_armtip_pose.head<3>() - controller_minus_model;
    const Eigen::Matrix3d desired_controller_rotation =
        RotationBaseFromControllerEuler(target_controller_armtip_pose.tail<3>());
    desired_model_pose.tail<3>() = ControllerEuler(
        controller_from_model_rotation.transpose() * desired_controller_rotation);

    Eigen::Matrix<double, 7, 1> target_joints;
    Eigen::Matrix<double, 6, 1> target_model_pose;
    int ik_iterations = 0;
    if (!SolveTarget(planner, current_joints, desired_model_pose,
                     target_joints, target_model_pose, ik_iterations, error)) {
        std::cerr << error << "\n";
        return 4;
    }

    const double maximum_joint_delta_deg =
        MaximumWrappedJointDeltaDeg(target_joints, current_joints);
    std::cout << "ik_iterations: " << ik_iterations << "\n";
    std::cout << "maximum_joint_delta_deg_actual: "
              << maximum_joint_delta_deg << "\n";
    PrintJoints("target_joints7", target_joints);
    if (maximum_joint_delta_deg > options.max_joint_delta_deg) {
        std::cerr << "Target exceeds max-joint-delta-deg gate\n";
        return 4;
    }

    const Eigen::Matrix3d planned_controller_rotation =
        controller_from_model_rotation
        * RotationBaseFromControllerEuler(target_model_pose.tail<3>());
    const Eigen::Vector3d planned_controller_armtip =
        target_model_pose.head<3>() + controller_minus_model;
    const Eigen::Vector3d planned_probe_position = planned_controller_armtip
        + planned_controller_rotation * frame_chain.ProbeTcpArmTipM();
    const Eigen::Matrix3d planned_tool_rotation = planned_controller_rotation
        * frame_chain.RotationArmTipFromTool();
    const double planned_position_error_mm = 1000.0
        * (planned_probe_position - target_probe_pose.head<3>()).norm();
    const double planned_orientation_error_deg = RotationDifferenceDeg(
        planned_tool_rotation,
        RotationBaseFromControllerEuler(target_probe_pose.tail<3>()));
    PrintVector("planned_probe_tcp_base_m", planned_probe_position);
    std::cout << "planned_probe_tcp_position_error_mm: "
              << planned_position_error_mm << "\n";
    std::cout << "planned_probe_tcp_orientation_error_deg: "
              << planned_orientation_error_deg << "\n";
    if (planned_position_error_mm > 0.6 || planned_orientation_error_deg > 0.2) {
        std::cerr << "IK target exceeds the fixed planning residual gate\n";
        return 4;
    }

    double minimum_margin_deg = 0.0;
    if (!ValidateMoveJSegment(planner, current_joints, target_joints,
                              minimum_margin_deg, error)) {
        std::cerr << error << "\n";
        return 4;
    }
    std::cout << "movej_segment_samples: 201\n";
    std::cout << "movej_minimum_joint_margin_deg: "
              << minimum_margin_deg << "\n";
    std::cout << "WARNING: no environment collision model is available.\n";

    if (!options.execute) {
        std::cout << "Dry-run only. Exactly zero motion commands were sent.\n";
        return 0;
    }

    std::cout << "Executing exactly one MoveJ to the validated IK target.\n";
    const RMResult move_result = command.TryMoveJ(target_joints, options.velocity);
    if (!move_result) {
        std::cerr << "MoveJ failed: " << move_result.message << "\n";
        (void)command.TryStopMotion(1000);
        return 5;
    }

    Eigen::Matrix<double, 7, 1> final_joints;
    Eigen::Matrix<double, 6, 1> final_pose;
    const RMResult final_result = command.TryReadArmState(
        final_joints, final_pose, arm_err, sys_err);
    if (!final_result || arm_err != 0 || sys_err != 0) {
        std::cerr << "Final state is unavailable or reports a robot error\n";
        (void)command.TryStopMotion(1000);
        return 5;
    }
    const Eigen::Vector3d final_probe_position = frame_chain.ProbeTcpBase(final_pose);
    const Eigen::Matrix3d final_tool_rotation =
        frame_chain.RotationBaseFromTool(final_pose);
    const double final_position_error_mm = 1000.0
        * (final_probe_position - target_probe_pose.head<3>()).norm();
    const double final_orientation_error_deg = RotationDifferenceDeg(
        final_tool_rotation,
        RotationBaseFromControllerEuler(target_probe_pose.tail<3>()));
    const double final_joint_error_deg =
        MaximumWrappedJointDeltaDeg(final_joints, target_joints);
    PrintVector("final_probe_tcp_base_m", final_probe_position);
    std::cout << "final_probe_tcp_position_error_mm: "
              << final_position_error_mm << "\n";
    std::cout << "final_probe_tcp_orientation_error_deg: "
              << final_orientation_error_deg << "\n";
    std::cout << "maximum_final_joint_error_deg: "
              << final_joint_error_deg << "\n";
    if (final_position_error_mm > options.max_final_position_error_mm
        || final_orientation_error_deg
            > options.max_final_orientation_error_deg
        || final_joint_error_deg > options.max_final_joint_error_deg) {
        std::cerr << "Final Probe TCP pose exceeds an acceptance gate\n";
        (void)command.TryStopMotion(1000);
        return 6;
    }
    std::cout << "target_reached: true\n";
    return 0;
}
