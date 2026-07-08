#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <realman_command.hpp>

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

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
    bool list_presets = false;
    bool allow_multistep = false;
    bool execute = false;
};

const std::vector<Preset>& Presets() {
    static const std::vector<Preset> presets = {
        {
            "ready_verified",
            (Eigen::Matrix<double, 7, 1>() << 27.215, 17.843, 82.979, 110.588,
                                                -28.429, -7.429, 1.023).finished(),
            "Pose observed after successful RM75 10-degree joint-space tests."
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
        << "RM75 joint-pose target tool.\n"
        << "Default controller address is 192.168.50.254:8080.\n"
        << "The program first reads the current joint pose as the initial state,\n"
        << "then plans to the target joint pose from --preset or --target-deg.\n"
        << "Default mode is dry-run. Motion is sent only when --execute is set.\n";
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
            if (parsed != item.size()) return false;
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
        } else if (arg == "--preset") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.preset = value;
        } else if (arg == "--target-deg") {
            const char* value = need_value(arg);
            if (value == nullptr) return false;
            options.target_deg_text = value;
        } else if (arg == "--list-presets") {
            options.list_presets = true;
        } else if (arg == "--allow-multistep") {
            options.allow_multistep = true;
        } else if (arg == "--execute") {
            options.execute = true;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return false;
        }
    }

    if (options.ip.size() >= sizeof(RMCommand().rlm_ip)) {
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
    if (!options.preset.empty() && !options.target_deg_text.empty()) {
        std::cerr << "Use either --preset or --target-deg, not both\n";
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

    RMCommand command;
    command.rlm_port = options.port;
    std::strncpy(command.rlm_ip, options.ip.c_str(), sizeof(command.rlm_ip) - 1);
    command.rlm_ip[sizeof(command.rlm_ip) - 1] = '\0';

    std::cout << "Connecting to Realman controller at "
              << command.rlm_ip << ":" << command.rlm_port << "\n";
    command.ConnectTCPSocket();

    Eigen::Matrix<double, 7, 1> current_joints;
    Eigen::Matrix<double, 6, 1> current_pose;
    int arm_err = 0;
    int sys_err = 0;
    command.ReadArmState(current_joints, current_pose, arm_err, sys_err);

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
        command.MoveJ(waypoint_rad, options.velocity);
    }

    Eigen::Matrix<double, 7, 1> final_joints;
    Eigen::Matrix<double, 6, 1> final_pose;
    command.ReadArmState(final_joints, final_pose, arm_err, sys_err);
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
