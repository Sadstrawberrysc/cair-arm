#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include <realman_command.hpp>

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kMaxDeltaDeg = 10.0;

struct Options {
    std::string ip = "192.168.50.254";
    int port = 8080;
    int joint_index = 7;
    double delta_deg = 1.0;
    int velocity = 5;
    int hold_ms = 1000;
    bool execute = false;
};

void PrintUsage(const char* program) {
    std::cout
        << "Usage: " << program << " [--ip A.B.C.D] [--port PORT]\n"
        << "       [--joint-index 1..7] [--delta-deg DEG] [--velocity V]\n"
        << "       [--hold-ms MS] [--execute]\n\n"
        << "Small RM75 MoveJ delta test.\n"
        << "Default mode is dry-run. Add --execute to move the robot out and back.\n";
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
        } else if (arg == "--joint-index") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.joint_index)) return false;
        } else if (arg == "--delta-deg") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseDouble(value, options.delta_deg)) return false;
        } else if (arg == "--velocity") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.velocity)) return false;
        } else if (arg == "--hold-ms") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.hold_ms)) return false;
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
    if (options.joint_index < 1 || options.joint_index > 7) {
        std::cerr << "joint-index must be in 1..7\n";
        return false;
    }
    if (std::fabs(options.delta_deg) <= 0.0 || std::fabs(options.delta_deg) > kMaxDeltaDeg) {
        std::cerr << "delta-deg must be non-zero and no larger than "
                  << kMaxDeltaDeg << " degrees\n";
        return false;
    }
    if (options.velocity <= 0 || options.velocity > 20) {
        std::cerr << "velocity must be in 1..20 for this smoke test\n";
        return false;
    }
    if (options.hold_ms < 0 || options.hold_ms > 10000) {
        std::cerr << "hold-ms must be in 0..10000\n";
        return false;
    }
    return true;
}

template <int Size>
void PrintVectorRadDeg(const char* label, const Eigen::Matrix<double, Size, 1>& values) {
    std::cout << label << "_rad: [";
    for (int i = 0; i < values.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << values[i];
    }
    std::cout << "]\n";

    std::cout << label << "_deg: [";
    for (int i = 0; i < values.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << values[i] * kRadToDeg;
    }
    std::cout << "]\n";
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        PrintUsage(argv[0]);
        return 2;
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

    if (arm_err != 0 || sys_err != 0) {
        std::cerr << "Robot reports an error. Refusing to send motion command.\n";
        return 3;
    }

    Eigen::Matrix<double, 7, 1> target_joints = current_joints;
    const int joint_offset = options.joint_index - 1;
    target_joints[joint_offset] += options.delta_deg * kDegToRad;

    std::cout << "\nMove plan\n";
    std::cout << "joint_index: " << options.joint_index << "\n";
    std::cout << "delta_deg: " << options.delta_deg << "\n";
    std::cout << "velocity: " << options.velocity << "\n";
    std::cout << "hold_ms: " << options.hold_ms << "\n";
    PrintVectorRadDeg("target_joints7", target_joints);

    if (!options.execute) {
        std::cout << "\nDry-run only. No motion command was sent.\n";
        std::cout << "Add --execute after confirming the robot workspace is clear.\n";
        return 0;
    }

    std::cout << "\nExecuting MoveJ to target, then returning to start.\n";
    command.MoveJ(target_joints, options.velocity);
    if (options.hold_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(options.hold_ms));
    }
    command.MoveJ(current_joints, options.velocity);

    Eigen::Matrix<double, 7, 1> final_joints;
    Eigen::Matrix<double, 6, 1> final_pose;
    command.ReadArmState(final_joints, final_pose, arm_err, sys_err);
    std::cout << "\nFinal state\n";
    std::cout << "arm_err: " << arm_err << "\n";
    std::cout << "sys_err: " << sys_err << "\n";
    PrintVectorRadDeg("final_joints7", final_joints);
    return 0;
}
