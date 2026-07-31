#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include <realman_command.hpp>

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

struct Options {
    std::string ip = "192.168.50.254";
    int port = 8080;
    int repeat = 1;
    int interval_ms = 1000;
};

void PrintUsage(const char* program) {
    std::cout
        << "Usage: " << program << " [--ip A.B.C.D] [--port PORT] "
        << "[--repeat N] [--interval-ms MS]\n\n"
        << "Read-only Realman arm smoke test.\n"
        << "This program connects to the controller and reads joint/pose state only.\n";
}

bool ParseInt(const char* text, int& value) {
    try {
        size_t parsed = 0;
        int parsed_value = std::stoi(text, &parsed, 10);
        if (parsed != std::strlen(text)) {
            return false;
        }
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
        } else if (arg == "--repeat") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.repeat)) return false;
        } else if (arg == "--interval-ms") {
            const char* value = need_value(arg);
            if (value == nullptr || !ParseInt(value, options.interval_ms)) return false;
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
    if (options.repeat <= 0) {
        std::cerr << "repeat must be positive\n";
        return false;
    }
    if (options.interval_ms < 0) {
        std::cerr << "interval-ms cannot be negative\n";
        return false;
    }
    return true;
}

template <int Size>
void PrintVector(const char* label, const Eigen::Matrix<double, Size, 1>& values) {
    std::cout << label << ": [";
    for (int i = 0; i < values.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << values[i];
    }
    std::cout << "]\n";
}

std::string FormatVector3(const Eigen::Vector3d& values) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4)
           << values[0] << "," << values[1] << "," << values[2];
    return stream.str();
}

void PrintPoseFriendly(const Eigen::Matrix<double, 6, 1>& pose) {
    Eigen::Vector3d position_cm = pose.head<3>() * 100.0;
    Eigen::Vector3d rotation_deg = pose.tail<3>() * kRadToDeg;

    PrintVector("position_cm", position_cm);
    PrintVector("rotation_deg", rotation_deg);
    std::cout << "current_as_target_position_cm: \""
              << FormatVector3(position_cm) << "\"\n";
    std::cout << "copyable_absolute_position_dry_run: ./rm75_servoj_diagnostic "
              << "--target-position-cm \"" << FormatVector3(position_cm)
              << "\"\n";
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
    const RMResult connect_result = command.TryConnectTCPSocket();
    if (!connect_result) {
        std::cerr << "Robot connection failed: "
                  << connect_result.message << '\n';
        return 3;
    }

    for (int index = 0; index < options.repeat; ++index) {
        Eigen::Matrix<double, 7, 1> joints =
            Eigen::Matrix<double, 7, 1>::Constant(
                std::numeric_limits<double>::quiet_NaN());
        Eigen::Matrix<double, 6, 1> pose =
            Eigen::Matrix<double, 6, 1>::Constant(
                std::numeric_limits<double>::quiet_NaN());
        int arm_err = 0;
        int sys_err = 0;

        const RMResult read_result = command.TryReadArmState(
            joints, pose, arm_err, sys_err, 2000);
        if (!read_result) {
            std::cerr << "Robot state read failed at sample "
                      << (index + 1) << ": " << read_result.message << '\n';
            return 4;
        }
        if (arm_err != 0 || sys_err != 0) {
            std::cerr << "Robot reported an error at sample " << (index + 1)
                      << ": arm_err=" << arm_err
                      << " sys_err=" << sys_err << '\n';
            return 5;
        }

        std::cout << "\nSample " << (index + 1) << "/" << options.repeat << "\n";
        std::cout << "arm_err: " << arm_err << "\n";
        std::cout << "sys_err: " << sys_err << "\n";
        PrintVector("joints7_rad", joints);
        PrintVector("pose_m_rad", pose);
        PrintPoseFriendly(pose);

        if (index + 1 < options.repeat && options.interval_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.interval_ms));
        }
    }

    std::cout << "\nRead-only state test finished. No motion command was sent.\n";
    return 0;
}
