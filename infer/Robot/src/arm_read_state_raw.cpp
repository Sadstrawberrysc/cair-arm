#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <json.hpp>

namespace {

using json = nlohmann::json;

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
        << "Read-only Realman raw JSON state test.\n"
        << "This program prints the raw controller replies and parsed joint/pose values.\n";
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

int Connect(const Options& options) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
    }

    timeval timeout{};
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(options.port);
    if (inet_pton(AF_INET, options.ip.c_str(), &addr.sin_addr) != 1) {
        close(fd);
        throw std::runtime_error("invalid IP address: " + options.ip);
    }

    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        throw std::runtime_error(std::string("connect failed: ") + std::strerror(errno));
    }
    return fd;
}

std::string SendCommand(int fd, const json& command) {
    std::string payload = command.dump() + "\r\n";
    ssize_t sent = send(fd, payload.c_str(), payload.size(), 0);
    if (sent < 0 || static_cast<size_t>(sent) != payload.size()) {
        throw std::runtime_error(std::string("send failed: ") + std::strerror(errno));
    }

    std::string response;
    char buffer[4096];
    while (true) {
        ssize_t received = recv(fd, buffer, sizeof(buffer) - 1, 0);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            throw std::runtime_error(std::string("recv failed: ") + std::strerror(errno));
        }
        if (received == 0) break;
        buffer[received] = '\0';
        response += buffer;
        if (response.find('\n') != std::string::npos) break;
    }
    return response;
}

void PrintNumberArray(const char* label, const json& values, double scale) {
    if (!values.is_array()) {
        std::cout << label << ": <missing or non-array>\n";
        return;
    }
    std::cout << label << " (" << values.size() << "): [";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) std::cout << ", ";
        if (values[i].is_number()) {
            std::cout << values[i].get<double>() * scale;
        } else {
            std::cout << values[i].dump();
        }
    }
    std::cout << "]\n";
}

void PrintParsedState(const std::string& joint_reply, const std::string& pose_reply) {
    json joint_json = json::parse(joint_reply, nullptr, false);
    json pose_json = json::parse(pose_reply, nullptr, false);

    if (joint_json.is_discarded()) {
        std::cout << "parsed joints: <invalid json>\n";
    } else {
        PrintNumberArray("joints_rad", joint_json.value("joint", json::array()), 3.14159265358979323846 / 180000.0);
    }

    if (pose_json.is_discarded()) {
        std::cout << "parsed pose: <invalid json>\n";
        return;
    }

    if (pose_json.contains("arm_state") && pose_json["arm_state"].contains("pose")) {
        const auto& pose = pose_json["arm_state"]["pose"];
        if (pose.is_array()) {
            std::cout << "pose_m_rad (" << pose.size() << "): [";
            for (size_t i = 0; i < pose.size(); ++i) {
                if (i > 0) std::cout << ", ";
                if (!pose[i].is_number()) {
                    std::cout << pose[i].dump();
                } else if (i < 3) {
                    std::cout << pose[i].get<double>() / 1000000.0;
                } else {
                    std::cout << pose[i].get<double>() / 1000.0;
                }
            }
            std::cout << "]\n";
            return;
        }
    }
    std::cout << "pose_m_rad: <arm_state.pose missing>\n";
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        PrintUsage(argv[0]);
        return 2;
    }

    try {
        std::cout << "Connecting to Realman controller at "
                  << options.ip << ":" << options.port << "\n";
        int fd = Connect(options);
        std::cout << "Robot connect!\n";

        for (int index = 0; index < options.repeat; ++index) {
            std::string joint_reply = SendCommand(fd, {{"command", "get_joint_degree"}});
            std::string pose_reply = SendCommand(fd, {{"command", "get_current_arm_state"}});

            std::cout << "\nSample " << (index + 1) << "/" << options.repeat << "\n";
            std::cout << "raw get_joint_degree: " << joint_reply;
            if (joint_reply.empty() || joint_reply.back() != '\n') std::cout << "\n";
            std::cout << "raw get_current_arm_state: " << pose_reply;
            if (pose_reply.empty() || pose_reply.back() != '\n') std::cout << "\n";
            PrintParsedState(joint_reply, pose_reply);

            if (index + 1 < options.repeat && options.interval_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(options.interval_ms));
            }
        }

        close(fd);
        std::cout << "\nRead-only raw state test finished. No motion command was sent.\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "ERROR: " << exc.what() << "\n";
        return 1;
    }
}
