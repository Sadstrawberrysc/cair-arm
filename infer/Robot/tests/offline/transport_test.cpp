#include <realman_command.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using namespace std::chrono_literals;

bool Check(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

std::string ReadLine(int socket) {
    std::string line;
    char byte = 0;
    while (::recv(socket, &byte, 1, 0) == 1) {
        if (byte == '\n') break;
        if (byte != '\r') line += byte;
    }
    return line;
}

bool Send(int socket, const std::string& line) {
    return ::send(socket, line.data(), line.size(), MSG_NOSIGNAL)
        == static_cast<ssize_t>(line.size());
}

// Only binds loopback, with an ephemeral port. The tests never use the
// controller endpoint or connect to any external service.
class FakeController {
public:
    explicit FakeController(std::function<bool(int)> conversation) {
        listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listener_ < 0) throw std::runtime_error("cannot create loopback socket");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0
            || ::listen(listener_, 1) != 0) {
            ::close(listener_);
            throw std::runtime_error("cannot bind loopback socket");
        }
        socklen_t size = sizeof(address);
        ::getsockname(listener_, reinterpret_cast<sockaddr*>(&address), &size);
        port_ = ntohs(address.sin_port);
        worker_ = std::thread([this, conversation] {
            const int client = ::accept(listener_, nullptr, nullptr);
            if (client < 0) return;
            timeval timeout{};
            timeout.tv_sec = 2;
            ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            try {
                passed_ = conversation(client);
            } catch (const std::exception& error) {
                std::cerr << "fake controller: " << error.what() << '\n';
            }
            ::close(client);
        });
    }

    ~FakeController() {
        ::shutdown(listener_, SHUT_RDWR);
        ::close(listener_);
        if (worker_.joinable()) worker_.join();
    }

    RMConnectionConfig Config() const { return {"127.0.0.1", port_}; }

    bool Finish() {
        if (worker_.joinable()) worker_.join();
        return passed_;
    }

private:
    int listener_ = -1;
    int port_ = 0;
    bool passed_ = false;
    std::thread worker_;
};

bool TestPreCancelled(bool linear = false) {
    FakeController controller([](int socket) { return ReadLine(socket).empty(); });
    RMCommand command(controller.Config());
    command.SetQuiet(true);
    bool ok = Check(command.TryConnectTCPSocket().ok(), "pre-cancel connection");
    std::atomic<bool> cancel{true};
    const auto result = linear
        ? command.TryMoveL(Eigen::Matrix<double, 6, 1>::Zero(), 5, 1000, &cancel)
        : command.TryMoveJ(Eigen::Matrix<double, 7, 1>::Zero(), 5, 1000, &cancel);
    command.CloseTCPSocket();
    ok &= Check(!result && result.message.find("before sending") != std::string::npos,
                "pre-cancel returns failure without a command");
    ok &= Check(controller.Finish(), "pre-cancel sends neither MoveJ nor StopMotion");
    return ok;
}

bool TestMidMoveCancellation(bool acknowledge_stop, bool linear = false) {
    std::atomic<bool> cancel{false};
    FakeController controller([&](int socket) {
        const auto move = nlohmann::json::parse(ReadLine(socket));
        if (move.value("command", "") != (linear ? "movel" : "movej")) return false;
        // More than one poll period: an intermediate receive timeout must
        // not be mistaken for completion or the overall deadline.
        std::this_thread::sleep_for(150ms);
        cancel.store(true);
        const auto stop = nlohmann::json::parse(ReadLine(socket));
        if (stop.value("command", "") != "set_arm_stop") return false;
        // Late trajectory completion does not acknowledge StopMotion.
        return Send(socket, "{\"trajectory_state\":true}\n")
            && Send(socket, acknowledge_stop ? "{\"arm_stop\":true}\n"
                                             : "{\"arm_stop\":false}\n");
    });
    RMCommand command(controller.Config());
    command.SetQuiet(true);
    bool ok = Check(command.TryConnectTCPSocket().ok(), "mid-cancel connection");
    const auto begin = std::chrono::steady_clock::now();
    const auto result = linear
        ? command.TryMoveL(Eigen::Matrix<double, 6, 1>::Zero(), 5, 3000, &cancel)
        : command.TryMoveJ(Eigen::Matrix<double, 7, 1>::Zero(), 5, 3000, &cancel);
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    command.CloseTCPSocket();
    ok &= Check(!result && result.message.find("cancelled") != std::string::npos,
                "mid-move cancellation is never reported as arrival");
    ok &= Check(result.message.find(acknowledge_stop ? "StopMotion acknowledged"
                                                   : "StopMotion failed") != std::string::npos,
                "cancellation reports the actual StopMotion acknowledgement");
    ok &= Check(elapsed < 1500ms, "cancellation does not wait for the trajectory timeout");
    ok &= Check(controller.Finish(), "cancellation sends StopMotion on the move connection");
    return ok;
}

bool TestNormalMove(bool cancellable) {
    FakeController controller([](int socket) {
        const auto move = nlohmann::json::parse(ReadLine(socket));
        if (move.value("command", "") != "movej" || move.value("v", 0) != 5
            || move.value("r", -1) != 0 || move.at("joint").size() != 7) return false;
        if (!Send(socket, "{\"receive_state\":true}\n")) return false;
        std::this_thread::sleep_for(250ms);
        if (!Send(socket, "{\"trajectory_state\":true}\n")) return false;
        return ReadLine(socket).empty();
    });
    RMCommand command(controller.Config());
    command.SetQuiet(true);
    bool ok = Check(command.TryConnectTCPSocket().ok(), "normal MoveJ connection");
    std::atomic<bool> cancel{false};
    const auto result = command.TryMoveJ(Eigen::Matrix<double, 7, 1>::Zero(), 5, 1500,
                                        cancellable ? &cancel : nullptr);
    command.CloseTCPSocket();
    ok &= Check(result.ok(), "normal MoveJ waits through receive ACK and poll timeouts");
    ok &= Check(controller.Finish(), "normal MoveJ leaves StopMotion unsent");
    return ok;
}

bool TestOverallTimeout() {
    FakeController controller([](int socket) {
        const auto move = nlohmann::json::parse(ReadLine(socket));
        return move.value("command", "") == "movej" && ReadLine(socket).empty();
    });
    RMCommand command(controller.Config());
    command.SetQuiet(true);
    bool ok = Check(command.TryConnectTCPSocket().ok(), "timeout connection");
    std::atomic<bool> cancel{false};
    const auto begin = std::chrono::steady_clock::now();
    const auto result = command.TryMoveJ(Eigen::Matrix<double, 7, 1>::Zero(), 5, 250, &cancel);
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    command.CloseTCPSocket();
    ok &= Check(!result && result.code == RMErrorCode::kTimeout,
                "cancellable MoveJ keeps the overall trajectory timeout");
    ok &= Check(elapsed >= 200ms && elapsed < 1500ms, "poll slices do not extend the deadline");
    ok &= Check(controller.Finish(), "timeout alone does not pretend cancellation");
    return ok;
}

}  // namespace

int main() {
    bool ok = TestPreCancelled();
    ok &= TestMidMoveCancellation(true);
    ok &= TestMidMoveCancellation(false);
    ok &= TestPreCancelled(true);
    ok &= TestMidMoveCancellation(true, true);
    ok &= TestMidMoveCancellation(false, true);
    ok &= TestNormalMove(false);
    ok &= TestNormalMove(true);
    ok &= TestOverallTimeout();
    return ok ? 0 : 1;
}
