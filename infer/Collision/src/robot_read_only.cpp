#include "collision/robot_read_only.hpp"
#include <chrono>
#include <realman_command.hpp>
#include <stdexcept>

namespace uspilot::collision {
std::int64_t MonitorSteadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
struct ReadOnlyRobot::Impl {
    RMCommand command;
    int timeout_ms;
    Impl(const std::string &ip, int port, int timeout)
        : command(RMConnectionConfig{ip, port}), timeout_ms(timeout) {
        if (timeout <= 0 || timeout > 3000)
            throw std::invalid_argument("Robot read timeout must be 1..3000 ms");
        command.SetQuiet(true);
        const auto result = command.TryConnectTCPSocket(timeout);
        if (!result)
            throw std::runtime_error("Read-only robot connection: " + result.message);
    }
};
ReadOnlyRobot::ReadOnlyRobot(const std::string &ip, int port, int timeout) {
    if (port < 1 || port > 65535)
        throw std::invalid_argument("Invalid robot port");
    impl_ = std::make_unique<Impl>(ip, port, timeout);
}
ReadOnlyRobot::~ReadOnlyRobot() = default;
ReadOnlyRobotState ReadOnlyRobot::Read() {
    ReadOnlyRobotState state;
    Eigen::Matrix<double, 7, 1> joints;
    Eigen::Matrix<double, 6, 1> pose;
    state.request_started_ns = MonitorSteadyNowNs();
    const auto result =
        impl_->command.TryReadArmState(joints, pose, state.arm_error, state.system_error, impl_->timeout_ms);
    state.received_at_ns = MonitorSteadyNowNs();
    state.valid = bool(result) && state.arm_error == 0 && state.system_error == 0;
    state.error = result ? (state.valid ? "" : "robot_reports_fault") : result.message;
    if (state.valid)
        for (int i = 0; i < 7; ++i)
            state.joints["joint" + std::to_string(i + 1)] = joints[i];
    return state;
}
} // namespace uspilot::collision
