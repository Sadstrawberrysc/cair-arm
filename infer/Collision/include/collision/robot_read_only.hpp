#pragma once
#include "collision/robot_model.hpp"
#include <cstdint>
#include <memory>

namespace uspilot::collision {
struct ReadOnlyRobotState {
    JointPositions joints;
    std::int64_t request_started_ns = 0;
    std::int64_t received_at_ns = 0;
    int arm_error = 0, system_error = 0;
    bool valid = false;
    std::string error;
};
// A deliberately restricted facade: only connect/query/close. No background
// servo sender, stop guard, high-speed-mode setup or motion method is exposed.
class ReadOnlyRobot {
  public:
    ReadOnlyRobot(const std::string &ip, int port, int timeout_ms = 250);
    ~ReadOnlyRobot();
    ReadOnlyRobotState Read();
    ReadOnlyRobot(const ReadOnlyRobot &) = delete;
    ReadOnlyRobot &operator=(const ReadOnlyRobot &) = delete;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
std::int64_t MonitorSteadyNowNs();
} // namespace uspilot::collision
