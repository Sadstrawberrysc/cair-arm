#pragma once
#include "collision/monitor.hpp"
#include "collision/robot_read_only.hpp"
#include <functional>
#include <memory>

namespace uspilot::collision {
struct LiveSourceOptions {
    std::string robot_ip;
    int robot_port = 8080;
    std::string camera_serial;
    int pixel_stride = 8;
    std::function<ReadOnlyRobotState()> feedback;
    std::filesystem::path shared_frame_directory;
    std::string calibration_id;
};
// Available only with BUILD_REALSENSE_SOURCE=ON. Opens a single D455 and a
// state-only controller connection, or an injected Robot execution feedback
// source. The injected source prevents opening a second controller connection.
class LiveMonitorSource {
  public:
    explicit LiveMonitorSource(const LiveSourceOptions &options);
    ~LiveMonitorSource();
    MonitorFrame Next();
    LiveMonitorSource(const LiveMonitorSource &) = delete;
    LiveMonitorSource &operator=(const LiveMonitorSource &) = delete;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace uspilot::collision
