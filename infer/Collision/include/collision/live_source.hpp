#pragma once
#include "collision/monitor.hpp"
#include <memory>

namespace uspilot::collision {
struct LiveSourceOptions {
    std::string robot_ip;
    int robot_port = 8080;
    std::string camera_serial;
    int pixel_stride = 8;
};
// Available only with BUILD_REALSENSE_SOURCE=ON. Opens a single D455 and a
// separate state-only controller connection for standalone diagnostics.
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
