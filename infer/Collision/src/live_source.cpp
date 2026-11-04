#include "collision/live_source.hpp"
#include "collision/robot_read_only.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace uspilot::collision {
struct LiveMonitorSource::Impl {
    LiveSourceOptions options;
    ReadOnlyRobot robot;
    rs2::pipeline pipeline;
    rs2::pointcloud pointcloud;
    std::atomic<bool> stop{false};
    std::thread feedback;
    std::mutex mutex;
    std::deque<ReadOnlyRobotState> history;
    bool started = false;
    explicit Impl(const LiveSourceOptions &o) : options(o), robot(o.robot_ip, o.robot_port) {
        if (o.camera_serial.empty() || o.pixel_stride < 1 || o.pixel_stride > 32)
            throw std::invalid_argument("Explicit camera serial/valid stride required");
        rs2::config config;
        config.enable_device(o.camera_serial);
        config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
        config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
        auto profile = pipeline.start(config);
        started = true;
        try {
            for (auto sensor : profile.get_device().query_sensors()) {
                if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
                    sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1);
                if (sensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE))
                    sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
            }
            feedback = std::thread([this] {
                while (!stop.load()) {
                    auto state = robot.Read();
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        history.push_back(std::move(state));
                        if (history.size() > 128)
                            history.pop_front();
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            });
        } catch (...) {
            pipeline.stop();
            started = false;
            throw;
        }
    }
    ~Impl() {
        stop.store(true);
        if (feedback.joinable())
            feedback.join();
        if (started) {
            try {
                pipeline.stop();
            } catch (...) {
            }
        }
    }
};
LiveMonitorSource::LiveMonitorSource(const LiveSourceOptions &o) {
    if (o.camera_serial.empty() || o.robot_ip.empty() || o.pixel_stride < 1 || o.pixel_stride > 32)
        throw std::invalid_argument("Explicit camera serial, robot IP and valid pixel stride required");
    impl_ = std::make_unique<Impl>(o);
}
LiveMonitorSource::~LiveMonitorSource() = default;
MonitorFrame LiveMonitorSource::Next() {
    auto &p = *impl_;
    const auto frames = p.pipeline.wait_for_frames(1000);
    const auto received = MonitorSteadyNowNs();
    const auto unix_now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::system_clock::now().time_since_epoch())
                              .count();
    const auto depth = frames.get_depth_frame();
    const auto color = frames.get_color_frame();
    if (!depth || !color)
        throw std::runtime_error("Incomplete RGB-D frameset");
    MonitorFrame result;
    result.camera_serial = p.options.camera_serial;
    result.sequence = depth.get_frame_number();
    result.received_at_ns = received;
    const double timestamp_ns = depth.get_timestamp() * 1e6;
    const auto domain = depth.get_frame_timestamp_domain();
    result.capture_time_trusted =
        (domain == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME || domain == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME) &&
        std::isfinite(timestamp_ns) && timestamp_ns >= 0 && timestamp_ns < 9e18;
    if (result.capture_time_trusted) {
        const auto age = unix_now - static_cast<std::int64_t>(timestamp_ns);
        result.captured_at_ns = received - age;
        if (age < 0 || age > 500000000)
            result.capture_time_trusted = false;
    } else
        result.captured_at_ns = received;
    // Use only a feedback response already available and close to exposure.
    // Its device sampling instant is not supplied by the controller, so the
    // RTT and host receive offset remain diagnostic timing uncertainties.
    {
        std::lock_guard<std::mutex> lock(p.mutex);
        const ReadOnlyRobotState *best = nullptr;
        for (const auto &s : p.history) {
            if (!best || std::abs(s.received_at_ns - result.captured_at_ns) <
                             std::abs(best->received_at_ns - result.captured_at_ns))
                best = &s;
        }
        if (best) {
            result.joints = best->joints;
            result.robot_at_ns = best->received_at_ns;
            result.robot_valid = best->valid && best->received_at_ns - best->request_started_ns <= 100000000;
        }
        if (!p.history.empty() && !p.history.back().valid)
            result.robot_valid = false;
    }
    p.pointcloud.map_to(color);
    const auto points = p.pointcloud.calculate(depth);
    const auto *vertices = points.get_vertices();
    const auto *uv = points.get_texture_coordinates();
    const auto extrinsics = depth.get_profile().get_extrinsics_to(color.get_profile());
    result.ray_origin_color_m = {extrinsics.translation[0], extrinsics.translation[1],
                                 extrinsics.translation[2]};
    const auto *rgb = static_cast<const unsigned char *>(color.get_data());
    const int width = depth.get_width(), height = depth.get_height(), cw = color.get_width(),
              ch = color.get_height();
    for (int y = 0; y < height; y += p.options.pixel_stride)
        for (int x = 0; x < width; x += p.options.pixel_stride) {
            const auto index = std::size_t(y) * width + x;
            const float raw[3] = {vertices[index].x, vertices[index].y, vertices[index].z};
            if (!std::isfinite(raw[2]) || raw[2] <= 0)
                continue;
            float optical[3];
            rs2_transform_point_to_point(optical, &extrinsics, raw);
            result.points_color_m.emplace_back(optical[0], optical[1], optical[2]);
            const int u = std::isfinite(uv[index].u) ? int(uv[index].u * cw) : -1;
            const int v = std::isfinite(uv[index].v) ? int(uv[index].v * ch) : -1;
            std::array<unsigned char, 3> pixel{100, 180, 240};
            if (u >= 0 && v >= 0 && u < cw && v < ch) {
                const auto offset = v * color.get_stride_in_bytes() + u * 3;
                pixel = {rgb[offset], rgb[offset + 1], rgb[offset + 2]};
            }
            result.colors.push_back(pixel);
        }
    return result;
}
} // namespace uspilot::collision
