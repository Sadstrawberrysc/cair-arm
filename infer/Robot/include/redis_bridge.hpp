#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include <rm75_control.hpp>

struct RedisBridgeConfig {
    bool enabled = true;
    std::string host = "127.0.0.1";
    int port = 7777;
    std::string command_channel = "robot:command:channel";
    std::string legacy_sensor_channel = "sensor_data";
    std::string sensor_v1_channel = "robot:sensor:v1";
    std::string status_channel = "robot:status:channel";
    int reconnect_ms = 1000;
    std::size_t max_publish_queue = 32;
};

struct RedisCommandSnapshot {
    ControlIntent intent;
    // Redis v1 视觉协议的生产者身份与时间信息。入口日志和状态确认依赖
    // 这些字段；旧协议消息保持 version=0、session 为空。
    int protocol_version = 0;
    std::string session_id;
    // Optional sequence supplied by a new producer. intent.sequence is
    // replaced by a bridge-local monotonic receive sequence before the
    // snapshot is published to the control loop.
    std::uint64_t producer_sequence = 0;
    std::int64_t producer_timestamp_unix_ms = 0;
    bool has_phase_confidence = false;
    double phase_confidence = 0.0;
    std::uint64_t connection_generation = 0;
    std::int64_t received_timestamp_ns = 0;
    bool has_desired_force = false;
    // Captured under the same mutex as the command payload so the control
    // loop cannot combine an old connected=true read with a command that is
    // being invalidated by the subscriber thread.
    bool subscriber_connected = false;
    bool valid = false;
    std::string error;
};

struct RedisStatusContext {
    std::string session_id;
    std::uint64_t producer_sequence = 0;
    int phase_index = -1;
    double model_y_m = 0.0;
    double model_rz_deg = 0.0;
    double command_age_ms = 0.0;
};

struct RedisCommandDecision {
    ControlIntent intent;
    bool present = false;
    bool valid = false;
    bool fresh = false;
    double age_ms = 0.0;
    std::string hold_reason;
};

struct RedisSensorMessage {
    std::uint64_t sequence = 0;
    std::int64_t timestamp_ns = 0;
    std::int64_t source_timestamp_unix_ns = 0;
    Eigen::Matrix<double, 6, 1> raw_wrench_sensor =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> compensated_wrench_tool =
        Eigen::Matrix<double, 6, 1>::Zero();
    // Legacy consumers expect the original STL/sensor-origin coordinates
    // (not TCP-relative) and commonly display z around 0.188 m.
    Eigen::Vector3d legacy_contact_point_sensor_m = Eigen::Vector3d::Zero();
    // v1 uses the probe TCP as origin; axes are aligned with the sensor/probe
    // model coordinate system described by the calibration.
    Eigen::Vector3d contact_point_probe_m = Eigen::Vector3d::Zero();
    double contact_residual_nm = 0.0;
    double contact_point_error_m = 0.0;
    std::string contact_error;
    bool wrench_valid = false;
    bool contact_valid = false;
    bool checksum_valid = false;
    bool sensor_stale = true;
    std::string sensor_io_status;
    int sensor_io_error = 0;
    bool legacy_completion = false;
    std::string control_state;
    std::string fault;
};

class RedisBridge {
public:
    explicit RedisBridge(RedisBridgeConfig config = {});
    ~RedisBridge();

    RedisBridge(const RedisBridge&) = delete;
    RedisBridge& operator=(const RedisBridge&) = delete;

    bool Start(std::string* error = nullptr);
    void Stop();
    bool Running() const { return running_.load(); }
    bool SubscriberConnected() const { return subscriber_connected_.load(); }
    bool PublisherConnected() const { return publisher_connected_.load(); }

    RedisCommandSnapshot LatestCommand() const;
    void PublishSensor(const RedisSensorMessage& message);
    void PublishStatus(Rm75SupervisorState state,
                       const std::string& status,
                       const std::string& message,
                       const std::string& error_code,
                       std::uint64_t command_sequence,
                       const RedisStatusContext& context = {});

    static bool ParseCommandJson(const std::string& payload,
                                 RedisCommandSnapshot& output,
                                 std::string* error = nullptr);
    // Fail-closed selection used by the control loop. A Redis-enabled
    // controller receives an actionable intent only while the subscriber is
    // connected and the latest valid command is fresh.
    static RedisCommandDecision EvaluateCommandForControl(
        const RedisCommandSnapshot& command,
        std::int64_t now_monotonic_ns,
        int stale_after_ms,
        double configured_force_n);
    static std::string BuildLegacySensorJson(const RedisSensorMessage& message);
    static std::string BuildSensorV1Json(const RedisSensorMessage& message);
    static std::string BuildStatusJson(Rm75SupervisorState state,
                                       const std::string& status,
                                       const std::string& message,
                                       const std::string& error_code,
                                       std::uint64_t command_sequence,
                                       const RedisStatusContext& context = {});

private:
    struct PendingStatus {
        Rm75SupervisorState state = Rm75SupervisorState::kInitializing;
        std::string status;
        std::string message;
        std::string error_code;
        std::uint64_t command_sequence = 0;
        RedisStatusContext context;
    };

    void SubscriberLoop();
    void PublisherLoop();
    // publish_mutex_ must be held. Keeping the event descriptor under the
    // same lock as producer admission forms a Stop/Publish lifecycle barrier.
    void NotifyPublisherLocked();

    RedisBridgeConfig config_;
    std::atomic<bool> running_{false};
    std::atomic<bool> subscriber_connected_{false};
    std::atomic<bool> publisher_connected_{false};
    std::thread subscriber_thread_;
    std::thread publisher_thread_;
    std::atomic<std::uint64_t> received_command_sequence_{0};
    std::atomic<std::uint64_t> subscriber_generation_{0};

    mutable std::mutex lifecycle_mutex_;

    mutable std::mutex command_mutex_;
    RedisCommandSnapshot latest_command_;

    mutable std::mutex publish_mutex_;
    int publish_event_fd_ = -1;
    std::deque<PendingStatus> pending_statuses_;
    RedisSensorMessage pending_sensor_message_;
    bool has_pending_sensor_message_ = false;
};
