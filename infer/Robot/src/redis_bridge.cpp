// Redis command/status/sensor I/O. The controller thread only exchanges
// snapshots with this module and never performs blocking Redis operations.
#include <redis_bridge.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <poll.h>
#include <sys/select.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <vector>

#include <hiredis/hiredis.h>
#include <json.hpp>

namespace {

using json = nlohmann::json;

struct RedisContextDeleter {
    void operator()(redisContext* context) const {
        if (context != nullptr) redisFree(context);
    }
};

struct RedisReplyDeleter {
    void operator()(redisReply* reply) const {
        if (reply != nullptr) freeReplyObject(reply);
    }
};

using ContextPtr = std::unique_ptr<redisContext, RedisContextDeleter>;
using ReplyPtr = std::unique_ptr<redisReply, RedisReplyDeleter>;

std::int64_t MonotonicNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::int64_t UnixNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void WaitForReconnectOrStop(const std::atomic<bool>& running, int delay_ms) {
    constexpr int kStopPollMs = 25;
    int remaining_ms = delay_ms;
    while (running.load() && remaining_ms > 0) {
        const int step_ms = std::min(remaining_ms, kStopPollMs);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
        remaining_ms -= step_ms;
    }
}

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) *error = message;
}

ContextPtr Connect(const RedisBridgeConfig& config) {
    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    ContextPtr context(redisConnectWithTimeout(config.host.c_str(), config.port, timeout));
    if (!context || context->err != 0) return {};
    timeval io_timeout{};
    io_timeout.tv_sec = 0;
    io_timeout.tv_usec = 200000;
    if (redisSetTimeout(context.get(), io_timeout) != REDIS_OK) return {};
    const int descriptor_flags = fcntl(context->fd, F_GETFD);
    if (descriptor_flags >= 0) {
        fcntl(context->fd, F_SETFD, descriptor_flags | FD_CLOEXEC);
    }
    redisEnableKeepAlive(context.get());
    return context;
}

bool JsonBoolean(const json& input, bool& output) {
    if (input.is_boolean()) {
        output = input.get<bool>();
        return true;
    }
    if (input.is_number_integer()) {
        const int value = input.get<int>();
        if (value == 0 || value == 1) {
            output = value == 1;
            return true;
        }
    }
    return false;
}

json VectorJson(const Eigen::Vector3d& value) {
    return json::array({value.x(), value.y(), value.z()});
}

json WrenchJson(const Eigen::Matrix<double, 6, 1>& wrench) {
    return {{"force_n", VectorJson(wrench.head<3>())},
            {"torque_nm", VectorJson(wrench.tail<3>())}};
}

bool ValidSubscribeReply(const redisReply* reply,
                         const std::string& expected_channel) {
    return reply != nullptr
        && reply->type == REDIS_REPLY_ARRAY
        && reply->elements >= 3
        && reply->element[0] != nullptr
        && reply->element[0]->type == REDIS_REPLY_STRING
        && reply->element[0]->str != nullptr
        && std::strcmp(reply->element[0]->str, "subscribe") == 0
        && reply->element[1] != nullptr
        && reply->element[1]->type == REDIS_REPLY_STRING
        && reply->element[1]->str != nullptr
        && expected_channel == reply->element[1]->str
        && reply->element[2] != nullptr
        && reply->element[2]->type == REDIS_REPLY_INTEGER;
}

}  // namespace

RedisBridge::RedisBridge(RedisBridgeConfig config) : config_(std::move(config)) {}

RedisBridge::~RedisBridge() {
    Stop();
}

bool RedisBridge::Start(std::string* error) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (!config_.enabled) return true;
    if (config_.host.empty() || config_.port <= 0 || config_.port > 65535
        || config_.reconnect_ms <= 0 || config_.max_publish_queue == 0) {
        SetError(error, "invalid Redis host, port, reconnect interval or queue size");
        return false;
    }
    if (running_.load()) return true;
    {
        std::lock_guard<std::mutex> publish_lock(publish_mutex_);
        pending_statuses_.clear();
        has_pending_sensor_message_ = false;
        publish_event_fd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
        if (publish_event_fd_ < 0) {
            SetError(error, std::string("cannot create Redis publisher event: ")
                                + std::strerror(errno));
            return false;
        }
        running_.store(true);
    }
    try {
        subscriber_thread_ = std::thread(&RedisBridge::SubscriberLoop, this);
        publisher_thread_ = std::thread(&RedisBridge::PublisherLoop, this);
    } catch (const std::exception& exception) {
        running_.store(false);
        {
            std::lock_guard<std::mutex> publish_lock(publish_mutex_);
            NotifyPublisherLocked();
        }
        if (subscriber_thread_.joinable()) subscriber_thread_.join();
        if (publisher_thread_.joinable()) publisher_thread_.join();
        {
            std::lock_guard<std::mutex> publish_lock(publish_mutex_);
            close(publish_event_fd_);
            publish_event_fd_ = -1;
            pending_statuses_.clear();
            has_pending_sensor_message_ = false;
        }
        subscriber_connected_.store(false);
        publisher_connected_.store(false);
        SetError(error, std::string("cannot start Redis I/O threads: ")
                            + exception.what());
        return false;
    }
    return true;
}

void RedisBridge::Stop() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (!running_.exchange(false)) return;
    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        latest_command_ = RedisCommandSnapshot{};
        latest_command_.connection_generation = subscriber_generation_.load();
        latest_command_.received_timestamp_ns = MonotonicNowNs();
        latest_command_.error = "Redis bridge stopped";
        subscriber_connected_.store(false);
    }
    {
        std::lock_guard<std::mutex> publish_lock(publish_mutex_);
        NotifyPublisherLocked();
    }
    if (subscriber_thread_.joinable()) subscriber_thread_.join();
    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        latest_command_ = RedisCommandSnapshot{};
        latest_command_.connection_generation = subscriber_generation_.load();
        latest_command_.received_timestamp_ns = MonotonicNowNs();
        latest_command_.error = "Redis bridge stopped";
        subscriber_connected_.store(false);
    }
    if (publisher_thread_.joinable()) publisher_thread_.join();
    {
        std::lock_guard<std::mutex> publish_lock(publish_mutex_);
        if (publish_event_fd_ >= 0) {
            close(publish_event_fd_);
            publish_event_fd_ = -1;
        }
        // Never carry telemetry from a stopped lifecycle into a later Start.
        pending_statuses_.clear();
        has_pending_sensor_message_ = false;
    }
    subscriber_connected_.store(false);
    publisher_connected_.store(false);
}

RedisCommandSnapshot RedisBridge::LatestCommand() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return latest_command_;
}

void RedisBridge::NotifyPublisherLocked() {
    if (publish_event_fd_ < 0) return;
    const std::uint64_t notification = 1;
    ssize_t written;
    do {
        written = write(publish_event_fd_,
                        &notification,
                        sizeof(notification));
    } while (written < 0 && errno == EINTR);
    // EAGAIN means the event counter is already saturated, which is still a
    // pending notification. Other errors are handled by the publisher's
    // bounded poll/retry path rather than the 10 ms producer path.
}

void RedisBridge::PublishSensor(const RedisSensorMessage& message) {
    if (!config_.enabled) return;
    std::lock_guard<std::mutex> lock(publish_mutex_);
    if (!running_.load()) return;
    // Sensor telemetry is latest-value data. Replacing an unpublished sample
    // prevents backpressure and keeps JSON serialization out of the 10 ms
    // control path.
    pending_sensor_message_ = message;
    has_pending_sensor_message_ = true;
    NotifyPublisherLocked();
}

void RedisBridge::PublishStatus(Rm75SupervisorState state,
                                const std::string& status,
                                const std::string& message,
                                const std::string& error_code,
                                std::uint64_t command_sequence,
                                const RedisStatusContext& context) {
    if (!config_.enabled) return;
    std::lock_guard<std::mutex> lock(publish_mutex_);
    if (!running_.load()) return;
    while (pending_statuses_.size() >= config_.max_publish_queue) {
        pending_statuses_.pop_front();
    }
    pending_statuses_.push_back(
        {state, status, message, error_code, command_sequence, context});
    NotifyPublisherLocked();
}

bool RedisBridge::ParseCommandJson(const std::string& payload,
                                   RedisCommandSnapshot& output,
                                   std::string* error) {
    output = RedisCommandSnapshot{};
    const json input = json::parse(payload, nullptr, false);
    if (input.is_discarded() || !input.is_object()) {
        SetError(error, "command is not a JSON object");
        return false;
    }
    try {
        if (input.contains("version")) {
            if (!input["version"].is_number_integer()
                || input["version"].get<int>() != 1) {
                SetError(error, "unsupported command protocol version");
                return false;
            }
            output.protocol_version = 1;
            if (!input.contains("session_id") || !input["session_id"].is_string()
                || input["session_id"].get<std::string>().empty()) {
                SetError(error, "version-1 command requires session_id");
                return false;
            }
            output.session_id = input["session_id"].get<std::string>();
            if (!input.contains("timestamp_unix_ms")
                || !input["timestamp_unix_ms"].is_number_integer()) {
                SetError(error, "version-1 command requires timestamp_unix_ms");
                return false;
            }
            output.producer_timestamp_unix_ms =
                input["timestamp_unix_ms"].get<std::int64_t>();
            if (input.contains("phase_confidence")) {
                if (!input["phase_confidence"].is_number()) {
                    SetError(error, "phase_confidence must be numeric");
                    return false;
                }
                output.phase_confidence = input["phase_confidence"].get<double>();
                if (!std::isfinite(output.phase_confidence)
                    || output.phase_confidence < 0.0
                    || output.phase_confidence > 1.0) {
                    SetError(error, "phase_confidence is outside 0..1");
                    return false;
                }
                output.has_phase_confidence = true;
            }
        }
        if (!input.contains("parameters") || !input["parameters"].is_object()) {
            SetError(error, "command has no parameters object");
            return false;
        }
        const auto& parameters = input["parameters"];
        if (!parameters.contains("y") || !parameters["y"].is_number()
            || !parameters.contains("rz") || !parameters["rz"].is_number()) {
            SetError(error, "command parameters y/rz must be numbers");
            return false;
        }
        ControlIntent intent;
        intent.model_y_m = parameters["y"].get<double>();
        intent.model_rz_deg = parameters["rz"].get<double>();
        if (!std::isfinite(intent.model_y_m) || std::abs(intent.model_y_m) > 0.2) {
            SetError(error, "command y is outside -0.2..0.2 m");
            return false;
        }
        if (!std::isfinite(intent.model_rz_deg) || std::abs(intent.model_rz_deg) > 180.0) {
            SetError(error, "command rz is outside -180..180 deg");
            return false;
        }

        bool terminate = false;
        if (!input.contains("terminate")
            || !JsonBoolean(input["terminate"], terminate)) {
            SetError(error, "command terminate must be bool or 0/1");
            return false;
        }
        // The deployed pose_pred producers predate action_state/phase_idx.
        // Preserve their semantics: a non-terminate inference result is an
        // actionable correction. New producers can still explicitly hold by
        // sending action_state=false.
        bool action_enabled = !terminate;
        if (input.contains("action_state")
            && !JsonBoolean(input["action_state"], action_enabled)) {
            SetError(error, "command action_state must be bool or 0/1 when present");
            return false;
        }
        intent.terminate = terminate;
        intent.action_enabled = action_enabled;
        intent.phase_index = -1;
        if (input.contains("phase_idx")) {
            if (!input["phase_idx"].is_number_integer()) {
                SetError(error, "command phase_idx must be an integer");
                return false;
            }
            const std::int64_t phase = input["phase_idx"].get<std::int64_t>();
            if (phase < -1 || phase > 2) {
                SetError(error, "command phase_idx is outside -1..2");
                return false;
            }
            intent.phase_index = static_cast<int>(phase);
        }
        if (parameters.contains("desired_force_n")) {
            if (!parameters["desired_force_n"].is_number()) {
                SetError(error, "desired_force_n must be numeric");
                return false;
            }
            intent.desired_force_n = parameters["desired_force_n"].get<double>();
            if (!std::isfinite(intent.desired_force_n)
                || intent.desired_force_n < -3.0
                || intent.desired_force_n > -0.1) {
                SetError(error, "desired_force_n is outside -3..-0.1 N");
                return false;
            }
            output.has_desired_force = true;
        }
        intent.sequence = 0;
        if (input.contains("sequence")) {
            if (!input["sequence"].is_number_integer()) {
                SetError(error, "command sequence must be a non-negative integer");
                return false;
            }
            if (input["sequence"].is_number_unsigned()) {
                intent.sequence = input["sequence"].get<std::uint64_t>();
            } else {
                const std::int64_t sequence =
                    input["sequence"].get<std::int64_t>();
                if (sequence < 0) {
                    SetError(error,
                             "command sequence must be a non-negative integer");
                    return false;
                }
                intent.sequence = static_cast<std::uint64_t>(sequence);
            }
        }
        if (output.protocol_version == 1 && intent.sequence == 0) {
            SetError(error, "version-1 command sequence must be positive");
            return false;
        }
        output.intent = intent;
        output.producer_sequence = intent.sequence;
        output.received_timestamp_ns = MonotonicNowNs();
        output.valid = true;
        output.error.clear();
        return true;
    } catch (const std::exception& exception) {
        SetError(error, std::string("invalid command field: ") + exception.what());
        return false;
    }
}

RedisCommandDecision RedisBridge::EvaluateCommandForControl(
    const RedisCommandSnapshot& command,
    std::int64_t now_monotonic_ns,
    int stale_after_ms,
    double configured_force_n) {
    RedisCommandDecision decision;
    decision.intent.desired_force_n = configured_force_n;
    decision.present = command.received_timestamp_ns != 0;
    if (decision.present) {
        const std::int64_t age_ns = std::max<std::int64_t>(
            0, now_monotonic_ns - command.received_timestamp_ns);
        decision.age_ms = static_cast<double>(age_ns) / 1'000'000.0;
        decision.fresh = stale_after_ms > 0
            && decision.age_ms <= static_cast<double>(stale_after_ms);
    }

    if (!command.subscriber_connected) {
        decision.hold_reason = "redis_subscriber_disconnected";
    } else if (!decision.present) {
        decision.hold_reason = "redis_command_not_received";
    } else if (!command.valid) {
        decision.hold_reason = "invalid_redis_command";
    } else if (!decision.fresh) {
        decision.hold_reason = "stale_redis_command";
    } else {
        decision.intent = command.intent;
        if (!command.has_desired_force) {
            decision.intent.desired_force_n = configured_force_n;
        }
        decision.valid = true;
    }
    return decision;
}

std::string RedisBridge::BuildLegacySensorJson(const RedisSensorMessage& message) {
    const Eigen::Vector3d point = message.contact_valid
        ? message.legacy_contact_point_sensor_m
        : Eigen::Vector3d::Zero();
    const double force_z = message.legacy_completion
        ? 1.0
        : (message.wrench_valid ? message.compensated_wrench_tool.z() : 0.0);
    return json::array({point.x(), point.y(), point.z(), force_z}).dump();
}

std::string RedisBridge::BuildSensorV1Json(const RedisSensorMessage& message) {
    json output;
    output["version"] = 1;
    output["sequence"] = message.sequence;
    output["timestamp_monotonic_ns"] = message.timestamp_ns;
    output["source_timestamp_unix_ns"] = message.source_timestamp_unix_ns;
    output["published_unix_ms"] = UnixNowMs();
    output["units"] = {{"force", "N"}, {"torque", "N*m"}, {"length", "m"}};
    output["valid"] = message.wrench_valid;
    output["sensor"] = {
        {"checksum_valid", message.checksum_valid},
        {"stale", message.sensor_stale},
        {"io_status", message.sensor_io_status},
        {"io_error", message.sensor_io_error}};
    output["raw_wrench_sensor"] = WrenchJson(message.raw_wrench_sensor);
    output["compensated_wrench_tool"] = WrenchJson(message.compensated_wrench_tool);
    output["contact"] = {
        {"valid", message.contact_valid},
        {"frame", "probe_tcp_sensor_aligned"},
        {"point_probe_m", VectorJson(message.contact_point_probe_m)},
        {"wrench_residual_nm", message.contact_residual_nm},
        {"equivalent_point_error_m", message.contact_point_error_m},
        {"error", message.contact_error.empty()
                      ? json(nullptr)
                      : json(message.contact_error)}};
    output["control_state"] = message.control_state;
    output["fault"] = message.fault.empty() ? json(nullptr) : json(message.fault);
    return output.dump();
}

std::string RedisBridge::BuildStatusJson(Rm75SupervisorState state,
                                         const std::string& status,
                                         const std::string& message,
                                         const std::string& error_code,
                                         std::uint64_t command_sequence,
                                         const RedisStatusContext& context) {
    json output;
    output["version"] = 1;
    output["status"] = status;
    output["message"] = message;
    output["state"] = ToString(state);
    output["error_code"] = error_code.empty() ? json(nullptr) : json(error_code);
    output["command_sequence"] = command_sequence;
    output["session_id"] = context.session_id.empty()
        ? json(nullptr) : json(context.session_id);
    output["producer_sequence"] = context.producer_sequence;
    output["phase_idx"] = context.phase_index;
    output["parameters"] = {{"y", context.model_y_m}, {"rz", context.model_rz_deg}};
    output["command_age_ms"] = context.command_age_ms;
    output["timestamp_unix_ms"] = UnixNowMs();
    return output.dump();
}

void RedisBridge::SubscriberLoop() {
    std::uint64_t last_producer_sequence = 0;
    while (running_.load()) {
        ContextPtr context = Connect(config_);
        if (!context) {
            subscriber_connected_.store(false);
            WaitForReconnectOrStop(running_, config_.reconnect_ms);
            continue;
        }
        ReplyPtr subscribe_reply(static_cast<redisReply*>(redisCommand(
            context.get(), "SUBSCRIBE %s", config_.command_channel.c_str())));
        if (!ValidSubscribeReply(subscribe_reply.get(),
                                 config_.command_channel)) {
            subscriber_connected_.store(false);
            WaitForReconnectOrStop(running_, config_.reconnect_ms);
            continue;
        }
        if (!running_.load()) break;
        const std::uint64_t connection_generation =
            subscriber_generation_.fetch_add(1) + 1;
        {
            // A subscription connection is a command-generation boundary.
            // Never let a command received on a previous connection become
            // actionable again merely because reconnect completed before its
            // stale timeout elapsed.
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!running_.load()) break;
            latest_command_ = RedisCommandSnapshot{};
            latest_command_.connection_generation = connection_generation;
            latest_command_.subscriber_connected = true;
            latest_command_.error =
                "new Redis subscription requires a new command";
            subscriber_connected_.store(true);
        }

        while (running_.load() && context->err == 0) {
            if (context->fd < 0 || context->fd >= FD_SETSIZE) break;
            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(context->fd, &read_set);
            timeval timeout{};
            timeout.tv_sec = 0;
            timeout.tv_usec = 200000;
            const int ready = select(context->fd + 1, &read_set, nullptr, nullptr, &timeout);
            if (ready < 0) break;
            if (ready == 0) continue;

            redisReply* raw_reply = nullptr;
            if (redisGetReply(context.get(), reinterpret_cast<void**>(&raw_reply)) != REDIS_OK) {
                break;
            }
            ReplyPtr reply(raw_reply);
            if (reply && reply->type == REDIS_REPLY_ERROR) break;
            if (!reply || reply->type != REDIS_REPLY_ARRAY || reply->elements < 3
                || reply->element[0] == nullptr || reply->element[0]->str == nullptr
                || std::strcmp(reply->element[0]->str, "message") != 0
                || reply->element[2] == nullptr || reply->element[2]->str == nullptr) {
                continue;
            }
            RedisCommandSnapshot parsed;
            std::string parse_error;
            if (ParseCommandJson(reply->element[2]->str, parsed, &parse_error)) {
                parsed.connection_generation = connection_generation;
                parsed.subscriber_connected = true;
                if (parsed.producer_sequence != 0
                    && parsed.producer_sequence <= last_producer_sequence) {
                    parsed.valid = false;
                    parsed.error = "command producer sequence is stale or replayed";
                    parsed.received_timestamp_ns = MonotonicNowNs();
                    {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        if (!running_.load()) break;
                        latest_command_ = parsed;
                    }
                    PublishStatus(Rm75SupervisorState::kHold,
                                  "error",
                                  "Stale Redis command; motion held",
                                  "stale_redis_command_sequence",
                                  0);
                    continue;
                }
                if (parsed.producer_sequence != 0) {
                    last_producer_sequence = parsed.producer_sequence;
                }
                parsed.intent.sequence =
                    received_command_sequence_.fetch_add(1) + 1;
                std::lock_guard<std::mutex> lock(command_mutex_);
                if (!running_.load()) break;
                latest_command_ = parsed;
            } else {
                parsed.connection_generation = connection_generation;
                parsed.subscriber_connected = true;
                parsed.valid = false;
                parsed.error = parse_error;
                parsed.received_timestamp_ns = MonotonicNowNs();
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    if (!running_.load()) break;
                    latest_command_ = parsed;
                }
                PublishStatus(Rm75SupervisorState::kHold,
                              "error",
                              "Invalid Redis command; motion held",
                              "invalid_redis_command",
                              0);
            }
        }
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            latest_command_ = RedisCommandSnapshot{};
            latest_command_.connection_generation = connection_generation;
            latest_command_.received_timestamp_ns = MonotonicNowNs();
            latest_command_.subscriber_connected = false;
            latest_command_.error = "Redis command subscription disconnected";
            subscriber_connected_.store(false);
        }
        if (running_.load()) {
            WaitForReconnectOrStop(running_, config_.reconnect_ms);
        }
    }
}

void RedisBridge::PublisherLoop() {
    ContextPtr context;
    for (;;) {
        std::vector<std::pair<std::string, std::string>> items;
        PendingStatus status;
        bool has_status = false;
        RedisSensorMessage sensor;
        bool has_sensor = false;
        bool has_pending = false;
        {
            std::lock_guard<std::mutex> lock(publish_mutex_);
            has_pending = !pending_statuses_.empty()
                || has_pending_sensor_message_;
            if (!running_.load() && pending_statuses_.empty()
                && !has_pending_sensor_message_) break;
        }
        // Once awakened, drain queued work without another 200 ms poll per
        // item. eventfd may retain a stale count after the queue becomes empty;
        // the next poll/read consumes it harmlessly.
        if (!has_pending) {
            pollfd event{};
            event.fd = publish_event_fd_;
            event.events = POLLIN;
            int poll_result;
            do {
                poll_result = poll(&event, 1, 200);
            } while (poll_result < 0 && errno == EINTR);
            if (poll_result > 0 && (event.revents & POLLIN) != 0) {
                std::uint64_t notifications = 0;
                while (read(publish_event_fd_,
                            &notifications,
                            sizeof(notifications)) < 0
                       && errno == EINTR) {
                }
            }
        }

        // Keep queued messages intact while Redis is unavailable. During
        // shutdown, make one bounded final connection attempt and then return.
        if (!context) {
            context = Connect(config_);
            publisher_connected_.store(static_cast<bool>(context));
            if (!context) {
                if (!running_.load()) break;
                WaitForReconnectOrStop(running_, config_.reconnect_ms);
                continue;
            }
        }

        {
            std::lock_guard<std::mutex> lock(publish_mutex_);
            if (!pending_statuses_.empty()) {
                status = std::move(pending_statuses_.front());
                pending_statuses_.pop_front();
                has_status = true;
            } else if (has_pending_sensor_message_) {
                sensor = pending_sensor_message_;
                has_pending_sensor_message_ = false;
                has_sensor = true;
            }
        }

        // JSON creation is deliberately outside the producer/control thread.
        if (has_status) {
            items.emplace_back(
                config_.status_channel,
                BuildStatusJson(status.state,
                                status.status,
                                status.message,
                                status.error_code,
                                status.command_sequence,
                                status.context));
        } else if (has_sensor) {
            items.emplace_back(config_.legacy_sensor_channel,
                               BuildLegacySensorJson(sensor));
            items.emplace_back(config_.sensor_v1_channel,
                               BuildSensorV1Json(sensor));
        }

        if (items.empty()) continue;
        bool publish_failed = false;
        for (const auto& item : items) {
            ReplyPtr reply(static_cast<redisReply*>(redisCommand(
                context.get(), "PUBLISH %s %b", item.first.c_str(),
                item.second.data(), item.second.size())));
            if (!reply || reply->type != REDIS_REPLY_INTEGER) {
                publisher_connected_.store(false);
                context.reset();
                publish_failed = true;
                break;
            }
        }
        if (publish_failed) {
            {
                std::lock_guard<std::mutex> lock(publish_mutex_);
                if (has_status) {
                    pending_statuses_.push_front(std::move(status));
                } else if (has_sensor && !has_pending_sensor_message_) {
                    pending_sensor_message_ = std::move(sensor);
                    has_pending_sensor_message_ = true;
                }
            }
            // A reachable Redis may still reject every PUBLISH (for example
            // NOAUTH). Avoid a CPU spin and let Stop abandon the requeued item
            // after at most one bounded in-flight attempt.
            if (!running_.load()) break;
            WaitForReconnectOrStop(running_, config_.reconnect_ms);
        }
    }
    publisher_connected_.store(false);
}
