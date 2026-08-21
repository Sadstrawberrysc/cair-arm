#include <realman_command.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <utility>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

// 本文件实现 RM75 的 TCP 传输层，分为五部分：
// 1) JSON 报文与单位转换；2) 同步维护命令；3) 10 ms 控制所用的异步
// 状态读取；4) 无锁 ServoJ 邮箱；5) Stop 的优先级与确认机制。
// 上层 main_rm75/rm75_control 只通过 realman_command.hpp 的接口调用，不应
// 依赖这里的 socket、报文拆包或线程同步细节。

namespace {

// -------------------------------------------------------------------------
// 模块一：协议单位、时间与 JSON 辅助函数
// -------------------------------------------------------------------------

// RM75 协议的关节角使用毫度；位置使用微米，姿态使用毫弧度。
constexpr double kMilliDegreeToRad = M_PI / 180000.0;
constexpr double kRadToMilliDegree = 180000.0 / M_PI;
constexpr std::int64_t kMinimumAsyncServoSendGapNs = 10'000'000;

std::int64_t SteadyNowNanoseconds() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void StoreSequenceMaximum(std::atomic<std::uint64_t>& destination,
                          std::uint64_t sequence) {
    std::uint64_t previous = destination.load(std::memory_order_relaxed);
    while (previous < sequence
           && !destination.compare_exchange_weak(
               previous,
               sequence,
               std::memory_order_release,
               std::memory_order_relaxed)) {}
}

int RemainingMilliseconds(std::chrono::steady_clock::time_point deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return 0;
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    return std::max(1, static_cast<int>(remaining.count()));
}

template <typename Derived>
bool AllFinite(const Eigen::MatrixBase<Derived>& values) {
    return values.array().isFinite().all();
}

template <int JointCount>
void FillJointCommand(nlohmann::json& message,
                      const Eigen::Matrix<double, JointCount, 1>& joints) {
    message["joint"] = nlohmann::json::array();
    for (int index = 0; index < joints.size(); ++index) {
        message["joint"].push_back(
            static_cast<int>(std::lround(kRadToMilliDegree * joints[index])));
    }
}

void FillPoseCommand(nlohmann::json& message,
                     const Eigen::Matrix<double, 6, 1>& pose) {
    message["pose"] = nlohmann::json::array();
    for (int index = 0; index < pose.size(); ++index) {
        const double scaled = index < 3 ? 1000000.0 * pose[index]
                                        : 1000.0 * pose[index];
        message["pose"].push_back(static_cast<int>(std::lround(scaled)));
    }
}

bool JsonInteger(const nlohmann::json& value, int& output) {
    if (!value.is_number_integer() && !value.is_number_unsigned()) return false;
    try {
        output = value.get<int>();
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseErrorValue(const nlohmann::json& value, int& output) {
    if (JsonInteger(value, output)) return true;
    if (!value.is_array()) return false;

    output = 0;
    for (const auto& entry : value) {
        int candidate = 0;
        if (!JsonInteger(entry, candidate)) return false;
        if (candidate != 0 && output == 0) output = candidate;
    }
    return true;
}

RMResult RobotRejection(const char* label, const nlohmann::json& response) {
    return RMResult::Failure(
        RMErrorCode::kRobotRejected,
        std::string(label) + " rejected by robot: " + response.dump());
}

}  // namespace

// -------------------------------------------------------------------------
// 模块二：统一结果、TCP 行分帧与机器人状态解析
// -------------------------------------------------------------------------

RMResult RMResult::Success() {
    return {};
}

RMResult RMResult::Failure(RMErrorCode code, std::string message) {
    RMResult result;
    result.code = code;
    result.message = std::move(message);
    return result;
}

RMJsonLineFramer::RMJsonLineFramer(std::size_t max_buffer_bytes)
    : max_buffer_bytes_(std::max<std::size_t>(1, max_buffer_bytes)) {}

void RMJsonLineFramer::Feed(const char* data, std::size_t size) {
    // TCP 是字节流，不保证一次 recv 对应一条 JSON；这里累积数据，直到遇到换行。
    if (data == nullptr || size == 0) return;
    buffer_.append(data, size);
    if (buffer_.size() > max_buffer_bytes_) {
        const std::size_t to_drop = buffer_.size() - max_buffer_bytes_;
        buffer_.erase(0, to_drop);
        dropped_bytes_ += to_drop;
    }
}

void RMJsonLineFramer::Feed(std::string_view data) {
    Feed(data.data(), data.size());
}

bool RMJsonLineFramer::PopLine(std::string& line) {
    const std::size_t newline = buffer_.find('\n');
    if (newline == std::string::npos) return false;

    line.assign(buffer_.data(), newline);
    buffer_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    return true;
}

void RMJsonLineFramer::Reset() {
    buffer_.clear();
}

std::size_t RMJsonLineFramer::buffered_bytes() const {
    return buffer_.size();
}

std::size_t RMJsonLineFramer::dropped_bytes() const {
    return dropped_bytes_;
}

bool RobotStateSnapshot::IsStale(
    std::chrono::milliseconds max_age,
    std::chrono::steady_clock::time_point now) const {
    if (!valid || received_at == std::chrono::steady_clock::time_point{}) return true;
    if (max_age.count() < 0) return false;
    return now - received_at > max_age;
}

RMResult ParseRobotStateMessage(const nlohmann::json& message,
                                RobotStateSnapshot& state) {
    // 只接受完整的 7 关节 + 6D 位姿状态，避免半截/错类型报文污染控制快照。
    if (!message.is_object() || !message.contains("arm_state")
        || !message["arm_state"].is_object()) {
        return RMResult::Failure(RMErrorCode::kProtocol,
                                 "response does not contain an arm_state object");
    }

    const auto& arm_state = message["arm_state"];
    if (!arm_state.contains("joint") || !arm_state["joint"].is_array()
        || arm_state["joint"].size() != 7) {
        return RMResult::Failure(RMErrorCode::kProtocol,
                                 "RM75 arm_state must contain exactly 7 joints");
    }
    if (!arm_state.contains("pose") || !arm_state["pose"].is_array()
        || arm_state["pose"].size() < 6) {
        return RMResult::Failure(RMErrorCode::kProtocol,
                                 "arm_state must contain a 6D pose");
    }

    RobotStateSnapshot parsed;
    try {
        for (int index = 0; index < parsed.joints.size(); ++index) {
            if (!arm_state["joint"][index].is_number()) {
                return RMResult::Failure(RMErrorCode::kProtocol,
                                         "arm_state joint contains a non-number");
            }
            parsed.joints[index] =
                arm_state["joint"][index].get<double>() * kMilliDegreeToRad;
        }
        for (int index = 0; index < parsed.pose.size(); ++index) {
            if (!arm_state["pose"][index].is_number()) {
                return RMResult::Failure(RMErrorCode::kProtocol,
                                         "arm_state pose contains a non-number");
            }
            const double raw = arm_state["pose"][index].get<double>();
            parsed.pose[index] = index < 3 ? raw / 1000000.0 : raw / 1000.0;
        }
    } catch (const std::exception& error) {
        return RMResult::Failure(RMErrorCode::kProtocol,
                                 std::string("invalid arm_state numeric value: ")
                                     + error.what());
    }

    if (arm_state.contains("arm_err")
        && !ParseErrorValue(arm_state["arm_err"], parsed.arm_err)) {
        return RMResult::Failure(RMErrorCode::kProtocol,
                                 "arm_state arm_err has an invalid type");
    }
    if (arm_state.contains("sys_err")) {
        if (!ParseErrorValue(arm_state["sys_err"], parsed.sys_err)) {
            return RMResult::Failure(RMErrorCode::kProtocol,
                                     "arm_state sys_err has an invalid type");
        }
    } else if (arm_state.contains("err")) {
        if (!ParseErrorValue(arm_state["err"], parsed.sys_err)) {
            return RMResult::Failure(RMErrorCode::kProtocol,
                                     "arm_state err has an invalid type");
        }
    }

    if (!AllFinite(parsed.joints) || !AllFinite(parsed.pose)) {
        return RMResult::Failure(RMErrorCode::kProtocol,
                                 "arm_state contains a non-finite value");
    }

    parsed.received_at = std::chrono::steady_clock::now();
    parsed.valid = true;
    parsed.stale = false;
    state = parsed;
    return RMResult::Success();
}

RMCommand::RMCommand()
    : rlm_port(8080),
      rlm_socket(-1),
      recv_times(0),
      last_joint_count(0),
      arm_err(0),
      sys_err(0),
      quiet(false),
      servo_result_snapshot_(
          std::make_shared<const RMResult>(RMResult::Success())),
      stop_completion_snapshot_(
          std::make_shared<const StopCompletion>()) {
    const std::string default_ip = "192.168.50.254";
    std::memset(rlm_ip, 0, sizeof(rlm_ip));
    std::memcpy(rlm_ip, default_ip.data(), default_ip.size());
    std::memset(send_msg, 0, sizeof(send_msg));
    std::memset(recv_msg, 0, sizeof(recv_msg));
    cmd_joints.setZero();
    cmd_pose.setZero();
    for (auto& slot : servo_mailbox_) {
        for (auto& joint : slot.joints) joint.store(0.0);
        slot.follow.store(false);
    }
}

RMCommand::~RMCommand() {
    CloseTCPSocket();
}

// -------------------------------------------------------------------------
// 模块三：连接生命周期与底层 JSON 收发
// -------------------------------------------------------------------------

RMResult RMCommand::TryConnectTCPSocket(int timeout_ms) {
    // 状态读取线程拥有 socket 接收权期间禁止重连，避免两个读取者消费同一响应。
    if (timeout_ms < 0) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kInvalidArgument, "connect timeout must be non-negative");
        StoreLastResult(result);
        return result;
    }
    if (async_receiver_active_.load()) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kBusy, "cannot reconnect while RMStateReader is active");
        StoreLastResult(result);
        return result;
    }

    CloseTCPSocket();

    sockaddr_in server_address{};
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(static_cast<std::uint16_t>(rlm_port));
    if (inet_pton(AF_INET, rlm_ip, &server_address.sin_addr) != 1) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kInvalidAddress,
            std::string("invalid robot IPv4 address: ") + rlm_ip);
        StoreLastResult(result);
        return result;
    }

    const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kSocketCreate,
            std::string("socket creation failed: ") + std::strerror(errno));
        StoreLastResult(result);
        return result;
    }
    if (socket_fd >= FD_SETSIZE) {
        close(socket_fd);
        const RMResult result = RMResult::Failure(
            RMErrorCode::kSocketCreate,
            "robot socket descriptor exceeds select() FD_SETSIZE");
        StoreLastResult(result);
        return result;
    }

    // 用非阻塞 connect + select 实现可控的连接超时；成功后恢复原 socket 模式。
    const int original_flags = fcntl(socket_fd, F_GETFL, 0);
    if (original_flags < 0 || fcntl(socket_fd, F_SETFL, original_flags | O_NONBLOCK) < 0) {
        const std::string error = std::strerror(errno);
        close(socket_fd);
        const RMResult result = RMResult::Failure(
            RMErrorCode::kConnect,
            std::string("failed to configure non-blocking connect: ") + error);
        StoreLastResult(result);
        return result;
    }

    int connect_result = connect(socket_fd,
                                 reinterpret_cast<sockaddr*>(&server_address),
                                 sizeof(server_address));
    if (connect_result < 0 && errno != EINPROGRESS) {
        const std::string error = std::strerror(errno);
        close(socket_fd);
        const RMResult result = RMResult::Failure(
            RMErrorCode::kConnect,
            std::string("robot connection failed: ") + error);
        StoreLastResult(result);
        return result;
    }

    if (connect_result < 0) {
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(socket_fd, &write_fds);
        timeval timeout{};
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        int ready = 0;
        do {
            ready = select(socket_fd + 1, nullptr, &write_fds, nullptr, &timeout);
        } while (ready < 0 && errno == EINTR);

        if (ready <= 0) {
            const bool timed_out = ready == 0;
            const std::string error = timed_out ? "connection timed out"
                                                : std::strerror(errno);
            close(socket_fd);
            const RMResult result = RMResult::Failure(
                timed_out ? RMErrorCode::kTimeout : RMErrorCode::kConnect,
                std::string("robot connection failed: ") + error);
            StoreLastResult(result);
            return result;
        }

        int socket_error = 0;
        socklen_t socket_error_size = sizeof(socket_error);
        if (getsockopt(socket_fd,
                       SOL_SOCKET,
                       SO_ERROR,
                       &socket_error,
                       &socket_error_size) < 0
            || socket_error != 0) {
            if (socket_error == 0) socket_error = errno;
            const std::string error = std::strerror(socket_error);
            close(socket_fd);
            const RMResult result = RMResult::Failure(
                RMErrorCode::kConnect,
                std::string("robot connection failed: ") + error);
            StoreLastResult(result);
            return result;
        }
    }

    if (fcntl(socket_fd, F_SETFL, original_flags) < 0) {
        const std::string error = std::strerror(errno);
        close(socket_fd);
        const RMResult result = RMResult::Failure(
            RMErrorCode::kConnect,
            std::string("failed to restore socket flags: ") + error);
        StoreLastResult(result);
        return result;
    }

    int enabled = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled));
    setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    timeval send_timeout{};
    send_timeout.tv_sec = 0;
    send_timeout.tv_usec = 20000;
    setsockopt(socket_fd,
               SOL_SOCKET,
               SO_SNDTIMEO,
               &send_timeout,
               sizeof(send_timeout));

    {
        std::lock_guard<std::mutex> receive_lock(receive_mutex_);
        receive_framer_.Reset();
        std::memset(recv_msg, 0, sizeof(recv_msg));
    }
    rlm_socket = socket_fd;
    connected_.store(true);

    const RMResult result = RMResult::Success();
    StoreLastResult(result);
    if (!quiet) {
        std::cout << "Robot connected to " << rlm_ip << ':' << rlm_port << std::endl;
    }
    return result;
}

RMResult RMCommand::SendJson(const nlohmann::json& request,
                             const char* label,
                             bool low_priority) {
    // ServoJ 的发送优先级最高。低优先级状态查询拿不到发送锁时会直接让步，
    // 而不是阻塞控制链路。
    std::unique_lock<std::mutex> lock(send_mutex_, std::defer_lock);
    if (low_priority) {
        if (!lock.try_lock()) {
            return RMResult::Failure(RMErrorCode::kBusy,
                                     std::string(label) + " deferred for ServoJ");
        }
    } else {
        lock.lock();
    }

    if (!connected_.load() || rlm_socket < 0) {
        return RMResult::Failure(RMErrorCode::kNotConnected,
                                 std::string(label) + ": robot is not connected");
    }

    const std::string payload = request.dump() + "\r\n";
    cmd_str = payload;
    std::memset(send_msg, 0, sizeof(send_msg));
    const std::size_t compatibility_size =
        std::min(payload.size(), sizeof(send_msg) - 1);
    std::memcpy(send_msg, payload.data(), compatibility_size);

    std::size_t sent = 0;
    while (sent < payload.size()) {
        const int flags = MSG_NOSIGNAL | (low_priority ? MSG_DONTWAIT : 0);
        const ssize_t count =
            send(rlm_socket, payload.data() + sent, payload.size() - sent, flags);
        if (count > 0) {
            sent += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && errno == EINTR) continue;

        const int send_error = count == 0 ? EPIPE : errno;
        if (send_error == EPIPE || send_error == ECONNRESET
            || send_error == ENOTCONN) {
            connected_.store(false);
        }
        const RMErrorCode code =
            (send_error == EAGAIN || send_error == EWOULDBLOCK)
                ? RMErrorCode::kTimeout
                : RMErrorCode::kSend;
        return RMResult::Failure(
            code,
            std::string(label) + " send failed: " + std::strerror(send_error));
    }
    return RMResult::Success();
}

RMResult RMCommand::ReceiveJsonLocked(nlohmann::json& response, int timeout_ms) {
    // 调用者必须持有 receive_mutex_：所有同步请求与异步接收线程共用同一 TCP 流。
    if (!connected_.load() || rlm_socket < 0) {
        return RMResult::Failure(RMErrorCode::kNotConnected,
                                 "robot is not connected");
    }
    if (timeout_ms < 0) {
        return RMResult::Failure(RMErrorCode::kInvalidArgument,
                                 "receive timeout must be non-negative");
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    bool polled_once = false;
    for (;;) {
        std::string line;
        while (receive_framer_.PopLine(line)) {
            if (line.empty()) continue;
            nlohmann::json parsed = nlohmann::json::parse(line, nullptr, false);
            if (parsed.is_discarded()) {
                if (!quiet) {
                    std::cerr << "WARNING! Ignoring invalid robot JSON frame: "
                              << line << std::endl;
                }
                continue;
            }
            response = std::move(parsed);
            return_msg = response;
            return RMResult::Success();
        }

        if (polled_once && std::chrono::steady_clock::now() >= deadline) {
            return RMResult::Failure(RMErrorCode::kTimeout,
                                     "robot receive timed out");
        }

        int remaining_ms = timeout_ms == 0 ? 0 : RemainingMilliseconds(deadline);
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(rlm_socket, &read_fds);
        timeval timeout{};
        timeout.tv_sec = remaining_ms / 1000;
        timeout.tv_usec = (remaining_ms % 1000) * 1000;

        int ready = 0;
        do {
            ready = select(rlm_socket + 1,
                           &read_fds,
                           nullptr,
                           nullptr,
                           &timeout);
        } while (ready < 0 && errno == EINTR);
        polled_once = true;

        if (ready == 0) {
            return RMResult::Failure(RMErrorCode::kTimeout,
                                     "robot receive timed out");
        }
        if (ready < 0) {
            return RMResult::Failure(
                RMErrorCode::kReceive,
                std::string("robot select failed: ") + std::strerror(errno));
        }

        char buffer[4096];
        const ssize_t count = recv(rlm_socket, buffer, sizeof(buffer), 0);
        if (count == 0) {
            connected_.store(false);
            return RMResult::Failure(RMErrorCode::kReceive,
                                     "robot closed the TCP connection");
        }
        if (count < 0) {
            if (errno == EINTR) continue;
            return RMResult::Failure(
                RMErrorCode::kReceive,
                std::string("robot recv failed: ") + std::strerror(errno));
        }

        std::memset(recv_msg, 0, sizeof(recv_msg));
        const std::size_t compatibility_size =
            std::min<std::size_t>(static_cast<std::size_t>(count),
                                  sizeof(recv_msg) - 1);
        std::memcpy(recv_msg, buffer, compatibility_size);
        receive_framer_.Feed(buffer, static_cast<std::size_t>(count));
    }
}

RMResult RMCommand::DrainInputLocked() {
    // 同步请求前丢弃遗留帧，避免把上一条命令的响应误认为当前请求的响应。
    receive_framer_.Reset();
    if (!connected_.load() || rlm_socket < 0) {
        return RMResult::Failure(RMErrorCode::kNotConnected,
                                 "robot is not connected");
    }

    // Bound stale-cache draining so a controller that continuously publishes
    // unsolicited data cannot delay a synchronous request indefinitely.
    for (int iteration = 0; iteration < 64; ++iteration) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(rlm_socket, &read_fds);
        timeval timeout{};
        const int ready =
            select(rlm_socket + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready == 0) break;
        if (ready < 0) {
            if (errno == EINTR) continue;
            return RMResult::Failure(
                RMErrorCode::kReceive,
                std::string("stale-input select failed: ") + std::strerror(errno));
        }

        char discarded[4096];
        const ssize_t count = recv(rlm_socket,
                                   discarded,
                                   sizeof(discarded),
                                   MSG_DONTWAIT);
        if (count == 0) {
            connected_.store(false);
            return RMResult::Failure(RMErrorCode::kReceive,
                                     "robot closed the TCP connection");
        }
        if (count < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            return RMResult::Failure(
                RMErrorCode::kReceive,
                std::string("stale-input recv failed: ") + std::strerror(errno));
        }
    }
    receive_framer_.Reset();
    return RMResult::Success();
}

// -------------------------------------------------------------------------
// 模块四：同步维护接口（仅供诊断、标定或非实时操作）
// -------------------------------------------------------------------------

RMResult RMCommand::TrySetHighSpeedEth(int timeout_ms) {
    if (async_receiver_active_.load()) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kBusy,
            "cannot configure Ethernet while RMStateReader is active");
        StoreLastResult(result);
        return result;
    }

    const nlohmann::json requests[] = {
        {{"command", "set_high_speed_eth"}, {"mode", 1}},
        {{"command", "set_high_ethernet"},
         {"ip", "192.168.50.254"},
         {"mask", "255.255.255.0"},
         {"gateway", "192.168.1.1"}},
        {{"command", "save_device_info_all"}},
    };

    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    for (const auto& request : requests) {
        RMResult result = DrainInputLocked();
        if (!result) {
            StoreLastResult(result);
            return result;
        }
        result = SendJson(request, "SetHighSpeedEth");
        if (!result) {
            StoreLastResult(result);
            return result;
        }
        nlohmann::json response;
        result = ReceiveJsonLocked(response, timeout_ms);
        if (!result) {
            StoreLastResult(result);
            return result;
        }
        if (!quiet) std::cout << response.dump() << std::endl;
    }

    const RMResult result = RMResult::Success();
    StoreLastResult(result);
    return result;
}

RMResult RMCommand::TryReadJ(Eigen::Matrix<double, 7, 1>& joints,
                             int timeout_ms) {
    // 异步状态读取启动后，禁止同步 ReadJ 抢占接收流。
    if (async_receiver_active_.load()) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kBusy,
            "synchronous ReadJ is disabled while RMStateReader is active");
        StoreLastResult(result);
        return result;
    }

    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    RMResult result = DrainInputLocked();
    if (!result) {
        StoreLastResult(result);
        return result;
    }
    result = SendJson({{"command", "get_joint_degree"}}, "ReadJ");
    if (!result) {
        StoreLastResult(result);
        return result;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
        nlohmann::json response;
        result = ReceiveJsonLocked(response, RemainingMilliseconds(deadline));
        if (!result) {
            StoreLastResult(result);
            return result;
        }
        if (!response.contains("joint")) {
            if (std::chrono::steady_clock::now() >= deadline) break;
            continue;
        }
        if (!response["joint"].is_array() || response["joint"].size() != 7) {
            result = RMResult::Failure(RMErrorCode::kProtocol,
                                       "ReadJ response must contain 7 joints");
            StoreLastResult(result);
            return result;
        }

        Eigen::Matrix<double, 7, 1> parsed;
        for (int index = 0; index < parsed.size(); ++index) {
            if (!response["joint"][index].is_number()) {
                result = RMResult::Failure(RMErrorCode::kProtocol,
                                           "ReadJ joint contains a non-number");
                StoreLastResult(result);
                return result;
            }
            parsed[index] =
                response["joint"][index].get<double>() * kMilliDegreeToRad;
        }
        if (!AllFinite(parsed)) {
            result = RMResult::Failure(RMErrorCode::kProtocol,
                                       "ReadJ contains a non-finite value");
            StoreLastResult(result);
            return result;
        }

        joints = parsed;
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            cmd_joints = parsed;
            last_joint_count = 7;
            hold_joints_ = parsed;
            has_hold_target_ = true;
        }
        result = RMResult::Success();
        StoreLastResult(result);
        return result;
    }

    result = RMResult::Failure(RMErrorCode::kTimeout,
                               "ReadJ timed out waiting for a joint response");
    StoreLastResult(result);
    return result;
}

RMResult RMCommand::TryReadArmState(Eigen::Matrix<double, 7, 1>& joints,
                                    Eigen::Matrix<double, 6, 1>& pose,
                                    int& arm_err_out,
                                    int& sys_err_out,
                                    int timeout_ms) {
    if (async_receiver_active_.load()) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kBusy,
            "synchronous ReadArmState is disabled while RMStateReader is active");
        StoreLastResult(result);
        return result;
    }

    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    RMResult result = DrainInputLocked();
    if (!result) {
        StoreLastResult(result);
        return result;
    }
    result = SendJson({{"command", "get_current_arm_state"}}, "ReadArmState");
    if (!result) {
        StoreLastResult(result);
        return result;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
        nlohmann::json response;
        result = ReceiveJsonLocked(response, RemainingMilliseconds(deadline));
        if (!result) {
            StoreLastResult(result);
            return result;
        }
        if (!response.contains("arm_state")) {
            if (std::chrono::steady_clock::now() >= deadline) break;
            continue;
        }

        RobotStateSnapshot parsed;
        result = ParseRobotStateMessage(response, parsed);
        if (!result) {
            StoreLastResult(result);
            return result;
        }
        parsed.sequence = state_sequence_.fetch_add(1) + 1;
        StoreRobotState(parsed);
        joints = parsed.joints;
        pose = parsed.pose;
        arm_err_out = parsed.arm_err;
        sys_err_out = parsed.sys_err;
        result = RMResult::Success();
        StoreLastResult(result);
        return result;
    }

    result = RMResult::Failure(
        RMErrorCode::kTimeout,
        "ReadArmState timed out waiting for an arm_state response");
    StoreLastResult(result);
    return result;
}

RMResult RMCommand::TryReadL(Eigen::Matrix<double, 6, 1>& pose,
                             int timeout_ms) {
    Eigen::Matrix<double, 7, 1> joints;
    int current_arm_error = 0;
    int current_system_error = 0;
    return TryReadArmState(joints,
                           pose,
                           current_arm_error,
                           current_system_error,
                           timeout_ms);
}

RMResult RMCommand::WaitTrajectoryResponseLocked(const char* label,
                                                 int timeout_ms) {
    // MoveJ/MoveL/MoveJ_P 需要等待轨迹完成确认；ServoJ 不走这个阻塞路径。
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
        nlohmann::json response;
        RMResult result =
            ReceiveJsonLocked(response, RemainingMilliseconds(deadline));
        if (!result) return result;

        if (!quiet) {
            std::cout << label << " response:\t" << response.dump() << std::endl;
        }
        if (response.contains("arm_err")) {
            int response_error = 0;
            if (!ParseErrorValue(response["arm_err"], response_error)) {
                return RMResult::Failure(RMErrorCode::kProtocol,
                                         std::string(label)
                                             + " returned invalid arm_err");
            }
            if (response_error != 0) return RobotRejection(label, response);
        }
        if (response.contains("receive_state")) {
            if (!response["receive_state"].is_boolean()) {
                return RMResult::Failure(RMErrorCode::kProtocol,
                                         std::string(label)
                                             + " returned invalid receive_state");
            }
            if (!response["receive_state"].get<bool>()) {
                return RobotRejection(label, response);
            }
        }
        if (response.contains("trajectory_state")) {
            if (!response["trajectory_state"].is_boolean()) {
                return RMResult::Failure(RMErrorCode::kProtocol,
                                         std::string(label)
                                             + " returned invalid trajectory_state");
            }
            return response["trajectory_state"].get<bool>()
                       ? RMResult::Success()
                       : RobotRejection(label, response);
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return RMResult::Failure(
                RMErrorCode::kTimeout,
                std::string(label) + " timed out before trajectory completion");
        }
    }
}

RMResult RMCommand::TryMoveJ(const Eigen::Matrix<double, 7, 1>& joints,
                             int velo,
                             int timeout_ms) {
    if (!AllFinite(joints)) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kInvalidArgument, "MoveJ joints must be finite");
        StoreLastResult(result);
        return result;
    }
    if (async_receiver_active_.load()) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kBusy,
            "blocking MoveJ is disabled while RMStateReader is active");
        StoreLastResult(result);
        return result;
    }

    nlohmann::json request;
    request["command"] = "movej";
    FillJointCommand(request, joints);
    request["v"] = velo;
    request["r"] = 0;
    if (!quiet) std::cout << request.dump() << std::endl;

    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    RMResult result = DrainInputLocked();
    if (result) result = SendJson(request, "MoveJ");
    if (result) result = WaitTrajectoryResponseLocked("MoveJ", timeout_ms);
    if (result) StoreHoldTarget(joints);
    StoreLastResult(result);
    return result;
}

RMResult RMCommand::TryMoveL(const Eigen::Matrix<double, 6, 1>& pose,
                             int velo,
                             int timeout_ms) {
    if (!AllFinite(pose)) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kInvalidArgument, "MoveL pose must be finite");
        StoreLastResult(result);
        return result;
    }
    if (async_receiver_active_.load()) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kBusy,
            "blocking MoveL is disabled while RMStateReader is active");
        StoreLastResult(result);
        return result;
    }

    nlohmann::json request;
    request["command"] = "movel";
    FillPoseCommand(request, pose);
    request["v"] = velo;
    request["r"] = 0;
    if (!quiet) std::cout << request.dump() << std::endl;

    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    RMResult result = DrainInputLocked();
    if (result) result = SendJson(request, "MoveL");
    if (result) result = WaitTrajectoryResponseLocked("MoveL", timeout_ms);
    StoreLastResult(result);
    return result;
}

RMResult RMCommand::TryMoveJP(const Eigen::Matrix<double, 6, 1>& pose,
                              int velo,
                              int timeout_ms) {
    if (!AllFinite(pose)) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kInvalidArgument, "MoveJ_P pose must be finite");
        StoreLastResult(result);
        return result;
    }
    if (async_receiver_active_.load()) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kBusy,
            "blocking MoveJ_P is disabled while RMStateReader is active");
        StoreLastResult(result);
        return result;
    }

    nlohmann::json request;
    request["command"] = "movej_p";
    FillPoseCommand(request, pose);
    request["v"] = velo;
    request["r"] = 0;
    if (!quiet) std::cout << request.dump() << std::endl;

    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    RMResult result = DrainInputLocked();
    if (result) result = SendJson(request, "MoveJ_P");
    if (result) result = WaitTrajectoryResponseLocked("MoveJ_P", timeout_ms);
    StoreLastResult(result);
    return result;
}

// -------------------------------------------------------------------------
// 模块五：ServoJ、保持与停止
// -------------------------------------------------------------------------

RMResult RMCommand::TryServoJ(const Eigen::Matrix<double, 7, 1>& joints,
                              bool follow) {
    const bool asynchronous =
        async_receiver_active_.load(std::memory_order_acquire);
    if (!AllFinite(joints)) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kInvalidArgument, "ServoJ joints must be finite");
        // Keep the asynchronous submission path free from mutex acquisition,
        // including validation failures.
        if (!asynchronous) StoreLastResult(result);
        return result;
    }
    if (servo_stop_active_.load(std::memory_order_acquire)) {
        return RMResult::Failure(
            RMErrorCode::kBusy,
            "ServoJ is latched off until StopMotion is acknowledged");
    }

    // In asynchronous mode this is the real-time submission path: fixed-size
    // atomic stores only. JSON allocation/serialization and socket I/O belong
    // exclusively to RMStateReader::ThreadMain.
    if (asynchronous) {
        if (!connected_.load(std::memory_order_acquire)) {
            return RMResult::Failure(RMErrorCode::kNotConnected,
                                     "ServoJ mailbox has no robot connection");
        }
        std::uint64_t sequence = 0;
        if (!TryPublishServoTarget(joints, follow, sequence)) {
            return RMResult::Failure(
                RMErrorCode::kBusy,
                "ServoJ mailbox has a pending target or StopMotion is active");
        }
        return RMResult::Success();
    }

    nlohmann::json request;
    request["command"] = "movej_canfd";
    FillJointCommand(request, joints);
    request["follow"] = follow;
    if (!quiet) std::cout << request.dump() << std::endl;

    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    RMResult result = DrainInputLocked();
    if (result) result = SendJson(request, "ServoJ");
    if (!result) {
        StoreLastResult(result);
        return result;
    }

    nlohmann::json response;
    const RMResult receive_result = ReceiveJsonLocked(response, 0);
    if (receive_result) {
        if (!quiet) {
            std::cout << "ServoJ response:\t" << response.dump() << std::endl;
        }
        if (response.contains("arm_err")) {
            int response_error = 0;
            if (!ParseErrorValue(response["arm_err"], response_error)) {
                result = RMResult::Failure(RMErrorCode::kProtocol,
                                           "ServoJ returned invalid arm_err");
            } else if (response_error != 0) {
                result = RobotRejection("ServoJ", response);
            }
        }
        if (result && response.contains("receive_state")) {
            if (!response["receive_state"].is_boolean()) {
                result = RMResult::Failure(RMErrorCode::kProtocol,
                                           "ServoJ returned invalid receive_state");
            } else if (!response["receive_state"].get<bool>()) {
                result = RobotRejection("ServoJ", response);
            }
        }
    } else if (receive_result.code != RMErrorCode::kTimeout) {
        result = receive_result;
    }

    if (result) StoreHoldTarget(joints);
    StoreLastResult(result);
    return result;
}

RMResult RMCommand::TryHoldMotion(
    const Eigen::Matrix<double, 7, 1>& joints,
    bool follow) {
    return TryServoJ(joints, follow);
}

RMResult RMCommand::TryHoldMotion(bool follow) {
    Eigen::Matrix<double, 7, 1> target;
    std::uint64_t sequence = 0;
    bool previous_follow = false;
    if (async_receiver_active_.load(std::memory_order_acquire)) {
        sequence = servo_published_sequence_.load(std::memory_order_acquire);
        if (sequence != 0
            && LoadServoTarget(sequence, target, previous_follow)) {
            return TryHoldMotion(target, follow);
        }
        return RMResult::Failure(
            RMErrorCode::kInvalidArgument,
            "HoldMotion has no target in the asynchronous ServoJ mailbox");
    }
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (!has_hold_target_) {
            const RMResult result = RMResult::Failure(
                RMErrorCode::kInvalidArgument,
                "HoldMotion has no validated state or command target");
            StoreLastResult(result);
            return result;
        }
        target = hold_joints_;
    }
    return TryHoldMotion(target, follow);
}

RMResult RMCommand::TryStopMotion(int timeout_ms) {
    // StopMotion 一旦开始即锁住 ServoJ。异步模式下由 I/O 线程发送 Stop 并接收
    // ACK；调用线程只等待完成快照，避免与 socket 读取竞争。
    if (timeout_ms < 0) {
        const RMResult result = RMResult::Failure(
            RMErrorCode::kInvalidArgument,
            "StopMotion timeout must be non-negative");
        StoreLastResult(result);
        return result;
    }
    if (async_receiver_active_.load(std::memory_order_acquire)) {
        if (!connected_.load(std::memory_order_acquire)) {
            return RMResult::Failure(RMErrorCode::kNotConnected,
                                     "StopMotion mailbox has no robot connection");
        }

        // Serialize Stop callers because the controller protocol has no
        // request ID with which to disambiguate concurrent acknowledgements.
        std::lock_guard<std::mutex> call_lock(stop_call_mutex_);
        servo_stop_active_.store(true, std::memory_order_release);
        const auto previous_completion = std::atomic_load_explicit(
            &stop_completion_snapshot_, std::memory_order_acquire);
        const std::uint64_t previous_completed_sequence =
            previous_completion ? previous_completion->sequence : 0;
        const std::uint64_t previous_requested_sequence =
            stop_requested_sequence_.load(std::memory_order_acquire);
        // A timed-out request remains live because a late acknowledgement is
        // still protocol-valid. Reuse that request rather than sending a
        // second indistinguishable Stop command.
        const std::uint64_t request_sequence =
            previous_requested_sequence > previous_completed_sequence
                ? previous_requested_sequence
                : stop_requested_sequence_.fetch_add(
                      1, std::memory_order_acq_rel) + 1;
        // Linearize cancellation against the I/O owner's mailbox claim. If
        // the owner already claimed the target it is in flight, not pending;
        // otherwise Stop owns and discards it without waiting on any mutex.
        DiscardPendingServoTarget();
        const auto deadline = std::chrono::steady_clock::now()
                            + std::chrono::milliseconds(timeout_ms);
        for (;;) {
            const auto completion = std::atomic_load_explicit(
                &stop_completion_snapshot_, std::memory_order_acquire);
            if (completion && completion->sequence >= request_sequence) {
                if (completion->result) {
                    servo_stop_active_.store(false, std::memory_order_release);
                }
                return completion->result;
            }
            if (!async_receiver_active_.load(std::memory_order_acquire)) {
                CompleteStopRequest(
                    request_sequence,
                    RMResult::Failure(
                        RMErrorCode::kBusy,
                        "RMStateReader stopped before StopMotion acknowledgement"));
            } else if (!connected_.load(std::memory_order_acquire)) {
                CompleteStopRequest(
                    request_sequence,
                    RMResult::Failure(
                        RMErrorCode::kNotConnected,
                        "robot disconnected before StopMotion acknowledgement"));
            } else if (std::chrono::steady_clock::now() >= deadline) {
                // Do not manufacture a completion: the command remains in
                // flight and a later call may observe its late ACK. The Servo
                // gate deliberately remains latched meanwhile.
                return RMResult::Failure(
                    RMErrorCode::kTimeout,
                    "StopMotion timed out waiting for an explicit acknowledgement");
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        }
    }

    servo_stop_active_.store(true, std::memory_order_release);
    const nlohmann::json request = {{"command", "set_arm_stop"}};
    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    RMResult result = DrainInputLocked();
    if (result) result = SendJson(request, "StopMotion");
    if (!result) {
        StoreLastResult(result);
        return result;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
        nlohmann::json response;
        result = ReceiveJsonLocked(response, RemainingMilliseconds(deadline));
        if (!result) break;
        const bool command_matched = response.contains("command")
                                  && response["command"].is_string()
                                  && response["command"].get<std::string>()
                                         == "set_arm_stop";
        const nlohmann::json* acknowledgement = nullptr;
        if (response.contains("arm_stop")) {
            acknowledgement = &response["arm_stop"];
        } else if (command_matched && response.contains("receive_state")) {
            acknowledgement = &response["receive_state"];
        }
        if (acknowledgement == nullptr) {
            if (std::chrono::steady_clock::now() >= deadline) {
                result = RMResult::Failure(
                    RMErrorCode::kTimeout,
                    "StopMotion timed out waiting for an explicit acknowledgement");
                break;
            }
            continue;
        }
        if (!acknowledgement->is_boolean()) {
            result = RMResult::Failure(RMErrorCode::kProtocol,
                                       "StopMotion returned invalid acknowledgement");
        } else if (!acknowledgement->get<bool>()) {
            result = RobotRejection("StopMotion", response);
        }
        break;
    }
    StoreLastResult(result);
    if (result) {
        servo_stop_active_.store(false, std::memory_order_release);
    }
    return result;
}

void RMCommand::CloseTCPSocket() {
    // shutdown wakes a background select/recv before the ownership locks are
    // acquired. RMStateReader may remain alive, but all of its later I/O will
    // return kNotConnected until it is stopped.
    const bool had_live_connection = connected_.load(std::memory_order_acquire)
        || rlm_socket >= 0
        || async_receiver_active_.load(std::memory_order_acquire);
    if (had_live_connection) {
        servo_stop_active_.store(true, std::memory_order_release);
        DiscardPendingServoTarget();
    }
    connected_.store(false);
    stop_in_flight_sequence_.store(0, std::memory_order_release);
    CompleteStopRequest(
        stop_requested_sequence_.load(std::memory_order_acquire),
        RMResult::Failure(RMErrorCode::kNotConnected,
                          "robot connection closed before StopMotion acknowledgement"));
    if (rlm_socket >= 0) shutdown(rlm_socket, SHUT_RDWR);
    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    std::lock_guard<std::mutex> send_lock(send_mutex_);
    if (rlm_socket >= 0) {
        close(rlm_socket);
        rlm_socket = -1;
    }
    receive_framer_.Reset();
}

bool RMCommand::IsConnected() const {
    return connected_.load() && rlm_socket >= 0;
}

RMResult RMCommand::LastResult() const {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return last_result_;
}

ServoSendSnapshot RMCommand::ServoStatus() const {
    ServoSendSnapshot snapshot;
    snapshot.submitted_sequence =
        servo_published_sequence_.load(std::memory_order_acquire);
    snapshot.consumed_sequence =
        servo_consumed_sequence_.load(std::memory_order_acquire);
    snapshot.discarded_sequence =
        servo_discarded_sequence_.load(std::memory_order_acquire);
    const std::uint64_t pending =
        servo_pending_sequence_.load(std::memory_order_acquire);
    snapshot.pending_sequence =
        pending == kServoMailboxWriting ? 0 : pending;
    snapshot.sent_sequence =
        servo_sent_sequence_.load(std::memory_order_acquire);
    const std::int64_t sent_ns =
        servo_sent_time_ns_.load(std::memory_order_relaxed);
    if (snapshot.sent_sequence != 0 && sent_ns > 0) {
        snapshot.sent_at = std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(sent_ns));
    }

    for (;;) {
        const std::uint64_t generation_before =
            servo_result_generation_.load(std::memory_order_acquire);
        if ((generation_before & 1U) != 0) continue;
        const std::uint64_t result_sequence =
            servo_result_sequence_.load(std::memory_order_relaxed);
        const auto result = std::atomic_load_explicit(
            &servo_result_snapshot_, std::memory_order_acquire);
        const std::int64_t updated_ns =
            servo_result_time_ns_.load(std::memory_order_relaxed);
        const std::uint64_t generation_after =
            servo_result_generation_.load(std::memory_order_acquire);
        if (generation_before != generation_after) continue;

        snapshot.result_sequence = result_sequence;
        snapshot.result = result ? *result : RMResult::Failure(
            RMErrorCode::kSend, "ServoJ has no send-result snapshot");
        if (updated_ns > 0) {
            snapshot.updated_at = std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(updated_ns));
        }
        return snapshot;
    }
}

RobotStateSnapshot RMCommand::CachedRobotState(
    std::chrono::milliseconds stale_after) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    RobotStateSnapshot snapshot = cached_state_;
    snapshot.stale = snapshot.IsStale(stale_after);
    return snapshot;
}

// -------------------------------------------------------------------------
// 模块六：异步 ServoJ 邮箱（控制线程 -> I/O 线程）
// -------------------------------------------------------------------------
// 控制线程在 10 ms 周期中只发布固定大小的关节目标；I/O 线程负责 JSON 序列化、
// socket 发送和结果记录。邮箱最多保留一个待发或在途目标，防止 I/O 卡顿后积压旧轨迹。

bool RMCommand::TryPublishServoTarget(
    const Eigen::Matrix<double, 7, 1>& joints,
    bool follow,
    std::uint64_t& sequence) {
    // Reserve the sole pending position before touching a slot. This CAS is
    // the backpressure boundary: a second producer returns Busy immediately
    // and does not allocate a command sequence or overwrite any target.
    if (servo_stop_active_.load(std::memory_order_acquire)) {
        sequence = 0;
        return false;
    }

    // Do not queue behind a target that the I/O owner has consumed but has
    // not yet written to the socket. This keeps the asynchronous path to one
    // pending-or-in-flight target and prevents several fixed-period steps
    // from being dispatched back-to-back after an I/O stall.
    const std::uint64_t published =
        servo_published_sequence_.load(std::memory_order_acquire);
    const std::uint64_t completed = std::max(
        servo_sent_sequence_.load(std::memory_order_acquire),
        servo_discarded_sequence_.load(std::memory_order_acquire));
    if (published > completed) {
        sequence = 0;
        return false;
    }

    std::uint64_t expected = 0;
    if (!servo_pending_sequence_.compare_exchange_strong(
            expected,
            kServoMailboxWriting,
            std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        sequence = 0;
        return false;
    }

    // Close the race in which StopMotion becomes active after the first gate
    // check but before this producer reserves the mailbox.
    if (servo_stop_active_.load(std::memory_order_acquire)) {
        expected = kServoMailboxWriting;
        servo_pending_sequence_.compare_exchange_strong(
            expected,
            0,
            std::memory_order_release,
            std::memory_order_relaxed);
        sequence = 0;
        return false;
    }

    sequence = servo_next_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    ServoMailboxSlot& slot = servo_mailbox_[sequence & 1U];
    // The acquire half prevents target stores from moving before the odd
    // marker; the release half publishes all fields before the even marker.
    // The seqlock also protects diagnostic reads of the latest accepted slot.
    slot.generation.fetch_add(1, std::memory_order_acq_rel);
    for (int index = 0; index < joints.size(); ++index) {
        slot.joints[index].store(joints[index], std::memory_order_relaxed);
    }
    slot.follow.store(follow, std::memory_order_relaxed);
    slot.sequence.store(sequence, std::memory_order_relaxed);
    slot.generation.fetch_add(1, std::memory_order_release);
    // The final CAS is the publication linearization point. StopMotion may
    // replace kServoMailboxWriting with zero; in that case this target was
    // cancelled before it became visible and must never be replayed.
    expected = kServoMailboxWriting;
    if (!servo_pending_sequence_.compare_exchange_strong(
            expected,
            sequence,
            std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        sequence = 0;
        return false;
    }
    StoreSequenceMaximum(servo_published_sequence_, sequence);
    return true;
}

bool RMCommand::LoadServoTarget(
    std::uint64_t sequence,
    Eigen::Matrix<double, 7, 1>& joints,
    bool& follow) const {
    for (int attempt = 0; attempt < 8; ++attempt) {
        if (sequence == 0 || sequence == kServoMailboxWriting) return false;
        const ServoMailboxSlot& slot = servo_mailbox_[sequence & 1U];
        const std::uint64_t generation_before =
            slot.generation.load(std::memory_order_acquire);
        if ((generation_before & 1U) != 0) continue;
        const std::uint64_t slot_sequence =
            slot.sequence.load(std::memory_order_relaxed);
        for (int index = 0; index < joints.size(); ++index) {
            joints[index] = slot.joints[index].load(std::memory_order_relaxed);
        }
        follow = slot.follow.load(std::memory_order_relaxed);
        const std::uint64_t generation_after =
            slot.generation.load(std::memory_order_acquire);
        if (slot_sequence == sequence
            && generation_before == generation_after
            && (generation_after & 1U) == 0) {
            return true;
        }
    }
    return false;
}

RMResult RMCommand::SendLatestServoTarget(
    std::uint64_t& last_dispatched_sequence) {
    const std::uint64_t sequence =
        servo_pending_sequence_.load(std::memory_order_acquire);
    if (sequence == 0 || sequence == kServoMailboxWriting) {
        return RMResult::Success();
    }
    const std::int64_t previous_send_ns =
        servo_sent_time_ns_.load(std::memory_order_acquire);
    if (previous_send_ns > 0
        && SteadyNowNanoseconds() - previous_send_ns
               < kMinimumAsyncServoSendGapNs) {
        // Leave the target pending; the I/O loop continues receiving state and
        // retries without blocking once the minimum dispatch gap has elapsed.
        return RMResult::Success();
    }

    Eigen::Matrix<double, 7, 1> joints;
    bool follow = false;
    if (!LoadServoTarget(sequence, joints, follow)) {
        return RMResult::Failure(
            RMErrorCode::kProtocol,
            "ServoJ mailbox target changed while awaiting I/O consumption");
    }

    // Claim only after copying a coherent target. StopMotion uses the same
    // CAS-to-zero operation; exactly one side can own this sequence.
    std::uint64_t expected = sequence;
    if (!servo_pending_sequence_.compare_exchange_strong(
            expected,
            0,
            std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        return RMResult::Success();
    }
    StoreSequenceMaximum(servo_consumed_sequence_, sequence);
    last_dispatched_sequence = std::max(last_dispatched_sequence, sequence);

    nlohmann::json request;
    request["command"] = "movej_canfd";
    FillJointCommand(request, joints);
    request["follow"] = follow;
    const RMResult result = SendJson(request, "ServoJ mailbox");
    StoreServoSendResult(sequence, result.ok(), result);
    if (result) {
        StoreHoldTarget(joints);
    }
    return result;
}

std::uint64_t RMCommand::DiscardPendingServoTarget() {
    std::uint64_t pending =
        servo_pending_sequence_.load(std::memory_order_acquire);
    for (;;) {
        if (pending == kServoMailboxWriting) {
            // Cancel the producer's reservation without waiting for that
            // thread to be scheduled. Its final publish CAS will then fail.
            if (servo_pending_sequence_.compare_exchange_weak(
                    pending,
                    0,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                return 0;
            }
            continue;
        }
        if (pending == 0) return 0;
        if (servo_pending_sequence_.compare_exchange_weak(
                pending,
                0,
                std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            StoreSequenceMaximum(servo_discarded_sequence_, pending);
            return pending;
        }
    }
}

void RMCommand::StoreServoSendResult(std::uint64_t sequence,
                                     bool sent,
                                     const RMResult& result) {
    const std::int64_t completed_at_ns = SteadyNowNanoseconds();
    if (sent) {
        // Publish the timestamp before the release-store of sent_sequence so
        // a snapshot that observes the new sequence also observes its time.
        servo_sent_time_ns_.store(completed_at_ns, std::memory_order_relaxed);
        StoreSequenceMaximum(servo_sent_sequence_, sequence);
    }
    servo_result_generation_.fetch_add(1, std::memory_order_acq_rel);
    std::atomic_store_explicit(
        &servo_result_snapshot_,
        std::make_shared<const RMResult>(result),
        std::memory_order_relaxed);
    servo_result_time_ns_.store(completed_at_ns, std::memory_order_relaxed);
    servo_result_sequence_.store(sequence, std::memory_order_relaxed);
    servo_result_generation_.fetch_add(1, std::memory_order_release);
}

void RMCommand::CompleteStopRequest(std::uint64_t sequence,
                                    const RMResult& result) {
    if (sequence == 0) return;
    auto current = std::atomic_load_explicit(&stop_completion_snapshot_,
                                             std::memory_order_acquire);
    for (;;) {
        if (current && sequence <= current->sequence) return;
        auto completion = std::make_shared<const StopCompletion>(
            StopCompletion{sequence, result});
        if (std::atomic_compare_exchange_weak_explicit(
                &stop_completion_snapshot_,
                &current,
                completion,
                std::memory_order_release,
                std::memory_order_acquire)) {
            return;
        }
    }
}

// -------------------------------------------------------------------------
// 模块七：异步接收权、共享快照和旧接口兼容层
// -------------------------------------------------------------------------

RMResult RMCommand::AcquireAsyncReceiver() {
    if (!IsConnected()) {
        return RMResult::Failure(RMErrorCode::kNotConnected,
                                 "cannot start state reader without a connection");
    }
    bool expected = false;
    if (!async_receiver_active_.compare_exchange_strong(expected, true)) {
        return RMResult::Failure(RMErrorCode::kBusy,
                                 "a receive owner is already active");
    }

    // Wait for any synchronous request that won the race immediately before
    // the ownership flag was set. New synchronous requests now return Busy.
    std::lock_guard<std::mutex> receive_lock(receive_mutex_);
    return RMResult::Success();
}

void RMCommand::ReleaseAsyncReceiver() {
    // A stopped I/O owner must not leave a target that a later reader instance
    // could replay on this or a reconnected socket.
    servo_stop_active_.store(true, std::memory_order_release);
    DiscardPendingServoTarget();
    async_receiver_active_.store(false);
    stop_in_flight_sequence_.store(0, std::memory_order_release);
    CompleteStopRequest(
        stop_requested_sequence_.load(std::memory_order_acquire),
        RMResult::Failure(RMErrorCode::kBusy,
                          "RMStateReader stopped before StopMotion acknowledgement"));
}

void RMCommand::StoreLastResult(const RMResult& result) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    last_result_ = result;
}

void RMCommand::StoreRobotState(const RobotStateSnapshot& state) {
    // 除缓存状态外，同时更新 Hold 的安全关节目标，供通信异常后的保持使用。
    std::lock_guard<std::mutex> lock(state_mutex_);
    cached_state_ = state;
    cmd_joints = state.joints;
    cmd_pose = state.pose;
    last_joint_count = 7;
    arm_err = state.arm_err;
    sys_err = state.sys_err;
    hold_joints_ = state.joints;
    has_hold_target_ = true;
}

void RMCommand::StoreHoldTarget(
    const Eigen::Matrix<double, 7, 1>& joints) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hold_joints_ = joints;
    has_hold_target_ = true;
}

void RMCommand::ReportLegacyFailure(const char* operation,
                                    const RMResult& result) const {
    if (!result) {
        std::cerr << "ERROR! " << operation << " failed: " << result.message
                  << std::endl;
    }
}

void RMCommand::ConnectTCPSocket() {
    // 以下无返回值接口保留给旧六轴代码；新 RM75 主线应使用 Try* 接口并检查 RMResult。
    const RMResult result = TryConnectTCPSocket();
    ReportLegacyFailure("ConnectTCPSocket", result);
}

void RMCommand::SetHighSpeedEth() {
    const RMResult result = TrySetHighSpeedEth();
    ReportLegacyFailure("SetHighSpeedEth", result);
    if (result && !quiet) {
        std::cout << "Successfully set high speed Ethernet. Change port and restart the robot."
                  << std::endl;
    }
}

void RMCommand::ReadJ(Eigen::Matrix<double, 7, 1>& joints) {
    const RMResult result = TryReadJ(joints);
    ReportLegacyFailure("ReadJ", result);
    if (!result) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        joints = cmd_joints;
    }
}

void RMCommand::ReadArmState(Eigen::Matrix<double, 7, 1>& joints,
                             Eigen::Matrix<double, 6, 1>& pose,
                             int& arm_err_out,
                             int& sys_err_out) {
    const RMResult result =
        TryReadArmState(joints, pose, arm_err_out, sys_err_out);
    ReportLegacyFailure("ReadArmState", result);
    if (!result) {
        const RobotStateSnapshot cached = CachedRobotState();
        joints = cached.joints;
        pose = cached.pose;
        arm_err_out = cached.arm_err;
        sys_err_out = cached.sys_err;
    }
}

void RMCommand::MoveJ(Eigen::Matrix<double, 7, 1>& joints, int velo) {
    const RMResult result = TryMoveJ(joints, velo);
    ReportLegacyFailure("MoveJ", result);
}

void RMCommand::MoveL(Eigen::Matrix<double, 6, 1>& pose, int velo) {
    const RMResult result = TryMoveL(pose, velo);
    ReportLegacyFailure("MoveL", result);
}

void RMCommand::MoveJP(Eigen::Matrix<double, 6, 1>& pose, int velo) {
    const RMResult result = TryMoveJP(pose, velo);
    ReportLegacyFailure("MoveJ_P", result);
}

void RMCommand::ServoJ(Eigen::Matrix<double, 7, 1>& joints, bool follow) {
    const RMResult result = TryServoJ(joints, follow);
    ReportLegacyFailure("ServoJ", result);
}

void RMCommand::ReadL(Eigen::Matrix<double, 6, 1>& pose) {
    const RMResult result = TryReadL(pose);
    ReportLegacyFailure("ReadL", result);
    if (!result) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pose = cmd_pose;
    }
}

void RMCommand::HoldMotion(bool follow) {
    const RMResult result = TryHoldMotion(follow);
    ReportLegacyFailure("HoldMotion", result);
}

void RMCommand::StopMotion() {
    const RMResult result = TryStopMotion();
    ReportLegacyFailure("StopMotion", result);
}

RMResult RequestConfirmedStop(RMCommand& command, int timeout_ms) {
    RMResult result = RMResult::Failure(
        RMErrorCode::kTimeout, "StopMotion was not attempted");
    for (int attempt = 0; attempt < 2; ++attempt) {
        result = command.TryStopMotion(timeout_ms);
        if (result) return result;
        if (result.code == RMErrorCode::kTimeout) break;
        if (attempt == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return result;
}

void BestEffortStopGuard::Arm(RMCommand& command) noexcept {
    command_ = &command;
    armed_ = true;
}

void BestEffortStopGuard::Disarm() noexcept { armed_ = false; }

BestEffortStopGuard::~BestEffortStopGuard() {
    if (!armed_ || command_ == nullptr || !command_->IsConnected()) return;
    try {
        (void)command_->TryStopMotion(500);
    } catch (...) {
        // Destructors must remain noexcept. The transport's latched Servo
        // gate still prevents a later target from overtaking this stop path.
    }
}

// -------------------------------------------------------------------------
// 模块八：RMStateReader 异步状态与 I/O 线程
// -------------------------------------------------------------------------
// 该线程是 socket 接收的唯一所有者：周期性请求 arm_state、发送最新 ServoJ、
// 优先处理 Stop ACK，并将最新机器人状态发布为线程安全快照。

RMStateReader::RMStateReader(RMCommand& command,
                             std::chrono::milliseconds poll_period,
                             std::chrono::milliseconds stale_after)
    : command_(command),
      poll_period_(std::max(std::chrono::milliseconds(10), poll_period)),
      stale_after_(std::max(std::chrono::milliseconds(1), stale_after)),
      latest_snapshot_(std::make_shared<const RobotStateSnapshot>(
          command.CachedRobotState(stale_after_))),
      last_result_snapshot_(
          std::make_shared<const RMResult>(RMResult::Success())) {}

RMStateReader::~RMStateReader() {
    Stop();
}

RMResult RMStateReader::Start() {
    if (running_.load()) return RMResult::Success();

    RMResult result = command_.AcquireAsyncReceiver();
    if (!result) {
        StoreReaderResult(result);
        return result;
    }

    std::atomic_store_explicit(
        &latest_snapshot_,
        std::make_shared<const RobotStateSnapshot>(
            command_.CachedRobotState(stale_after_)),
        std::memory_order_release);
    std::atomic_store_explicit(
        &last_result_snapshot_,
        std::make_shared<const RMResult>(RMResult::Success()),
        std::memory_order_release);
    stop_requested_.store(false);
    running_.store(true);
    try {
        thread_ = std::thread(&RMStateReader::ThreadMain, this);
    } catch (const std::exception& error) {
        running_.store(false);
        command_.ReleaseAsyncReceiver();
        result = RMResult::Failure(
            RMErrorCode::kReceive,
            std::string("failed to start RMStateReader thread: ") + error.what());
        StoreReaderResult(result);
        return result;
    }
    return RMResult::Success();
}

void RMStateReader::Stop() {
    if (!running_.load() && !thread_.joinable()) return;
    stop_requested_.store(true);
    if (thread_.joinable()) thread_.join();
    running_.store(false);
    command_.ReleaseAsyncReceiver();
}

bool RMStateReader::running() const {
    return running_.load();
}

RobotStateSnapshot RMStateReader::Latest() const {
    const auto latest = std::atomic_load_explicit(&latest_snapshot_,
                                                  std::memory_order_acquire);
    RobotStateSnapshot snapshot = latest ? *latest : RobotStateSnapshot{};
    snapshot.stale = snapshot.IsStale(stale_after_);
    return snapshot;
}

bool RMStateReader::WaitForUpdate(std::uint64_t after_sequence,
                                  std::chrono::milliseconds timeout,
                                  RobotStateSnapshot& state) const {
    const auto deadline = std::chrono::steady_clock::now()
                        + std::max(std::chrono::milliseconds(0), timeout);
    for (;;) {
        state = Latest();
        if (state.sequence > after_sequence) return true;
        if (!running_.load() || std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

RMResult RMStateReader::LastResult() const {
    const auto result = std::atomic_load_explicit(&last_result_snapshot_,
                                                  std::memory_order_acquire);
    return result ? *result : RMResult::Failure(
        RMErrorCode::kReceive, "RMStateReader has no result snapshot");
}

void RMStateReader::StoreReaderResult(const RMResult& result) {
    std::atomic_store_explicit(
        &last_result_snapshot_,
        std::make_shared<const RMResult>(result),
        std::memory_order_release);
}

void RMStateReader::ThreadMain() {
    auto next_request = std::chrono::steady_clock::now();
    std::uint64_t last_servo_dispatched =
        std::max(command_.servo_sent_sequence_.load(std::memory_order_acquire),
                 std::max(
                     command_.servo_consumed_sequence_.load(
                         std::memory_order_acquire),
                     command_.servo_discarded_sequence_.load(
                         std::memory_order_acquire)));
    const auto completed_stop = std::atomic_load_explicit(
        &command_.stop_completion_snapshot_, std::memory_order_acquire);
    std::uint64_t last_stop_dispatched =
        completed_stop ? completed_stop->sequence : 0;
    while (!stop_requested_.load()) {
        const auto now = std::chrono::steady_clock::now();

        // Stop has strict priority and suppresses targets that were published
        // before it was dispatched. A target published afterwards is treated
        // as an explicit request to resume only after the stop is acknowledged.
        const std::uint64_t requested_stop =
            command_.stop_requested_sequence_.load(std::memory_order_acquire);
        if (requested_stop > last_stop_dispatched) {
            last_stop_dispatched = requested_stop;
            const std::uint64_t discarded =
                command_.DiscardPendingServoTarget();
            last_servo_dispatched =
                std::max(last_servo_dispatched, discarded);
            const RMResult stop_send_result = command_.SendJson(
                {{"command", "set_arm_stop"}}, "StopMotion mailbox");
            if (stop_send_result) {
                command_.stop_in_flight_sequence_.store(
                    requested_stop, std::memory_order_release);
            } else {
                command_.CompleteStopRequest(requested_stop, stop_send_result);
                StoreReaderResult(stop_send_result);
            }
        }

        if (command_.stop_in_flight_sequence_.load(std::memory_order_acquire) == 0) {
            const RMResult servo_send_result =
                command_.SendLatestServoTarget(last_servo_dispatched);
            if (!servo_send_result) StoreReaderResult(servo_send_result);
        }

        if (now >= next_request) {
            // 状态查询是低优先级发送：ServoJ 正在占用发送锁时，本周期直接跳过，
            // 下一个 poll 周期再请求，不能阻塞 ServoJ。
            const RMResult send_result = command_.SendJson(
                {{"command", "get_current_arm_state"}},
                "RMStateReader",
                true);
            if (!send_result && send_result.code != RMErrorCode::kBusy) {
                StoreReaderResult(send_result);
            }
            next_request = now + poll_period_;
        }

        nlohmann::json response;
        RMResult receive_result;
        {
            std::lock_guard<std::mutex> receive_lock(command_.receive_mutex_);
            receive_result = command_.ReceiveJsonLocked(response, 5);
        }
        if (!receive_result) {
            if (receive_result.code != RMErrorCode::kTimeout) {
                StoreReaderResult(receive_result);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            continue;
        }

        std::uint64_t active_stop =
            command_.stop_in_flight_sequence_.load(std::memory_order_acquire);
        bool stop_response = response.contains("arm_stop");
        if (!stop_response && response.contains("command")
            && response["command"].is_string()
            && response["command"].get<std::string>() == "set_arm_stop"
            && response.contains("receive_state")) {
            stop_response = true;
        }
        if (active_stop != 0 && stop_response) {
            const std::uint64_t stop_sequence = active_stop;
            RMResult stop_result;
            const nlohmann::json* acknowledgement = nullptr;
            if (response.contains("arm_stop")) {
                acknowledgement = &response["arm_stop"];
            } else {
                acknowledgement = &response["receive_state"];
            }
            if (!acknowledgement->is_boolean()) {
                stop_result = RMResult::Failure(
                    RMErrorCode::kProtocol,
                    "StopMotion returned a non-boolean acknowledgement");
            } else if (!acknowledgement->get<bool>()) {
                stop_result = RobotRejection("StopMotion", response);
            } else {
                stop_result = RMResult::Success();
            }
            command_.stop_in_flight_sequence_.compare_exchange_strong(
                active_stop,
                0,
                std::memory_order_acq_rel,
                std::memory_order_acquire);
            command_.CompleteStopRequest(stop_sequence, stop_result);
            if (!stop_result) StoreReaderResult(stop_result);
            continue;
        }

        if (response.contains("arm_state")) {
            // 正常状态帧：解析、分配递增序号并原子发布，供 10 ms 控制循环读取。
            RobotStateSnapshot parsed;
            const RMResult parse_result = ParseRobotStateMessage(response, parsed);
            if (!parse_result) {
                StoreReaderResult(parse_result);
                continue;
            }
            parsed.sequence = command_.state_sequence_.fetch_add(1) + 1;
            command_.StoreRobotState(parsed);
            std::atomic_store_explicit(
                &latest_snapshot_,
                std::make_shared<const RobotStateSnapshot>(parsed),
                std::memory_order_release);
            // A negative command acknowledgement is a safety-relevant fault
            // and remains latched for the controller to observe.
            const auto previous_result = std::atomic_load_explicit(
                &last_result_snapshot_, std::memory_order_acquire);
            if (!previous_result
                || previous_result->code != RMErrorCode::kRobotRejected) {
                StoreReaderResult(RMResult::Success());
            }
            continue;
        }

        if (response.contains("arm_err")) {
            int response_error = 0;
            if (!ParseErrorValue(response["arm_err"], response_error)) {
                const RMResult protocol_result = RMResult::Failure(
                    RMErrorCode::kProtocol,
                    "asynchronous response contains invalid arm_err");
                command_.StoreServoSendResult(
                    command_.servo_sent_sequence_.load(std::memory_order_acquire),
                    false,
                    protocol_result);
                StoreReaderResult(protocol_result);
            } else if (response_error != 0) {
                RobotStateSnapshot error_state = Latest();
                error_state.arm_err = response_error;
                std::atomic_store_explicit(
                    &latest_snapshot_,
                    std::make_shared<const RobotStateSnapshot>(error_state),
                    std::memory_order_release);
                const RMResult rejection =
                    RobotRejection("asynchronous command", response);
                command_.StoreServoSendResult(
                    command_.servo_sent_sequence_.load(std::memory_order_acquire),
                    false,
                    rejection);
                StoreReaderResult(rejection);
            }
        }
        if (response.contains("receive_state")) {
            RMResult acknowledgement_result;
            if (!response["receive_state"].is_boolean()) {
                acknowledgement_result = RMResult::Failure(
                    RMErrorCode::kProtocol,
                    "asynchronous response contains invalid receive_state");
            } else if (!response["receive_state"].get<bool>()) {
                acknowledgement_result =
                    RobotRejection("asynchronous command", response);
            } else {
                acknowledgement_result = RMResult::Success();
            }
            command_.StoreServoSendResult(
                command_.servo_sent_sequence_.load(std::memory_order_acquire),
                false,
                acknowledgement_result);
            if (!acknowledgement_result) {
                StoreReaderResult(acknowledgement_result);
            }
        }
        if (response.contains("arm_stop")) {
            if (!response["arm_stop"].is_boolean()) {
                StoreReaderResult(RMResult::Failure(
                    RMErrorCode::kProtocol,
                    "asynchronous response contains invalid arm_stop"));
            } else if (!response["arm_stop"].get<bool>()) {
                StoreReaderResult(RobotRejection("StopMotion", response));
            }
        }
    }
    running_.store(false);
}
