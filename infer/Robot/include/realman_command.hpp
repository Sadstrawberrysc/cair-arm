#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <json.hpp>

enum class RMErrorCode {
    kNone = 0,
    kNotConnected,
    kSocketCreate,
    kInvalidAddress,
    kConnect,
    kSend,
    kReceive,
    kTimeout,
    kProtocol,
    kRobotRejected,
    kBusy,
    kInvalidArgument,
};

struct RMResult {
    RMErrorCode code = RMErrorCode::kNone;
    std::string message;

    bool ok() const { return code == RMErrorCode::kNone; }
    explicit operator bool() const { return ok(); }

    static RMResult Success();
    static RMResult Failure(RMErrorCode code, std::string message);
};

// Incremental line framer for the Realman JSON-over-TCP protocol. The
// controller terminates JSON objects with LF or CRLF. TCP packet boundaries
// are deliberately ignored, so split and coalesced frames are both handled.
class RMJsonLineFramer {
public:
    explicit RMJsonLineFramer(std::size_t max_buffer_bytes = 64 * 1024);

    void Feed(const char* data, std::size_t size);
    bool PopLine(std::string& line);
    void Reset();

private:
    std::size_t max_buffer_bytes_;
    std::string buffer_;
};

struct RobotStateSnapshot {
    Eigen::Matrix<double, 7, 1> joints = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 1> pose = Eigen::Matrix<double, 6, 1>::Zero();
    int arm_err = 0;
    int sys_err = 0;
    std::uint64_t sequence = 0;
    std::chrono::steady_clock::time_point received_at{};
    bool valid = false;
    bool stale = true;

    bool IsStale(std::chrono::milliseconds max_age,
                 std::chrono::steady_clock::time_point now =
                     std::chrono::steady_clock::now()) const;
};

struct ServoSendSnapshot {
    std::uint64_t submitted_sequence = 0;
    // Sequence removed from the mailbox by the I/O owner for processing.
    // Consumption precedes serialization/send, so sent_sequence may lag it.
    std::uint64_t consumed_sequence = 0;
    // Sequence removed without sending because StopMotion took priority.
    std::uint64_t discarded_sequence = 0;
    // Non-zero only while a submitted target still awaits I/O ownership.
    std::uint64_t pending_sequence = 0;
    std::uint64_t sent_sequence = 0;
    std::uint64_t result_sequence = 0;
    RMResult result;
    // Timestamp of the latest successful socket write. This is kept
    // separate from updated_at because a later robot acknowledgement may
    // refresh the result without representing a new command dispatch.
    std::chrono::steady_clock::time_point sent_at{};
    std::chrono::steady_clock::time_point updated_at{};

    bool pending() const { return pending_sequence != 0; }
};

// Parses and validates a get_current_arm_state response and converts all
// values to SI units (joint radians, position metres, orientation radians).
RMResult ParseRobotStateMessage(const nlohmann::json& message,
                                RobotStateSnapshot& state);

struct RMConnectionConfig {
    static constexpr std::size_t kMaximumIpv4TextLength = 15;

    std::string ip = "192.168.50.254";
    int port = 8080;
};

class RMStateReader;

class RMCommand {
public:
    explicit RMCommand(RMConnectionConfig connection = {});
    ~RMCommand();
    RMCommand(const RMCommand&) = delete;
    RMCommand& operator=(const RMCommand&) = delete;

    RMResult TryConnectTCPSocket(int timeout_ms = 3000);
    RMResult TrySetHighSpeedEth(int timeout_ms = 3000);
    RMResult TryReadJ(Eigen::Matrix<double, 7, 1>& joints,
                      int timeout_ms = 1000);
    RMResult TryReadArmState(Eigen::Matrix<double, 7, 1>& joints,
                             Eigen::Matrix<double, 6, 1>& pose,
                             int& arm_err_out,
                             int& sys_err_out,
                             int timeout_ms = 1000);
    RMResult TryReadL(Eigen::Matrix<double, 6, 1>& pose,
                      int timeout_ms = 1000);
    RMResult TryMoveJ(const Eigen::Matrix<double, 7, 1>& joints,
                      int velo,
                      int timeout_ms = 30000);
    RMResult TryMoveL(const Eigen::Matrix<double, 6, 1>& pose,
                      int velo,
                      int timeout_ms = 30000);
    RMResult TryMoveJP(const Eigen::Matrix<double, 6, 1>& pose,
                       int velo,
                       int timeout_ms = 30000);
    RMResult TryServoJ(const Eigen::Matrix<double, 7, 1>& joints, bool follow);

    // Hold resends a validated joint target through ServoJ. The no-target
    // overload uses the latest successfully read or commanded target.
    RMResult TryHoldMotion(const Eigen::Matrix<double, 7, 1>& joints,
                           bool follow = false);
    RMResult TryHoldMotion(bool follow = false);

    // Controlled trajectory stop. This is the Realman set_arm_stop command,
    // not a power-off or emergency-stop reset.
    RMResult TryStopMotion(int timeout_ms = 1000);

    void CloseTCPSocket();
    bool IsConnected() const;
    void SetQuiet(bool quiet);
    const RMConnectionConfig& ConnectionConfig() const;
    RMResult LastResult() const;
    ServoSendSnapshot ServoStatus() const;
    RobotStateSnapshot CachedRobotState(
        std::chrono::milliseconds stale_after = std::chrono::milliseconds(250)) const;

private:
    friend class RMStateReader;

    const RMConnectionConfig connection_;
    int rlm_socket;
    bool quiet_;

    RMResult SendJson(const nlohmann::json& request,
                      const char* label,
                      bool low_priority = false);
    RMResult ReceiveJsonLocked(nlohmann::json& response, int timeout_ms);
    RMResult DrainInputLocked();
    RMResult WaitTrajectoryResponseLocked(const char* label, int timeout_ms);
    RMResult AcquireAsyncReceiver();
    void ReleaseAsyncReceiver();
    void StoreLastResult(const RMResult& result);
    void StoreRobotState(const RobotStateSnapshot& state);
    void StoreHoldTarget(const Eigen::Matrix<double, 7, 1>& joints);
    bool TryPublishServoTarget(
        const Eigen::Matrix<double, 7, 1>& joints,
        bool follow,
        std::uint64_t& sequence);
    bool LoadServoTarget(std::uint64_t sequence,
                         Eigen::Matrix<double, 7, 1>& joints,
                         bool& follow) const;
    std::uint64_t DiscardPendingServoTarget();
    RMResult SendLatestServoTarget(std::uint64_t& last_dispatched_sequence);
    void StoreServoSendResult(std::uint64_t sequence,
                              bool sent,
                              const RMResult& result);
    void CompleteStopRequest(std::uint64_t sequence, const RMResult& result);

    struct ServoMailboxSlot {
        // Single-producer seqlock. Odd means that the producer is rewriting
        // this slot; an unchanged even value brackets one coherent target.
        std::atomic<std::uint64_t> generation{0};
        std::atomic<std::uint64_t> sequence{0};
        std::atomic<double> joints[7];
        std::atomic<bool> follow{false};
    };

    struct StopCompletion {
        std::uint64_t sequence = 0;
        RMResult result;
    };

    mutable std::mutex send_mutex_;
    mutable std::mutex receive_mutex_;
    mutable std::mutex state_mutex_;
    mutable std::mutex result_mutex_;
    RMJsonLineFramer receive_framer_;
    RobotStateSnapshot cached_state_;
    Eigen::Matrix<double, 7, 1> hold_joints_ =
        Eigen::Matrix<double, 7, 1>::Zero();
    bool has_hold_target_ = false;
    RMResult last_result_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> async_receiver_active_{false};
    std::atomic<std::uint64_t> state_sequence_{0};
    ServoMailboxSlot servo_mailbox_[2];
    std::atomic<std::uint64_t> servo_next_sequence_{0};
    std::atomic<std::uint64_t> servo_published_sequence_{0};
    // Zero means free; kServoMailboxWriting reserves the producer slot; any
    // other value is the one target awaiting I/O consumption.
    static constexpr std::uint64_t kServoMailboxWriting = ~std::uint64_t{0};
    std::atomic<std::uint64_t> servo_pending_sequence_{0};
    std::atomic<std::uint64_t> servo_sent_sequence_{0};
    std::atomic<std::uint64_t> servo_consumed_sequence_{0};
    std::atomic<std::uint64_t> servo_discarded_sequence_{0};
    std::atomic<std::uint64_t> servo_result_sequence_{0};
    std::atomic<std::uint64_t> servo_result_generation_{0};
    std::atomic<std::int64_t> servo_sent_time_ns_{0};
    std::atomic<std::int64_t> servo_result_time_ns_{0};
    std::shared_ptr<const RMResult> servo_result_snapshot_;
    // Prevents a producer from opening a new pending slot while StopMotion is
    // cancelling the current one and awaiting its bounded acknowledgement.
    std::atomic<bool> servo_stop_active_{false};
    std::atomic<std::uint64_t> stop_requested_sequence_{0};
    std::atomic<std::uint64_t> stop_in_flight_sequence_{0};
    mutable std::mutex stop_call_mutex_;
    std::shared_ptr<const StopCompletion> stop_completion_snapshot_;
};

// Requests a bounded, explicitly acknowledged StopMotion. A non-timeout
// transport failure receives one retry; timeout remains fail-closed.
RMResult RequestConfirmedStop(RMCommand& command, int timeout_ms = 1000);

// Arms only after execute mode owns a live connection. Early-return paths
// receive one best-effort StopMotion unless a confirmed stationary stop has
// explicitly disarmed the guard.
class BestEffortStopGuard {
public:
    void Arm(RMCommand& command) noexcept;
    void Disarm() noexcept;
    ~BestEffortStopGuard();

private:
    RMCommand* command_ = nullptr;
    bool armed_ = false;
};

// A single background I/O owner for one RMCommand connection. It serializes
// and sends each accepted ServoJ mailbox target, prioritizes StopMotion,
// issues state queries, and consumes all incoming JSON. The real-time thread
// only publishes a fixed-size target and atomically loads immutable snapshots.
class RMStateReader {
public:
    RMStateReader(RMCommand& command,
                  std::chrono::milliseconds poll_period =
                      std::chrono::milliseconds(40),
                  std::chrono::milliseconds stale_after =
                      std::chrono::milliseconds(250));
    ~RMStateReader();
    RMStateReader(const RMStateReader&) = delete;
    RMStateReader& operator=(const RMStateReader&) = delete;

    RMResult Start();
    void Stop();
    bool running() const;
    RobotStateSnapshot Latest() const;
    bool WaitForUpdate(std::uint64_t after_sequence,
                       std::chrono::milliseconds timeout,
                       RobotStateSnapshot& state) const;
    RMResult LastResult() const;

private:
    void ThreadMain();
    void StoreReaderResult(const RMResult& result);

    RMCommand& command_;
    std::chrono::milliseconds poll_period_;
    std::chrono::milliseconds stale_after_;
    std::shared_ptr<const RobotStateSnapshot> latest_snapshot_;
    std::shared_ptr<const RMResult> last_result_snapshot_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
};
