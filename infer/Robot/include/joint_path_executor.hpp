#pragma once
#include "realman_command.hpp"
#include "motion_ownership.hpp"
#include <optional>
#include <vector>

// Explicit coarse-positioning session. Owns the only command/feedback connection
// used by Collision in execution mode. Construction never sends motion or Stop.
class RMJointPathExecutor {
public:
    using Joints = Eigen::Matrix<double, 7, 1>;
    struct StopDiagnostics {
        std::size_t segment = 0;
        std::string phase;
        int joint = 0; // One-based joint with the largest absolute tracking error.
        double reference_deg = 0, actual_deg = 0, error_deg = 0;
        double feedback_age_ms = 0;
        std::uint64_t feedback_sequence = 0;
        bool feedback_valid = false, feedback_stale = true;
        int arm_error = 0, system_error = 0;
    };
    struct Status {
        bool active = false;
        std::size_t segment = 1;
        Joints reference = Joints::Zero();
        std::string phase = "idle", detail;
        bool stationary_confirmed = false;
        std::optional<StopDiagnostics> stop_diagnostics;
    };
    explicit RMJointPathExecutor(const RMConnectionConfig& connection, int velocity_percent = 6);
    ~RMJointPathExecutor();
    RobotStateSnapshot Feedback() const;
    Status Snapshot() const;
    // Retain original OMPL waypoints; remove only consecutive duplicate points.
    static std::vector<Joints> Prepare(const std::vector<Joints>& path);
    void Start(std::vector<Joints> path);
    void Cancel(const std::string& reason);
    void Join();
private:
    void Run();
    void StopAndConfirm();
    void Check();
    void WaitForWaypoint(const Joints& target, bool final,
                         std::chrono::steady_clock::time_point overall_deadline);
    void RecordStopDiagnostics(const RobotStateSnapshot& feedback, const Joints& reference);
    RMCommand command_;
    RMStateReader reader_;
    mutable std::mutex mutex_;
    Status status_;
    std::vector<Joints> path_;
    std::thread worker_;
    std::atomic<bool> cancel_{false};
    int velocity_percent_;
    RMMotionOwnership ownership_;
};
