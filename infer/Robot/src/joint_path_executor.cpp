#include "joint_path_executor.hpp"
#include "realman_kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
using Clock = std::chrono::steady_clock;
constexpr double kRad = 3.14159265358979323846 / 180.;
void Require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
void Limits(const RMJointPathExecutor::Joints& q) {
    static const RMKinematics model;
    Require(q.allFinite(), "non_finite_joint_target");
    for (int i = 0; i < 7; ++i)
        Require(q[i] >= model.JointMinimums()[i] + 3*kRad &&
                q[i] <= model.JointMaximums()[i] - 3*kRad,
                "joint_limit_margin_J" + std::to_string(i+1));
}
bool SameMoveJTarget(const RMJointPathExecutor::Joints& a, const RMJointPathExecutor::Joints& b) {
    for (int i=0; i<7; ++i)
        if (std::lround(a[i]/kRad*1000)!=std::lround(b[i]/kRad*1000)) return false;
    return true;
}
RMJointPathExecutor::StopDiagnostics Diagnostics(
    const RobotStateSnapshot& feedback, const RMJointPathExecutor::Status& status,
    const RMJointPathExecutor::Joints& reference) {
    RMJointPathExecutor::StopDiagnostics out;
    out.segment=status.segment; out.phase=status.phase;
    Eigen::Index joint=0;
    (feedback.joints-reference).cwiseAbs().maxCoeff(&joint);
    out.joint=int(joint)+1;
    out.reference_deg=reference[joint]/kRad; out.actual_deg=feedback.joints[joint]/kRad;
    out.error_deg=out.actual_deg-out.reference_deg;
    out.feedback_age_ms=std::chrono::duration<double,std::milli>(Clock::now()-feedback.received_at).count();
    out.feedback_sequence=feedback.sequence;
    out.feedback_valid=feedback.valid; out.feedback_stale=feedback.stale;
    out.arm_error=feedback.arm_err; out.system_error=feedback.sys_err;
    return out;
}
}

RMJointPathExecutor::RMJointPathExecutor(const RMConnectionConfig& connection, int velocity_percent)
    : command_(connection), reader_(command_, std::chrono::milliseconds(20), std::chrono::milliseconds(200)),
      velocity_percent_(velocity_percent) {
    Require(velocity_percent>=1 && velocity_percent<=100,"invalid_movej_velocity_percent");
    Require(ownership_.Acquire(connection),"robot_motion_already_owned");
    command_.SetQuiet(true);
    const auto connected = command_.TryConnectTCPSocket(1000);
    Require(bool(connected), "robot_connect: " + connected.message);
    const auto started = reader_.Start();
    Require(bool(started), "robot_reader: " + started.message);
}
RMJointPathExecutor::~RMJointPathExecutor() {
    Cancel("session_closed");
    Join();
    reader_.Stop();
    command_.CloseTCPSocket();
}
RobotStateSnapshot RMJointPathExecutor::Feedback() const { return reader_.Latest(); }
RMJointPathExecutor::Status RMJointPathExecutor::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}
std::vector<RMJointPathExecutor::Joints> RMJointPathExecutor::Prepare(const std::vector<Joints>& path) {
    Require(!path.empty() && path.size()<=10000,"invalid_joint_path_size");
    std::vector<Joints> result;
    for (const auto& q : path) {
        Limits(q);
        if (result.empty() || (q-result.back()).cwiseAbs().maxCoeff()>1e-12) result.push_back(q);
    }
    if (result.size()==1) result.push_back(result.front());
    return result;
}
void RMJointPathExecutor::Start(std::vector<Joints> path) {
    Require(!Snapshot().active, "execution_already_active");
    Require(Snapshot().phase!="stop_unconfirmed", "previous_stop_unconfirmed_restart_required");
    Join();
    Require(!command_.MoveJStatus().restart_required,"interrupted_movej_restart_required");
    Require(path.size()>=2 && path.size()<=20000, "invalid_execution_path");
    for (const auto& q : path) Limits(q);
    const auto state = Feedback();
    Require(state.valid && !state.stale && !state.arm_err && !state.sys_err, "feedback_not_ready");
    Require((state.joints-path.front()).cwiseAbs().maxCoeff() <= .5*kRad, "start_changed_replan_required");
    path_=std::move(path);
    cancel_.store(false);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_={true, 1, path_.front(), "preparing", "", false, std::nullopt};
    }
    try { worker_=std::thread([this] { Run(); }); }
    catch (...) { std::lock_guard<std::mutex> lock(mutex_); status_.active=false; throw; }
}
void RMJointPathExecutor::Cancel(const std::string& reason) {
    const auto feedback=Feedback();
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_.active && !cancel_.load()) {
        status_.detail=reason;
        if (!status_.stop_diagnostics)
            status_.stop_diagnostics=Diagnostics(feedback,status_,status_.reference);
        cancel_.store(true);
    }
}
void RMJointPathExecutor::Join() { if (worker_.joinable()) worker_.join(); }
void RMJointPathExecutor::RecordStopDiagnostics(const RobotStateSnapshot& feedback, const Joints& reference) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.stop_diagnostics)
        status_.stop_diagnostics=Diagnostics(feedback,status_,reference);
}
void RMJointPathExecutor::Check() {
    Require(!cancel_.load(),"execution_cancelled");
    const auto state=Feedback();
    if (!state.valid || state.stale || state.arm_err || state.sys_err) {
        RecordStopDiagnostics(state,Snapshot().reference);
        throw std::runtime_error("feedback_stale_or_fault");
    }
    Limits(state.joints);
    const auto segment=Snapshot().segment;
    const auto delta=(path_.at(segment)-path_.at(segment-1)).eval();
    const double u=delta.squaredNorm()>0 ? std::clamp(
        (state.joints-path_[segment-1]).dot(delta)/delta.squaredNorm(),0.,1.) : 1.;
    const auto reference=(path_[segment-1]+u*delta).eval();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.reference=reference;
    }
    // MoveJ's destination is not a streaming reference. Monitor deviation
    // from the checked joint-space edge, not distance to a far-away goal.
    if ((state.joints-reference).cwiseAbs().maxCoeff()>5*kRad) {
        RecordStopDiagnostics(state,reference);
        throw std::runtime_error("joint_path_deviation");
    }
}
void RMJointPathExecutor::WaitForWaypoint(const Joints& target, bool final, Clock::time_point overall_deadline) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.phase=final ? "settling_goal" : "settling_waypoint";
    }
    const auto deadline=Clock::now()+std::chrono::seconds(final ? 3 : 2);
    auto sequence=Feedback().sequence;
    std::optional<Clock::time_point> stable_since;
    int fresh_in_tolerance=0;
    while (true) {
        const auto cycle=Clock::now();
        Require(cycle<overall_deadline,"execution_total_timeout");
        Check(); // No ServoJ hold: the unblended MoveJ controller owns its endpoint.
        const auto feedback=Feedback();
        if (feedback.sequence!=sequence) {
            sequence=feedback.sequence;
            if (feedback.valid && !feedback.stale && !feedback.arm_err && !feedback.sys_err &&
                (feedback.joints-target).cwiseAbs().maxCoeff()<=(final ? .5 : .3)*kRad) {
                if (!stable_since) stable_since=Clock::now();
                ++fresh_in_tolerance;
                if (fresh_in_tolerance>=2 && (!final || Clock::now()-*stable_since>=std::chrono::milliseconds(300))) return;
            } else { stable_since.reset(); fresh_in_tolerance=0; }
        }
        Require(Clock::now()<deadline,final ? "goal_not_reached" : "segment_not_reached");
        std::this_thread::sleep_until(cycle+std::chrono::milliseconds(10));
    }
}
void RMJointPathExecutor::StopAndConfirm() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.stationary_confirmed=false;
    }
    const auto stop=RequestConfirmedStop(command_,500);
    Require(bool(stop), "stop_not_acknowledged: "+stop.message);
    auto anchor=Feedback(), previous=anchor;
    Require(anchor.valid && !anchor.stale, "stop_feedback_unavailable");
    const auto deadline=Clock::now()+std::chrono::seconds(4);
    int stable=0;
    while (Clock::now()<deadline) {
        RobotStateSnapshot current;
        if (!reader_.WaitForUpdate(previous.sequence,std::chrono::milliseconds(100),current)) continue;
        Require(current.valid && !current.stale && !current.arm_err && !current.sys_err,"stop_feedback_fault");
        if ((current.joints-previous.joints).cwiseAbs().maxCoeff()<=.01*kRad &&
            (current.joints-anchor.joints).cwiseAbs().maxCoeff()<=.02*kRad &&
            (current.pose.head<3>()-anchor.pose.head<3>()).norm()<=.0001 &&
            (current.pose.tail<3>()-anchor.pose.tail<3>()).cwiseAbs().maxCoeff()<=.02*kRad) {
            if (++stable>=5) {
                std::lock_guard<std::mutex> lock(mutex_);
                status_.stationary_confirmed=true;
                return;
            }
        } else { stable=0; anchor=current; }
        previous=current;
    }
    throw std::runtime_error("stop_stationary_unconfirmed");
}
void RMJointPathExecutor::Run() {
    BestEffortStopGuard guard;
    guard.Arm(command_);
    bool success=false;
    std::string error;
    try {
        StopAndConfirm();
        Require((Feedback().joints-path_.front()).cwiseAbs().maxCoeff()<=.5*kRad,"start_changed_after_stop");
        const auto overall_deadline=Clock::now()+std::chrono::minutes(10);
        for (std::size_t i=1; i<path_.size(); ++i) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                status_.segment=i; status_.reference=path_[i-1]; status_.phase="preparing_movej";
            }
            Check();
            if (!SameMoveJTarget(path_[i],Feedback().joints)) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    status_.phase="executing_movej"; status_.stationary_confirmed=false;
                }
                const auto submitted=command_.TryStartMoveJ(path_[i],velocity_percent_);
                Require(bool(submitted),"movej_submit: "+submitted.message);
                const auto submitted_at=Clock::now();
                while (true) {
                    Check();
                    const auto move=command_.MoveJStatus();
                    Require(bool(move.result),"movej_failed: "+move.result.message);
                    if (move.completed) break;
                    const auto now=Clock::now();
                    Require(move.sent || now-submitted_at<std::chrono::milliseconds(500),"movej_dispatch_timeout");
                    Require(move.accepted || now-submitted_at<std::chrono::seconds(2),"movej_acceptance_timeout");
                    Require(now-submitted_at<std::chrono::seconds(120),"movej_completion_timeout");
                    Require(now<overall_deadline,"execution_total_timeout");
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            WaitForWaypoint(path_[i],i+1==path_.size(),overall_deadline);
        }
        success=true;
    } catch (const std::exception& e) {
        error=e.what();
        RecordStopDiagnostics(Feedback(),Snapshot().reference);
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.phase="stopping";
    }
    try {
        StopAndConfirm();
        guard.Disarm();
        if (success && (Feedback().joints-path_.back()).cwiseAbs().maxCoeff()>.5*kRad) {
            success=false; error="final_joint_error_after_stop";
            RecordStopDiagnostics(Feedback(),Snapshot().reference);
        }
    } catch (const std::exception& e) {
        success=false; error+="; "+std::string(e.what());
        RecordStopDiagnostics(Feedback(),Snapshot().reference);
    }
    if (cancel_.load()) success=false;
    std::lock_guard<std::mutex> lock(mutex_);
    status_.active=false;
    status_.phase=success ? "completed" : "stopped";
    if (status_.detail.empty() || success) status_.detail=error;
    else if (!error.empty()) status_.detail+="; "+error;
    if (!status_.stationary_confirmed) status_.phase="stop_unconfirmed";
}
