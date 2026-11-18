#pragma once
#include "collision/joint_planner.hpp"
#include "collision/monitor.hpp"
#include "collision/robot_read_only.hpp"
#include "joint_path_executor.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <json.hpp>

namespace uspilot::collision::app {
class CoarseExecution {
public:
    CoarseExecution(const RobotModel& model, const CollisionChecker& checker,
                    const RMConnectionConfig& connection, CheckOptions options, int movej_velocity_percent = 6)
        : model_(model), checker_(checker), robot_(connection,movej_velocity_percent), options_(options),
          movej_velocity_percent_(movej_velocity_percent) {
        if (model.ActiveJointNames().size()!=7) throw std::invalid_argument("Execution requires RM75 seven-axis model");
        for (int i=1; i<=7; ++i)
            if (std::find(model.ActiveJointNames().begin(),model.ActiveJointNames().end(),"joint"+std::to_string(i))==model.ActiveJointNames().end())
                throw std::invalid_argument("Execution requires joint1..joint7");
        options_.max_scene_age_ns=500000000;
        options_.joint_limit_margin_rad=std::max(options_.joint_limit_margin_rad,3*std::acos(-1.)/180);
    }
    ~CoarseExecution() {
        try {
            Shutdown();
            if (!reported_completion_) ReportCompletion();
        } catch (...) { std::cerr << "Collision execution shutdown failed; inspect robot stop state\n"; }
    }
    ReadOnlyRobotState Feedback() const {
        const auto s=robot_.Feedback();
        ReadOnlyRobotState out;
        out.received_at_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(s.received_at.time_since_epoch()).count();
        out.request_timing_available=false; // Shared receiver supplies host receive time, not query RTT.
        out.valid=s.valid && !s.stale && !s.arm_err && !s.sys_err;
        out.arm_error=s.arm_err; out.system_error=s.sys_err;
        if (out.valid) out.joints=Decode(s.joints);
        return out;
    }
    RMJointPathExecutor::Status Status() const {
        auto state=robot_.Snapshot();
        if (postcheck_failed_ && state.phase=="completed") {
            state.phase="goal_verification_failed"; state.detail="final_tcp_error_or_feedback_invalid";
        }
        return state;
    }
    void SetPlan(const PlanResult& plan, const nlohmann::json& target) {
        if (Status().active) throw std::runtime_error("Cannot replace path during execution");
        if (!reported_completion_) ReportCompletion();
        plan_=plan;
        tip_from_tcp_=Eigen::Isometry3d::Identity(); tip_link_="Link7";
        if (target.contains("T_armtip_tcp")) {
            tip_link_=target.at("arm_tip_link").get<std::string>();
            const auto& matrix=target.at("T_armtip_tcp");
            for (int r=0; r<4; ++r) for (int c=0; c<4; ++c) tip_from_tcp_.matrix()(r,c)=matrix.at(r).at(c).get<double>();
        }
        ready_=plan.Exact() && !plan.waypoints.empty();
    }
    void Start(const MonitorResult& observation, const std::function<bool()>& cancelled = {}) {
        if (!ready_) throw std::runtime_error("No exact plan available; press P first");
        if (Status().active) throw std::runtime_error("Execution already active");
        const auto feedback=robot_.Feedback();
        if (!observation.input_valid || !feedback.valid || feedback.stale || feedback.arm_err || feedback.sys_err)
            throw std::runtime_error("Fresh camera and healthy feedback required");
        if (plan_.model_id!=checker_.ModelId() || plan_.calibration_id!=observation.scene.calibration_id)
            throw std::runtime_error("Execution context differs from plan");
        const auto now=MonitorSteadyNowNs();
        if (now<observation.scene.captured_at_ns || now-observation.scene.captured_at_ns>options_.max_scene_age_ns)
            throw std::runtime_error("Fresh camera snapshot required");
        if ((feedback.joints-Encode(plan_.waypoints.front())).cwiseAbs().maxCoeff()>.5*std::acos(-1.)/180)
            throw std::runtime_error("Robot moved from plan start; press P to replan");
        std::vector<RMJointPathExecutor::Joints> path{feedback.joints};
        for (const auto& q : plan_.waypoints) path.push_back(Encode(q));
        path_=RMJointPathExecutor::Prepare(path);
        // Check every waypoint against the loaded URDF soft limits before any
        // Stop/MoveJ command. Validate the full path once on this frozen snapshot.
        for (const auto& q : path_) {
            const auto named=Decode(q);
            model_.Forward(named);
            for (const auto& j : model_.Joints()) if (j.type==JointType::kRevolute)
                if (named.at(j.name)<j.lower+options_.joint_limit_margin_rad ||
                    named.at(j.name)>j.upper-options_.joint_limit_margin_rad)
                    throw std::runtime_error("joint_limit_margin: "+j.name);
        }
        auto check_options=options_;
        check_options.allow_unknown_space=plan_.unknown_space_allowed;
        const auto check_time=MonitorSteadyNowNs();
        for (std::size_t i=1; i<path_.size(); ++i) {
            const auto check=checker_.CheckSegment(Decode(path_[i-1]),Decode(path_[i]),
                observation.scene,check_time,check_options,cancelled);
            if (!check.Valid()) throw std::runtime_error("preexecution_check: "+check.detail);
        }
        if (cancelled && cancelled()) throw std::runtime_error("execution_cancelled_before_start");
        robot_.Start(path_);
        ready_=false; reported_completion_=false; postcheck_failed_=false;
        std::cout << nlohmann::json({{"event","execution_started"},
            {"backend","movej"},{"velocity_percent",movej_velocity_percent_},{"blend",false},
            {"scene_supervision",false},
            {"unknown_space_allowed",plan_.unknown_space_allowed},{"segment_count",path_.size()-1}}).dump() << std::endl;
    }
    void Update() {
        if (!Status().active && !reported_completion_) ReportCompletion();
    }
    void Cancel(const std::string& reason) { robot_.Cancel(reason); }
    void Shutdown() {
        Cancel("operator_or_source_exit"); robot_.Join();
        if (!reported_completion_) ReportCompletion();
    }
private:
    void ReportCompletion() {
        robot_.Join();
        if (robot_.Snapshot().phase=="completed") {
            const auto state=robot_.Feedback();
            if (!state.valid || state.stale || state.arm_err || state.sys_err) postcheck_failed_=true;
            else {
                const auto actual=model_.Forward(Decode(state.joints)).at(tip_link_)*tip_from_tcp_;
                const auto expected=model_.Forward(plan_.waypoints.back()).at(tip_link_)*tip_from_tcp_;
                postcheck_failed_=(actual.translation()-expected.translation()).norm()>.005 ||
                    Eigen::AngleAxisd(actual.linear()*expected.linear().transpose()).angle()>2*std::acos(-1.)/180;
            }
        }
        const auto done=Status();
        nlohmann::json result={{"event","execution_result"},{"status",done.phase},
            {"backend","movej"},{"scene_supervision",false},
            {"detail",done.detail},{"stationary_confirmed",done.stationary_confirmed}};
        if (done.stop_diagnostics) {
            const auto& d=*done.stop_diagnostics;
            result["stop_diagnostics"]={{"segment",d.segment},{"phase",d.phase},
                {"reference_kind","active_joint_edge_projection"},
                {"joint","joint"+std::to_string(d.joint)},{"reference_deg",d.reference_deg},
                {"actual_deg",d.actual_deg},{"error_deg",d.error_deg},
                {"feedback_age_ms",d.feedback_age_ms},{"feedback_sequence",d.feedback_sequence},
                {"feedback_valid",d.feedback_valid},{"feedback_stale",d.feedback_stale},
                {"arm_error",d.arm_error},{"system_error",d.system_error}};
        }
        std::cout << result.dump() << std::endl;
        reported_completion_=true;
    }
    static JointPositions Decode(const RMJointPathExecutor::Joints& q) {
        JointPositions out;
        for (int i=0; i<7; ++i) out["joint"+std::to_string(i+1)]=q[i];
        return out;
    }
    static RMJointPathExecutor::Joints Encode(const JointPositions& q) {
        if (q.size()!=7) throw std::runtime_error("Expected exactly seven named joints");
        RMJointPathExecutor::Joints out;
        for (int i=0; i<7; ++i) out[i]=q.at("joint"+std::to_string(i+1));
        return out;
    }
    const RobotModel& model_;
    const CollisionChecker& checker_;
    RMJointPathExecutor robot_;
    CheckOptions options_;
    PlanResult plan_;
    std::string tip_link_="Link7";
    Eigen::Isometry3d tip_from_tcp_=Eigen::Isometry3d::Identity();
    std::vector<RMJointPathExecutor::Joints> path_;
    bool ready_=false, reported_completion_=true, postcheck_failed_=false;
    int movej_velocity_percent_;
};
}
