#include <cmath>
#include <iostream>
#include <limits>

#include <rm75_control.hpp>

namespace {

bool Check(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

}  // namespace

int main() {
    bool ok = true;
    {
        Rm75ControlInput visual_input;
        visual_input.robot_valid = visual_input.wrench_valid = true;
        visual_input.compensated_wrench_tool.z() = -2.0;
        ControlIntent visual_intent;
        visual_intent.action_enabled = true;
        visual_intent.desired_force_n = -2.0;
        // Right-of-center image publishes negative Y; existing RM75 mapping
        // must produce +Tool-Y. Left-of-center must produce the reverse.
        for (double error : {-0.004, 0.004}) {
            Rm75ControlLaw visual_control;
            visual_intent.model_y_m = error;
            const auto aligned = visual_control.Step(visual_input, visual_intent, true);
            ok &= Check(std::abs(aligned.requested_delta.y() + error * 0.005) < 1e-12,
                        "image centering preserves Tool-Y direction and 4 mm/s speed bound");
        }
        Rm75ControlLaw no_contact_control;
        visual_input.compensated_wrench_tool.z() = -0.5;
        ok &= Check(no_contact_control.Step(visual_input, visual_intent, true)
                        .requested_delta.y() == 0.0,
                    "image centering retains no-contact lateral force gate");
    }
    Rm75ControlLaw control;
    Rm75ControlInput input;
    input.robot_valid = true;
    input.wrench_valid = true;
    ControlIntent intent;
    intent.desired_force_n = -2.0;

    Rm75ControlOutput output = control.Step(input, intent, false);
    ok &= Check(output.state == Rm75SupervisorState::kObserve,
                "disarmed valid control remains Observe");

    output = control.Step(input, intent, true);
    ok &= Check(output.state == Rm75SupervisorState::kArmed
                    && !output.command_motion,
                "idle armed control must not request motion");

    input.wrench_valid = false;
    output = control.Step(input, intent, true);
    ok &= Check(output.state == Rm75SupervisorState::kHold,
                "invalid wrench enters Hold");
    input.wrench_valid = true;
    intent.model_y_m = std::numeric_limits<double>::quiet_NaN();
    output = control.Step(input, intent, true);
    ok &= Check(output.state == Rm75SupervisorState::kFault,
                "non-finite intent enters Fault");

    Rm75ServoPlannerConfig planner_config;
    planner_config.allow_near_singularity = true;
    Rm75ServoPlanner planner(planner_config);
    Eigen::Matrix<double, 7, 1> joints =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 1> pose = planner.PoseFromJoints(joints);
    Eigen::Matrix<double, 6, 1> non_finite_pose = pose;
    non_finite_pose[0] = std::numeric_limits<double>::infinity();
    Rm75ServoPlan plan = planner.Plan(joints, pose, non_finite_pose);
    ok &= Check(!plan.valid && plan.error == Rm75PlanError::kNonFiniteInput,
                "planner rejects non-finite desired pose");

    const Eigen::Matrix<double, 7, 1> excessive_previous =
        Eigen::Matrix<double, 7, 1>::Ones();
    plan = planner.Plan(joints, pose, pose, excessive_previous);
    ok &= Check(!plan.valid
                    && plan.error == Rm75PlanError::kPreviousStepOutOfBounds,
                "planner rejects previous step outside speed envelope");

    Rm75ServoPlannerConfig wrist_speed_config = planner_config;
    wrist_speed_config.max_joint_speed_deg_s = 2.0;
    wrist_speed_config.max_joint_accel_deg_s2 = 0.0;
    Rm75ServoPlanner wrist_speed_planner(wrist_speed_config);
    Eigen::Matrix<double, 7, 1> moved_joints = joints;
    moved_joints[0] += 10.0 * M_PI / 180.0;
    const auto moved_pose = wrist_speed_planner.PoseFromJoints(moved_joints);
    const auto wrist_speed_plan = wrist_speed_planner.Plan(joints, pose, moved_pose);
    const double joint_step_limit_rad = M_PI / 180.0 * 0.01;
    ok &= Check(wrist_speed_plan.valid
                    && wrist_speed_plan.joint_delta.cwiseAbs().maxCoeff()
                           <= joint_step_limit_rad + 1e-12,
                "wrist planner caps every 5 ms joint step at 2 deg/s");
    ok &= Check(wrist_speed_plan.valid
                    && wrist_speed_plan.joint_delta.cwiseAbs().maxCoeff()
                           >= joint_step_limit_rad * 0.99,
                "joint speed cap is exercised by a large target change");
    Eigen::Matrix<double, 7, 1> previous_too_fast =
        Eigen::Matrix<double, 7, 1>::Zero();
    previous_too_fast[0] = joint_step_limit_rad * 1.1;
    plan = wrist_speed_planner.Plan(joints, pose, moved_pose, previous_too_fast);
    ok &= Check(!plan.valid && plan.error == Rm75PlanError::kPreviousStepOutOfBounds,
                "wrist planner rejects a previous joint step above 2 deg/s");

    Eigen::Matrix<double, 7, 1> outside_limits = joints;
    outside_limits[0] = 10.0;
    const Eigen::Matrix<double, 6, 1> outside_pose =
        planner.PoseFromJoints(outside_limits);
    plan = planner.Plan(outside_limits, outside_pose, outside_pose);
    ok &= Check(!plan.valid && plan.error == Rm75PlanError::kJointLimit,
                "planner rejects joint target outside hard limits");

    Eigen::Matrix<double, 7, 1> near_limit = joints;
    near_limit[0] = planner.Kinematics().JointMaximums()[0]
        - 2.5 * M_PI / 180.0;
    const Eigen::Matrix<double, 6, 1> near_limit_pose =
        planner.PoseFromJoints(near_limit);
    plan = planner.Plan(near_limit, near_limit_pose, near_limit_pose);
    ok &= Check(!plan.valid
                    && plan.error == Rm75PlanError::kJointLimitMargin
                    && plan.detail.find("J1 target=") != std::string::npos
                    && plan.detail.find("limits=[") != std::string::npos,
                "joint margin rejection identifies joint, target and limits");

    Rm75ServoPlannerConfig artery_planner_config;
    artery_planner_config.allow_near_singularity = true;
    artery_planner_config.joint_limit_warning_deg = 5.0;
    Rm75ServoPlanner artery_planner(artery_planner_config);
    Eigen::Matrix<double, 7, 1> outside_artery_warning = joints;
    outside_artery_warning[0] = artery_planner.Kinematics().JointMaximums()[0]
        - 6.0 * M_PI / 180.0;
    Eigen::Matrix<double, 6, 1> artery_pose =
        artery_planner.PoseFromJoints(outside_artery_warning);
    plan = artery_planner.Plan(outside_artery_warning, artery_pose, artery_pose);
    ok &= Check(plan.valid && !plan.near_joint_limit,
                "artery maintenance planner accepts at least 5 deg margin");
    Eigen::Matrix<double, 7, 1> inside_artery_warning = joints;
    inside_artery_warning[0] = artery_planner.Kinematics().JointMaximums()[0]
        - 4.0 * M_PI / 180.0;
    artery_pose = artery_planner.PoseFromJoints(inside_artery_warning);
    plan = artery_planner.Plan(inside_artery_warning, artery_pose, artery_pose);
    ok &= Check(plan.valid && plan.near_joint_limit,
                "artery maintenance planner warns below 5 deg margin");
    Rm75ServoPlannerConfig probe_config;
    probe_config.joint_limit_warning_deg = 3.0;
    Rm75ServoPlanner probe_planner(probe_config);
    Eigen::Matrix<double, 7, 1> probe_joints;
    probe_joints << -23.919, -4.030, 55.033, 131.9, -102.653, 52.930, 85.403;
    probe_joints *= M_PI / 180.0;
    auto probe_pose = probe_planner.PoseFromJoints(probe_joints);
    plan = probe_planner.Plan(probe_joints, probe_pose, probe_pose);
    ok &= Check(plan.valid && !plan.near_joint_limit,
                "probe planner accepts J4 margin 3.1 deg");
    probe_joints[3] = 132.1 * M_PI / 180.0;
    probe_pose = probe_planner.PoseFromJoints(probe_joints);
    plan = probe_planner.Plan(probe_joints, probe_pose, probe_pose);
    ok &= Check(!plan.valid && plan.error == Rm75PlanError::kJointLimitMargin,
                "probe planner retains hard stop at J4 margin 2.9 deg");

    {
        WristFollowController filtered;
        WristFollowInput input;
        input.producer_id="producer"; input.capture_ns=1000000000;
        input.fresh=true; input.target_id="noise"; input.request_token="begin"; input.action="begin";
        input.normal_base=Eigen::Vector3d(0,0,-1);
        auto first=filtered.Step(input);
        filtered.Commit(first.tcp_reference);
        input.action="heartbeat";
        input.normal_base=Eigen::AngleAxisd(14.*M_PI/180.,Eigen::Vector3d::UnitY())*Eigen::Vector3d(0,0,-1);
        auto small=filtered.Step(input);
        ok &= Check((small.tcp_goal.topLeftCorner<3,3>()-first.tcp_goal.topLeftCorner<3,3>()).norm()<1e-12,
                    "fourteen-degree normal change retains orientation goal");
        input.normal_base=Eigen::AngleAxisd(16.*M_PI/180.,Eigen::Vector3d::UnitY())*Eigen::Vector3d(0,0,-1);
        auto large=filtered.Step(input);
        ok &= Check((large.tcp_goal.topLeftCorner<3,3>().col(2)+input.normal_base).norm()<1e-12,
                    "real tilt beyond deadband updates orientation goal");
        const double angle_step = Eigen::Quaterniond(first.tcp_reference.topLeftCorner<3,3>()).angularDistance(
            Eigen::Quaterniond(large.tcp_reference.topLeftCorner<3,3>()));
        ok &= Check(std::abs(angle_step-4.*M_PI/180.*input.cycle_s) < 1e-12,
                    "wrist orientation advances at 4 deg/s for a distant goal");
    }

    {
        WristFollowController wrist_position;
        WristFollowInput input;
        input.fresh = true;
        input.producer_id = "producer";
        input.target_id = "deadband";
        input.request_token = "producer:begin";
        input.action = "begin";
        input.capture_ns = 1'000'000'000;
        input.normal_base = -Eigen::Vector3d::UnitZ();
        input.surface_base = Eigen::Vector3d(.007, 0., .05);
        const auto first = wrist_position.Step(input);
        ok &= Check(first.following && !first.motion_requested
                    && first.tcp_reference.topRightCorner<3,1>().isZero(1e-12),
                    "position remains still at the 7 mm start threshold");

        input.capture_ns += 33'000'000;
        input.surface_base.x() = .008;
        const auto moving = wrist_position.Step(input);
        ok &= Check(moving.motion_requested && std::abs(moving.tcp_reference(0,3)-.004*input.cycle_s)<1e-12,
                    "position starts outside the deadband and advances at the reference speed limit");
        wrist_position.Commit(moving.tcp_reference);

        input.capture_ns += 33'000'000;
        input.surface_base.x() = moving.tcp_reference(0,3) + .004;
        const auto within_hysteresis = wrist_position.Step(input);
        ok &= Check(within_hysteresis.motion_requested,
                    "an active translation continues inside the start threshold");
        wrist_position.Commit(within_hysteresis.tcp_reference);

        input.capture_ns += 33'000'000;
        input.surface_base.x() = within_hysteresis.tcp_reference(0,3) + .0029;
        const auto settled = wrist_position.Step(input);
        ok &= Check(settled.following && !settled.motion_requested
                    && (settled.tcp_reference-within_hysteresis.tcp_reference).norm()<1e-12,
                    "position stops inside 3 mm without dropping follow authorization");
        input.capture_ns += 33'000'000;
        input.surface_base.x() = settled.tcp_reference(0,3) - .006;
        ok &= Check(!wrist_position.Step(input).motion_requested,
                    "small reversals after settling do not restart translation");

        input.capture_ns += 33'000'000;
        input.normal_base = Eigen::AngleAxisd(20.*M_PI/180., Eigen::Vector3d::UnitY())
            * -Eigen::Vector3d::UnitZ();
        input.surface_base = settled.tcp_reference.topRightCorner<3,1>() - .050*input.normal_base;
        const auto orienting = wrist_position.Step(input);
        ok &= Check(orienting.motion_requested
                    && (orienting.tcp_reference.topRightCorner<3,1>()
                        -settled.tcp_reference.topRightCorner<3,1>()).norm()<1e-12,
                    "position deadband still allows an orientation-only correction");
    }

    {
        WristFollowInput input;
        input.fresh = true;
        input.producer_id = "producer";
        input.target_id = "pose-wait";
        input.request_token = "producer:begin";
        input.action = "begin";
        input.capture_ns = 1'000'000'000;
        input.normal_base = -Eigen::Vector3d::UnitZ();
        input.surface_base.z() = .1;
        WristFollowController late_feedback, paused, rejected;
        const auto first = late_feedback.Step(input);
        late_feedback.Commit(first.tcp_reference);
        paused.Step(input);
        rejected.Step(input);

        input.capture_ns += 33'000'000;
        input.fresh = false;
        input.awaiting_pose_feedback = true;
        input.reject_reason = "wrist_pose_time_unmatched";
        for (double waited_s = 0.; waited_s < .050; waited_s += input.cycle_s) {
            const auto waiting = late_feedback.Step(input);
            ok &= Check(waiting.following && waiting.waiting_for_pose
                        && (waiting.tcp_reference-first.tcp_reference).norm()<1e-12,
                        "the reference stays frozen while eligible feedback is pending, including beyond 20 ms");
        }
        paused.Step(input);
        rejected.Step(input);

        auto pause = input;
        pause.request_token = "producer:pause";
        pause.action = "pause";
        const auto stopped = paused.Step(pause);
        ok &= Check(!stopped.following && !stopped.waiting_for_pose && stopped.reason == "operator_paused",
                    "operator pause wins while awaiting pose feedback");
        auto invalid = input;
        invalid.awaiting_pose_feedback = false;
        invalid.reject_reason = "wrist_observation_missing_stale_or_mismatched";
        const auto lost = rejected.Step(invalid);
        ok &= Check(!lost.following && lost.reason == invalid.reject_reason,
                    "an observation that becomes invalid or stale ends the wait immediately");

        auto matched = input;
        matched.fresh = true;
        matched.awaiting_pose_feedback = false;
        matched.surface_base.x() = .01;
        const auto continued = late_feedback.Step(matched);
        ok &= Check(continued.following && !continued.waiting_for_pose && !continued.reference_reset,
                    "feedback arriving after the old 20 ms limit continues the same follow session");
        const Eigen::Vector3d expected_goal = matched.surface_base + .050*matched.normal_base;
        ok &= Check((continued.tcp_goal.topRightCorner<3,1>()-expected_goal).norm()<1e-12,
                    "the first matched frame supplies the entire new surface position");
        ok &= Check((continued.tcp_reference.topRightCorner<3,1>()-
                     first.tcp_reference.topRightCorner<3,1>()).norm()<=.004*input.cycle_s+1e-12,
                    "the Cartesian reference still limits the step toward the latest surface position");
        --matched.capture_ns;
        const auto backwards = late_feedback.Step(matched);
        ok &= Check(!backwards.following && backwards.reason == "wrist_capture_time_reversed",
                    "an older observation still causes Hold without position filtering");
    }

    WristFollowController wrist;
    WristFollowInput wi;
    wi.producer_id="producer"; wi.capture_ns=1000000000;
    wi.fresh=true; wi.target_id="target1"; wi.request_token="producer:1"; wi.action="begin";
    wi.normal_base=Eigen::Vector3d(0,0,-1);
    wi.surface_base=Eigen::Vector3d(.01,0,.10);
    auto wo=wrist.Step(wi);
    ok &= Check(wo.following && wo.reference_reset, "wrist explicit begin resets measured reference");
    ok &= Check((wo.tcp_goal.topRightCorner<3,1>()-Eigen::Vector3d(.01,0,.05)).norm()<1e-12,
                "wrist adds exactly 50 mm outward");
    ok &= Check(std::abs(wo.tcp_reference.topRightCorner<3,1>().norm()-.004*wi.cycle_s)<1e-12,
                "wrist TCP reference advances at 4 mm/s for a distant goal");
    ok &= Check((wo.tcp_goal.topLeftCorner<3,3>().col(2)-Eigen::Vector3d(0,0,1)).norm()<1e-12,
                "wrist Tool Z points inward");
    wrist.Commit(wo.tcp_reference);
    wi.fresh=false; wi.reject_reason="camera_lost";
    ok &= Check(!wrist.Step(wi).following, "wrist loss holds");
    wi.fresh=true;
    ok &= Check(!wrist.Step(wi).following, "old begin cannot auto resume");
    wi.action="resume"; wi.request_token="producer:2";
    ok &= Check(!wrist.Step(wi).following, "resume needs a new click");
    wi.target_id="target2"; wi.request_token="producer:3"; wi.actual_tcp(0,3)=.2;
    wo=wrist.Step(wi);
    const Eigen::Vector3d resume_step = wo.tcp_reference.topRightCorner<3,1>()
        - wi.actual_tcp.topRightCorner<3,1>();
    const Eigen::Vector3d resume_goal_delta = wo.tcp_goal.topRightCorner<3,1>()
        - wi.actual_tcp.topRightCorner<3,1>();
    ok &= Check(wo.following && wo.reference_reset && resume_step.norm()<=.004*wi.cycle_s+1e-12
                && resume_step.dot(resume_goal_delta)>0.,
                "wrist resume rebases on actual TCP");
    wi.request_token="producer:4"; wi.action="pause";
    ok &= Check(!wrist.Step(wi).following, "pause remains explicit Hold");
    wi.action="resume"; wi.target_id="target3"; wi.request_token="producer:5";
    wi.normal_base=Eigen::Vector3d(.1,0,-1).normalized();
    wo=wrist.Step(wi);
    ok &= Check(wo.following && Eigen::AngleAxisd(wo.tcp_reference.topLeftCorner<3,3>()).angle()
                <=4.*M_PI/180.*wi.cycle_s+1e-9, "angular reference limited to 4 deg/s");
    wrist.Hold("planner_rejected");
    ok &= Check(!wrist.Step(wi).following, "planner rejection is latched");
    wi.normal_base=Eigen::Vector3d(1,0,0); wi.target_id="degenerate";
    wi.request_token="producer:6";
    wo=wrist.Step(wi);
    ok &= Check(!wo.following && wo.reason=="tangent_orientation_degenerate",
                "degenerate Tool-X projection holds");
    wi.normal_base=Eigen::Vector3d(0,0,-1);
    WristFollowController early;
    wi.action="begin"; wi.request_token="early:1"; wi.target_id="early"; wi.fresh=false;
    ok &= Check(!early.Step(wi).following && early.ResumeRequired(), "premature request rejected explicitly");
    wi.fresh=true;
    ok &= Check(!early.Step(wi).following, "premature begin cannot activate on later frames");
    wi.action="resume"; wi.request_token="early:2"; wi.target_id="new_click";
    ok &= Check(early.Step(wi).following, "rejected start can recover with explicit new click");
    return ok ? 0 : 1;
}
