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
        input.fresh=true; input.target_id="noise"; input.request_token="begin"; input.action="begin";
        input.normal_base=Eigen::Vector3d(0,0,-1);
        auto first=filtered.Step(input);
        filtered.Commit(first.tcp_reference);
        input.action="heartbeat";
        input.normal_base=Eigen::AngleAxisd(4.*M_PI/180.,Eigen::Vector3d::UnitY())*Eigen::Vector3d(0,0,-1);
        auto small=filtered.Step(input);
        ok &= Check((small.tcp_goal.topLeftCorner<3,3>()-first.tcp_goal.topLeftCorner<3,3>()).norm()<1e-12,
                    "four-degree normal change retains orientation goal");
        input.normal_base=Eigen::AngleAxisd(6.*M_PI/180.,Eigen::Vector3d::UnitY())*Eigen::Vector3d(0,0,-1);
        auto large=filtered.Step(input);
        ok &= Check((large.tcp_goal.topLeftCorner<3,3>().col(2)+input.normal_base).norm()<1e-12,
                    "real tilt beyond deadband updates orientation goal");
        ok &= Check(Eigen::Quaterniond(first.tcp_reference.topLeftCorner<3,3>()).angularDistance(
                    Eigen::Quaterniond(large.tcp_reference.topLeftCorner<3,3>()))<=5.*M_PI/180.*input.cycle_s+1e-12,
                    "filtered orientation retains angular speed bound");
    }

    WristFollowController wrist;
    WristFollowInput wi;
    wi.fresh=true; wi.target_id="target1"; wi.request_token="producer:1"; wi.action="begin";
    wi.normal_base=Eigen::Vector3d(0,0,-1);
    wi.surface_base=Eigen::Vector3d(.01,0,.10);
    auto wo=wrist.Step(wi);
    ok &= Check(wo.following && wo.reference_reset, "wrist explicit begin resets measured reference");
    ok &= Check((wo.tcp_goal.topRightCorner<3,1>()-Eigen::Vector3d(.01,0,.05)).norm()<1e-12,
                "wrist adds exactly 50 mm outward");
    ok &= Check(wo.tcp_reference.topRightCorner<3,1>().norm()<=.000050001,
                "wrist speed at most 5 mm/s");
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
    ok &= Check(wo.following && wo.reference_reset && std::abs(wo.tcp_reference(0,3)-.2)<.000051,
                "wrist resume rebases on actual TCP");
    wi.request_token="producer:4"; wi.action="pause";
    ok &= Check(!wrist.Step(wi).following, "pause remains explicit Hold");
    wi.action="resume"; wi.target_id="target3"; wi.request_token="producer:5";
    wi.normal_base=Eigen::Vector3d(.1,0,-1).normalized();
    wo=wrist.Step(wi);
    ok &= Check(wo.following && Eigen::AngleAxisd(wo.tcp_reference.topLeftCorner<3,3>()).angle()
                <=5*M_PI/180.*.01+1e-9, "angular reference limited to 5 deg/s");
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
