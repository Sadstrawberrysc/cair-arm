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
    return ok ? 0 : 1;
}
