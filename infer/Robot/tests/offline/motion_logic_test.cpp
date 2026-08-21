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
    return ok ? 0 : 1;
}
