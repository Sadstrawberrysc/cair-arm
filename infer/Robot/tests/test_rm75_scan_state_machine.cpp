#include <rm75_control.hpp>

#include <cmath>
#include <iostream>
#include <string>

namespace {

int failures = 0;

void Check(bool condition, const std::string& message) {
    if (condition) return;
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
}

Rm75ControlInput ValidInput(double force_z_n,
                            double robot_model_position_error_m = 0.0) {
    Rm75ControlInput input;
    input.robot_valid = true;
    input.wrench_valid = true;
    input.compensated_wrench_tool.z() = force_z_n;
    input.robot_model_position_error_m = robot_model_position_error_m;
    return input;
}

}  // namespace

int main() {
    Rm75ControlConfig config;
    config.cycle_s = 0.02;
    config.desired_force_n = -3.0;
    config.contact_threshold_n = 0.99;
    config.force_limit_z_n = 50.0;
    config.torque_limit_nm = 5.0;
    config.approach_speed_m_s = 0.01;
    config.approach_direction_tool_z = 1.0;
    config.max_force_axis_speed_m_s = 0.0005;
    config.max_angular_speed_rad_s = 0.2;
    config.model_y_direction = -1.0;
    config.model_y_velocity_gain_per_s = 0.5;
    config.rotate_align_y_scale = 0.3;
    config.model_rz_gain = 0.05;
    config.visual_y_enable_force_n = 2.0;
    config.visual_y_force_stable_duration_s = 0.06;
    config.scan_start_force_n = 2.0;
    config.scan_force_tolerance_n = 0.3;
    config.scan_force_stable_duration_s = 0.10;
    config.target_force_recovery_stable_duration_s = 0.10;
    config.scan_speed_m_s = 0.004;
    config.maximum_scan_distance_m = 0.0;
    config.scan_alignment_tolerance_m = 0.0008;
    config.scan_direction_tool_x = -1.0;
    config.trigger_alignment_tolerance_m = 0.008;
    config.trigger_alignment_stable_duration_s = 0.08;

    Rm75ControlLaw control(config);
    ControlIntent intent;
    intent.action_enabled = true;
    intent.desired_force_n = -3.0;
    intent.phase_index = 0;
    intent.model_y_m = 0.02;
    intent.model_rz_deg = 20.0;
    intent.sequence = 1;

    Rm75ControlOutput output = control.Step(ValidInput(0.0), intent, true);
    Check(output.state == Rm75SupervisorState::kApproach,
          "moving without contact must enter approach");
    Check(std::abs(output.requested_delta.z() - 0.0002) < 1e-12,
          "approach must move at 1.0 cm/s along +Tool-Z");
    Check(std::abs(output.requested_delta.x()) < 1e-15
              && std::abs(output.requested_delta.y()) < 1e-15
              && std::abs(output.requested_delta[5]) < 1e-15,
          "approach must ignore visual X/Y/RZ corrections");

    for (int cycle = 0; cycle < 3; ++cycle) {
        output = control.Step(ValidInput(-1.5), intent, true);
        Check(std::abs(output.requested_delta.y()) < 1e-15,
              "visual Y must remain blocked below the -2 N force gate");
    }
    for (int cycle = 0; cycle < 2; ++cycle) {
        output = control.Step(ValidInput(-2.1), intent, true);
        Check(std::abs(output.requested_delta.y()) < 1e-15,
              "visual Y must wait for the complete stable-force duration");
    }
    output = control.Step(ValidInput(-2.1), intent, true);
    Check(std::abs(output.requested_delta.y() - (-0.0002)) < 1e-12,
          "visual Y must become inverted proportional velocity after stable -2 N");
    output = control.Step(ValidInput(-2.1, 0.0021), intent, true);
    Check(output.visual_y_tracking_paused
              && std::abs(output.requested_delta.y()) < 1e-15,
          "Tool-Y integration must pause when real TCP tracking falls behind");
    output = control.Step(ValidInput(-2.1, 0.0015), intent, true);
    Check(output.visual_y_tracking_paused
              && std::abs(output.requested_delta.y()) < 1e-15,
          "Tool-Y tracking pause must remain latched inside hysteresis band");
    output = control.Step(ValidInput(-2.1, 0.0009), intent, true);
    Check(!output.visual_y_tracking_paused
              && std::abs(output.requested_delta.y() - (-0.0002)) < 1e-12,
          "Tool-Y integration must resume after the real TCP catches up");

    control.Reset();

    intent.model_y_m = 0.0;
    intent.model_rz_deg = 0.0;
    intent.sequence = 2;
    for (int cycle = 0; cycle < 4; ++cycle) {
        output = control.Step(ValidInput(-3.0), intent, true);
        Check(output.state == Rm75SupervisorState::kForceSettle,
              "contact must remain in force_settle until stable duration");
        Check(std::abs(output.requested_delta.x()) < 1e-15,
              "force_settle must keep Tool-X stopped");
    }
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.state == Rm75SupervisorState::kScan,
          "stable force with phase 0 and aligned Y must enter scan");
    Check(std::abs(output.requested_delta.x() - (-0.00008)) < 1e-12,
          "scan must move at 0.4 cm/s along configured negative Tool-X");

    // More-compressive-than-target force must immediately override scanning
    // with a pure -Tool-Z unloading step. It remains latched until the force
    // returns to the requested target, avoiding noisy direction chatter.
    output = control.Step(ValidInput(-4.0), intent, true);
    Check(output.target_force_unloading,
          "force beyond target margin must enter target-relative unloading");
    Check(output.state == Rm75SupervisorState::kForceSettle,
          "target-relative unloading must pause the scan state");
    Check(std::abs(output.requested_delta.x()) < 1e-15
              && std::abs(output.requested_delta.y()) < 1e-15
              && std::abs(output.requested_delta[5]) < 1e-15,
          "target-relative unloading must suppress X/Y/RZ motion");
    Check(std::abs(output.requested_delta.z() - (-0.0001)) < 1e-12,
          "one-newton overload must use the 0.5 cm/s maximum retreat");
    output = control.Step(ValidInput(-3.2), intent, true);
    Check(output.target_force_unloading
              && std::abs(output.requested_delta.z() - (-0.00002)) < 1e-12,
          "unloading speed must decrease linearly near the force target");
    output = control.Step(ValidInput(-3.05), intent, true);
    Check(output.target_force_unloading
              && output.requested_delta.z() < 0.0,
          "target-relative unloading must remain latched below target force");
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.target_force_recovering
              && std::abs(output.requested_delta.z()) < 1e-15,
          "release must brake unloading velocity to zero before admittance");
    output = control.Step(ValidInput(0.0), intent, true);
    Check(output.target_force_recovering
              && std::abs(output.requested_delta.z() - 0.0001) < 1e-12,
          "lost contact must be reacquired at 0.5 cm/s along +Tool-Z");
    output = control.Step(ValidInput(-2.5), intent, true);
    Check(std::abs(output.requested_delta.z() - 0.000002) < 1e-12,
          "reacquired contact must switch to low-speed ordinary admittance");

    // Complete independent -3 N restoration, then separately re-establish
    // the ordinary scan-ready force window.
    for (int cycle = 0; cycle < 5; ++cycle) {
        output = control.Step(ValidInput(-3.0), intent, true);
    }
    Check(output.target_force_recovering,
          "recovery output must remain active through its final stable cycle");
    for (int cycle = 0; cycle < 5; ++cycle) {
        output = control.Step(ValidInput(-3.0), intent, true);
    }
    Check(output.state == Rm75SupervisorState::kScan,
          "scan must resume only after force settles again");

    // Target-relative unloading is independent of the visual action gate.
    // Redis stale/invalid commands are represented by an action-disabled
    // intent with the configured force target retained.
    control.Reset();
    intent.action_enabled = false;
    intent.terminate = false;
    output = control.Step(ValidInput(-3.2), intent, true);
    Check(output.target_force_unloading
              && output.state == Rm75SupervisorState::kForceSettle,
          "idle or stale Redis must not suppress target-force unloading");
    Check(output.command_motion
              && std::abs(output.requested_delta.x()) < 1e-15
              && std::abs(output.requested_delta.y()) < 1e-15
              && std::abs(output.requested_delta.z() - (-0.00002)) < 1e-12
              && std::abs(output.requested_delta[3]) < 1e-15
              && std::abs(output.requested_delta[4]) < 1e-15
              && std::abs(output.requested_delta[5]) < 1e-15,
          "independent unloading must command pure -Tool-Z only");
    intent.terminate = true;
    output = control.Step(ValidInput(-3.05), intent, true);
    Check(output.target_force_unloading && !output.completed,
          "terminate must be deferred while independent unloading is active");
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(!output.target_force_unloading && !output.target_force_recovering
              && output.completed
              && output.state == Rm75SupervisorState::kArmed
              && !output.command_motion
              && std::abs(output.requested_delta.z()) < 1e-15,
          "terminate must complete at zero velocity without force reacquisition");

    // Plain idle follows the same rule: finish -Tool-Z unloading and braking,
    // then hold. It must never continue into automatic +Tool-Z reacquisition.
    control.Reset();
    intent.terminate = false;
    output = control.Step(ValidInput(-3.2), intent, true);
    Check(output.target_force_unloading
              && output.requested_delta.z() < 0.0,
          "idle must continue pure -Tool-Z while overload remains");
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.target_force_unloading
              && output.requested_delta.z() <= 0.0,
          "idle must finish zero-speed braking before holding");
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.state == Rm75SupervisorState::kArmed
              && !output.target_force_unloading
              && !output.target_force_recovering
              && !output.command_motion
              && output.requested_delta.cwiseAbs().maxCoeff() < 1e-15,
          "idle must hold after braking without +Tool-Z reacquisition");

    output = control.Step(ValidInput(-3.2), intent, false);
    Check(output.state == Rm75SupervisorState::kObserve
              && !output.command_motion,
          "independent unloading must still require an armed controller");

    // A previously enabled visual-Y path must be invalidated by unloading.
    // Force samples observed during retreat/braking must not count toward the
    // post-release 0.06 s qualification window used by this test profile.
    control.Reset();
    intent.action_enabled = true;
    intent.terminate = false;
    intent.model_y_m = 0.02;
    for (int cycle = 0; cycle < 3; ++cycle) {
        output = control.Step(ValidInput(-2.1), intent, true);
    }
    Check(std::abs(output.requested_delta.y() - (-0.0002)) < 1e-12,
          "precondition: visual Y must be enabled before unloading");
    output = control.Step(ValidInput(-3.2), intent, true);
    Check(output.target_force_unloading
              && std::abs(output.requested_delta.y()) < 1e-15,
          "entering unload must immediately disable visual Y");
    for (int cycle = 0; cycle < 4; ++cycle) {
        output = control.Step(ValidInput(-3.2), intent, true);
        Check(std::abs(output.requested_delta.y()) < 1e-15,
              "unload samples must not re-enable or qualify visual Y");
    }
    for (int cycle = 0; cycle < 2; ++cycle) {
        output = control.Step(ValidInput(-3.0), intent, true);
        Check(std::abs(output.requested_delta.y()) < 1e-15,
              "release braking must keep visual Y disabled");
    }
    Check(output.target_force_recovering,
          "braking completion must enter post-unload recovery");
    intent.action_enabled = false;
    output = control.Step(ValidInput(0.0), intent, true);
    Check(!output.target_force_unloading
              && !output.target_force_recovering
              && output.state == Rm75SupervisorState::kArmed
              && !output.command_motion
              && output.requested_delta.cwiseAbs().maxCoeff() < 1e-15,
          "Redis idle must cancel recovery and forbid +Tool-Z reacquisition");
    intent.action_enabled = true;
    output = control.Step(ValidInput(-1.5), intent, true);
    Check(!output.target_force_unloading && !output.target_force_recovering
              && std::abs(output.requested_delta.y()) < 1e-15,
          "weak post-release contact must not restore visual Y");
    for (int cycle = 0; cycle < 2; ++cycle) {
        output = control.Step(ValidInput(-2.1), intent, true);
        Check(std::abs(output.requested_delta.y()) < 1e-15,
              "post-release visual Y must wait for the full force window");
    }
    output = control.Step(ValidInput(-2.1), intent, true);
    Check(std::abs(output.requested_delta.y() - (-0.0002)) < 1e-12,
          "visual Y may resume only after a fresh post-release force window");

    intent.action_enabled = true;
    intent.terminate = false;
    intent.phase_index = 0;
    intent.model_y_m = 0.0;
    intent.model_rz_deg = 0.0;
    for (int cycle = 0; cycle < 5; ++cycle) {
        output = control.Step(ValidInput(-3.0), intent, true);
    }
    Check(output.state == Rm75SupervisorState::kScan,
          "trigger tests require a newly settled scan after safety reset");

    intent.phase_index = 1;
    intent.model_y_m = 0.009;
    intent.sequence = 3;
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.state == Rm75SupervisorState::kTriggerAlign,
          "phase 1 must stop scan and enter trigger_align");
    Check(std::abs(output.requested_delta.x()) < 1e-15,
          "trigger_align must keep Tool-X stopped");

    intent.model_y_m = 0.001;
    intent.model_rz_deg = -10.0;
    intent.sequence = 4;
    for (int cycle = 0; cycle < 3; ++cycle) {
        output = control.Step(ValidInput(-3.0), intent, true);
        Check(output.state == Rm75SupervisorState::kTriggerAlign,
              "trigger alignment must satisfy its stable duration");
        Check(std::abs(output.requested_delta[5]) < 1e-15,
              "RZ must remain paused before rotate_align");
    }
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.state == Rm75SupervisorState::kRotateAlign,
          "stable trigger alignment must latch rotate_align");
    Check(output.requested_delta[5] < 0.0,
          "rotate_align must preserve the negative RZ sign");

    intent.phase_index = 2;
    intent.sequence = 5;
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.state == Rm75SupervisorState::kRotateAlign,
          "phase jitter must not release the rotation latch");
    Check(std::abs(output.requested_delta.x()) < 1e-15,
          "latched rotate_align must keep Tool-X stopped");

    // Tool-Y is a continuous proportional velocity while RZ remains a finite
    // correction that a repeated sequence must drain without reloading.
    intent.model_y_m = 0.001;
    intent.model_rz_deg = -10.0;
    intent.sequence = 6;
    double accumulated_y_m = 0.0;
    double accumulated_rz_rad = 0.0;
    for (int cycle = 0; cycle < 20; ++cycle) {
        output = control.Step(ValidInput(-3.0), intent, true);
        accumulated_y_m += output.requested_delta.y();
        accumulated_rz_rad += output.requested_delta[5];
    }
    Check(std::abs(accumulated_y_m - (-0.00006)) < 1e-12,
          "rotate Y must integrate the inverted 0.3 * 0.5/s velocity gain");
    Check(std::abs(accumulated_rz_rad
                   - (-10.0 * 0.05 * M_PI / 180.0)) < 1e-12,
          "rotate RZ must preserve sign and apply the 0.05 gain once");
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(std::abs(output.requested_delta.y() - (-0.000003)) < 1e-12
              && std::abs(output.requested_delta[5]) < 1e-15,
          "same sequence must keep Y velocity active without reloading RZ");

    intent.model_y_m = 0.004;
    intent.model_rz_deg = 10.0;
    intent.sequence = 7;
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.requested_delta.y() < 0.0
              && output.requested_delta[5] > 0.0,
          "positive visual Y must produce negative Tool-Y without changing RZ");
    intent.model_y_m = -0.001;
    intent.model_rz_deg = -10.0;
    intent.sequence = 8;
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.requested_delta.y() > 0.0
              && output.requested_delta[5] < 0.0,
          "negative visual Y must produce positive Tool-Y immediately");

    intent.terminate = true;
    intent.action_enabled = false;
    intent.sequence = 9;
    output = control.Step(ValidInput(-3.0), intent, true);
    Check(output.state == Rm75SupervisorState::kArmed && output.completed,
          "terminate must clear motion and return to armed");
    Check(!output.command_motion, "terminate must not command motion");

    intent.terminate = false;
    intent.sequence = 10;
    output = control.Step(ValidInput(0.0), intent, true);
    Check(output.state == Rm75SupervisorState::kArmed && !output.completed,
          "idle handshake must remain armed without completing again");

    if (failures != 0) {
        std::cerr << failures << " RM75 scan state-machine test(s) failed\n";
        return 1;
    }
    std::cout << "RM75 scan state-machine tests passed\n";
    return 0;
}
