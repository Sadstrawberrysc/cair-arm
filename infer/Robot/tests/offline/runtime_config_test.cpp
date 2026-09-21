#include <iostream>
#include <string>

#include <rm75_runtime_config.hpp>

namespace {

bool Check(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

}  // namespace

int main() {
    bool ok = true;
    RobotRuntimeConfig config = MakeImplicitRm75ProductionConfig(
        "/opt/rm75/bin", "20260820_120000");
    std::string error;
    ok &= Check(ValidateRobotRuntimeConfig(config, &error),
                "implicit production configuration must validate");
    ok &= Check(config.control.desired_force_n == -2.0,
                "effective force target remains -2 N");
    ok &= Check(config.control.approach_speed_m_s == 0.020,
                "effective approach speed remains 2 cm/s");
    ok &= Check(config.control.scan_speed_m_s == 0.010,
                "effective scan speed remains 1 cm/s");
    ok &= Check(config.safety.raw_force_limit_n == 50.0
                    && config.safety.raw_torque_limit_nm == 5.0,
                "raw wrench gates remain 50 N / 5 N*m");
    ok &= Check(config.safety.max_tracking_joint_error_deg == 20.0
                    && config.safety.max_tracking_position_error_mm == 25.0
                    && config.safety.max_tracking_orientation_error_deg == 0.0,
                "tracking gates remain 20 deg / 25 mm / disabled orientation");
    ok &= Check(config.safety.maximum_no_contact_approach_distance_m == 0.250
                    && config.safety.maximum_orientation_excursion_deg == 0.0,
                "implicit no-contact envelope remains 250 mm / 0 deg");
    ok &= Check(config.runtime_log_path.find("3n") == std::string::npos,
                "profile log name no longer claims 3 N");

    config.control.desired_force_n = -3.1;
    ok &= Check(!ValidateRobotRuntimeConfig(config, &error),
                "out-of-envelope force target is rejected");
    config = MakeImplicitRm75ProductionConfig(
        "/opt/rm75/bin", "20260820_120000");
    config.safety.raw_force_limit_n = 51.0;
    ok &= Check(!ValidateRobotRuntimeConfig(config, &error),
                "raw force gate cannot exceed 50 N");
    config=MakeImplicitRm75ProductionConfig("/opt/rm75/bin", "test");
    config.wrist_projection_calibration="wrist.json";
    config.wrist_global_calibration="global.json";
    config.publish_every=2;
    ok &= Check(!ValidateRobotRuntimeConfig(config, &error), "wrist projection rejects execute");
    config.mode=ControllerMode::kDryRun;
    ok &= Check(!ValidateRobotRuntimeConfig(config, &error), "projection alone does not enable planner");
    config.mode=ControllerMode::kObserve;
    ok &= Check(ValidateRobotRuntimeConfig(config, &error), "wrist projection observe accepted");
    config.redis_enabled=false;
    ok &= Check(!ValidateRobotRuntimeConfig(config, &error), "wrist projection needs state transport");
    config.redis_enabled=true;
    config.wrist_projection_calibration.clear();
    ok &= Check(!ValidateRobotRuntimeConfig(config, &error), "partial wrist calibration configuration rejected");
    config=RobotRuntimeConfig{}; config.calibration_path="force.json";
    config.wrist_follow_calibration="wrist.json"; config.publish_every=1;
    ok &= Check(ValidateRobotRuntimeConfig(config,&error), "seed-free wrist observe accepted");
    config.mode=ControllerMode::kDryRun;
    ok &= Check(ValidateRobotRuntimeConfig(config,&error), "wrist dry run accepted");
    config.wrist_global_calibration="global.json";
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "click and seed modes mutually exclusive");
    config.wrist_global_calibration.clear(); config.mode=ControllerMode::kExecute;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "wrist execute needs explicit confirmation");
    config.confirm_wrist_follow=true; config.duration_s=30;
    ok &= Check(ValidateRobotRuntimeConfig(config,&error), "bounded confirmed wrist execute config accepted");
    config.allow_provisional_force_control=true;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "wrist rejects provisional force bypass");
    config=RobotRuntimeConfig{}; config.calibration_path="force.json";
    config.wrist_no_force=true;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "no-force forbidden outside wrist mode");
    config.wrist_follow_calibration="wrist.json"; config.publish_every=1;
    ok &= Check(ValidateRobotRuntimeConfig(config,&error), "no-force wrist observe accepted");
    config.tare_no_contact_s=2;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "no-force forbids tare");
    config.tare_no_contact_s=0; config.wrist_candidate_trial=true; config.duration_s=30;
    config.mode=ControllerMode::kExecute; config.confirm_wrist_follow=true; config.wrist_execute_requested=true;
    ok &= Check(ValidateRobotRuntimeConfig(config,&error), "explicit bounded candidate trial accepted");
    config.duration_s=0;
    ok &= Check(ValidateRobotRuntimeConfig(config,&error), "candidate trial accepts continuous lifetime");
    config.duration_s=-1;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "negative duration rejected");
    config.duration_s=31;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "candidate duration bounded");
    config.duration_s=30; config.wrist_no_force=false;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "candidate trial requires no force mode");
    config.wrist_no_force=true; config.safety.maximum_orientation_excursion_deg=0;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "candidate cannot disable angular envelope");
    config.safety.maximum_orientation_excursion_deg=5; config.safety.maximum_no_contact_approach_distance_m=.006;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "candidate cannot enlarge distance envelope");
    config.safety.maximum_no_contact_approach_distance_m=.005;
    config.wrist_unlimited_excursion=true;
    ok &= Check(ValidateRobotRuntimeConfig(config,&error), "explicit candidate can disable total excursion gates");
    config.wrist_candidate_trial=false;
    ok &= Check(!ValidateRobotRuntimeConfig(config,&error), "excursion override forbidden in ordinary wrist mode");
    RobotRuntimeConfig polling;
    ok &= Check(polling.StatePollPeriodMs()==40, "ordinary robot polling remains 40 ms");
    ok &= Check(polling.EffectiveSafety().max_tracking_position_error_mm==25.0, "ordinary tracking error remains 25 mm");
    polling.wrist_follow_calibration="wrist.json";
    ok &= Check(polling.EffectiveSafety().max_tracking_position_error_mm==50.0, "wrist tracking error uses 50 mm");
    ok &= Check(polling.StatePollPeriodMs()==10, "wrist polling uses requested 10 ms period");
    return ok ? 0 : 1;
}
