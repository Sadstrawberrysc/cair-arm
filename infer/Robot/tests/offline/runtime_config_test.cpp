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
    return ok ? 0 : 1;
}
