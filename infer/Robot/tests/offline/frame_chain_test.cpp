#include <cmath>
#include <iostream>

#include <calibrated_frame_chain.hpp>

namespace {

bool Check(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

bool Near(const Eigen::MatrixXd& actual,
          const Eigen::MatrixXd& expected,
          double tolerance = 1e-12) {
    return (actual - expected).cwiseAbs().maxCoeff() <= tolerance;
}

}  // namespace

int main() {
    bool ok = true;
    ForceCalibration calibration;
    calibration.rotation_tool_from_sensor =
        Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    calibration.translation_sensor_to_tool_m =
        Eigen::Vector3d(0.001, -0.002, 0.003);
    calibration.probe_tcp_sensor_m =
        Eigen::Vector3d(0.004, 0.005, 0.188);
    const CalibratedFrameChain chain(calibration);

    const Eigen::Vector3d expected_probe_tool =
        calibration.probe_tcp_sensor_m
        - calibration.translation_sensor_to_tool_m;
    const Eigen::Vector3d expected_probe_arm_tip =
        calibration.rotation_tool_from_sensor * expected_probe_tool;
    ok &= Check(Near(chain.TranslationSensorToToolM(),
                     calibration.translation_sensor_to_tool_m)
                    && Near(chain.ProbeTcpSensorM(),
                            calibration.probe_tcp_sensor_m),
                "calibration translations are injected without modification");
    ok &= Check(Near(chain.ProbeTcpToolM(), expected_probe_tool),
                "Sensor-to-Tool translation order remains unchanged");
    ok &= Check(Near(chain.ProbeTcpArmTipM(), expected_probe_arm_tip),
                "Arm_Tip probe offset keeps rotation-before-pose semantics");

    Eigen::Matrix<double, 6, 1> pose;
    pose << 0.1, -0.2, 0.3, 0.15, -0.10, 0.25;
    const Eigen::Matrix3d rotation_base_from_arm_tip =
        RotationBaseFromControllerEuler(pose.tail<3>());
    const Eigen::Vector3d expected_probe_base = pose.head<3>()
        + rotation_base_from_arm_tip * expected_probe_arm_tip;
    ok &= Check(Near(chain.ProbeTcpBase(pose), expected_probe_base),
                "Base-to-Probe TCP formula is numerically equivalent");
    ok &= Check(Near(chain.RotationBaseFromTool(pose),
                     rotation_base_from_arm_tip
                         * calibration.rotation_tool_from_sensor),
                "Base-to-Tool rotation multiplication order is unchanged");
    ok &= Check(Near(chain.ToolYAxisBase(pose),
                     rotation_base_from_arm_tip
                         * calibration.rotation_tool_from_sensor.col(1)),
                "Tool-Y Base direction remains unchanged");

    Eigen::Matrix<double, 6, 1> reference = pose;
    reference.tail<3>() += Eigen::Vector3d(0.01, -0.02, 0.03);
    ok &= Check(Near(chain.ArmTipOrientationDelta(pose, reference),
                     rotation_base_from_arm_tip
                         * RotationBaseFromControllerEuler(
                               reference.tail<3>()).transpose()),
                "orientation delta multiplication order is unchanged");

    Eigen::Matrix<double, 7, 1> lhs =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> rhs = lhs;
    lhs[0] = M_PI - 0.01;
    rhs[0] = -M_PI + 0.01;
    ok &= Check(std::abs(MaximumWrappedJointDeltaDeg(lhs, rhs)
                         - 0.02 * 180.0 / M_PI) <= 1e-10,
                "stationary joint delta keeps wrapped-angle semantics");

    const RuntimeTareConfig tare_config;
    ok &= Check(tare_config.maximum_joint_span_deg == 0.02
                    && tare_config.maximum_tcp_span_mm == 0.35
                    && tare_config.maximum_orientation_span_deg == 0.10
                    && tare_config.maximum_force_deviation_n == 0.25
                    && tare_config.maximum_torque_deviation_nm == 0.02
                    && tare_config.minimum_samples_per_second == 20,
                "runtime tare safety thresholds keep their original values");
    CompensatedWrench compensated;
    compensated.valid = true;
    compensated.sensor << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
    compensated.tool = compensated.sensor;
    Eigen::Matrix<double, 6, 1> tare_offset;
    tare_offset << 0.5, 0.25, -0.5, 0.01, 0.02, 0.03;
    const Eigen::Matrix<double, 6, 1> original_tool = compensated.tool;
    ApplyRuntimeTare(tare_offset, compensated);
    ok &= Check(Near(compensated.sensor,
                     (Eigen::Matrix<double, 6, 1>()
                          << 0.5, 1.75, 3.5, 0.09, 0.18, 0.27).finished())
                    && Near(compensated.tool, original_tool),
                "runtime tare changes only the compensated Sensor frame");
    return ok ? 0 : 1;
}
