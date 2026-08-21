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
    const RMCommand default_command;
    ok &= Check(default_command.ConnectionConfig().ip == "192.168.50.254"
                    && default_command.ConnectionConfig().port == 8080,
                "RMCommand default endpoint remains unchanged");
    const RMCommand configured_command(
        RMConnectionConfig{"127.0.0.1", 18080});
    ok &= Check(configured_command.ConnectionConfig().ip == "127.0.0.1"
                    && configured_command.ConnectionConfig().port == 18080,
                "RMCommand connection endpoint is injected once");

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

    return ok ? 0 : 1;
}
