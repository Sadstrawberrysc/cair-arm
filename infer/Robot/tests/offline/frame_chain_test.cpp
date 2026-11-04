#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <json.hpp>

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
    Eigen::Matrix4d camera=Eigen::Matrix4d::Identity();
    camera.topLeftCorner<3,3>()=Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
    camera.topRightCorner<3,1>()=Eigen::Vector3d(.03,.02,.16);
    const auto projected=chain.CameraPoseBase(pose, camera);
    ok &= Check(Near(projected.topRightCorner<3,1>(), pose.head<3>()
                     +rotation_base_from_arm_tip*camera.topRightCorner<3,1>()),
                "wrist camera translation uses ArmTip, not Probe TCP");
    ok &= Check(Near(projected.topLeftCorner<3,3>(), rotation_base_from_arm_tip*camera.topLeftCorner<3,3>()),
                "wrist camera rotation multiplication order");
    reference.tail<3>() += Eigen::Vector3d(0.01, -0.02, 0.03);
    ok &= Check(Near(chain.ArmTipOrientationDelta(pose, reference),
                     rotation_base_from_arm_tip
                         * RotationBaseFromControllerEuler(
                               reference.tail<3>()).transpose()),
                "orientation delta multiplication order is unchanged");

    // New installation: keep Sensor +30, add probe +30; TCP position stays fixed.
    ForceCalibration rotated = calibration;
    rotated.rotation_sensor_from_probe = calibration.rotation_tool_from_sensor;
    rotated.gravity_base_n = Eigen::Vector3d(.3, -.5, -5.0);
    rotated.force_bias_n = Eigen::Vector3d(.1, .2, .3);
    rotated.torque_bias_nm = Eigen::Vector3d(.01, .02, .03);
    rotated.center_of_mass_sensor_m = Eigen::Vector3d(.01, -.02, .04);
    rotated.sensor_id = "offline-test";
    rotated.created_at = "2026-10-29T00:00:00Z";
    rotated.probe_model = "Lprobe-IFS.STL";
    const CalibratedFrameChain rotated_chain(rotated);
    const Eigen::Matrix3d r_sensor_tool = rotated.rotation_sensor_from_probe;
    ok &= Check(Near(rotated_chain.RotationArmTipFromTool(),
                     Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d::UnitZ()).toRotationMatrix()),
                "Sensor +30 and probe +30 compose to +60 degrees");
    ok &= Check(Near(rotated_chain.ProbeTcpBase(pose), chain.ProbeTcpBase(pose))
                && Near(rotated_chain.ProbeTcpToolM(), r_sensor_tool.transpose()*expected_probe_tool),
                "probe axes rotate without moving TCP or changing the Tool origin");
    ok &= Check(Near(rotated_chain.ArmTipPoseFromTcp(rotated_chain.ToolTcpPoseBase(pose)),pose,1e-10),
                "rotated probe TCP roundtrip includes nonzero mount translation");
    const Eigen::Vector3d force_sensor(2., -3., 4.);
    const Eigen::Vector3d contact_sensor(.025, .015, .190);
    const Eigen::Vector3d torque_sensor = contact_sensor.cross(force_sensor);
    const Eigen::Matrix3d base_sensor = rotated_chain.RotationBaseFromSensor(pose);
    const Eigen::Vector3d gravity_sensor = base_sensor.transpose()*rotated.gravity_base_n;
    Eigen::Matrix<double,6,1> raw;
    raw.head<3>() = force_sensor + gravity_sensor + rotated.force_bias_n;
    raw.tail<3>() = torque_sensor + rotated.center_of_mass_sensor_m.cross(gravity_sensor)
                   + rotated.torque_bias_nm;
    const auto wrench = rotated.Compensate(raw, base_sensor);
    ok &= Check(wrench.valid && Near(wrench.sensor.head<3>(), force_sensor)
                && Near(wrench.sensor.tail<3>(), torque_sensor)
                && Near(base_sensor, chain.RotationBaseFromSensor(pose)),
                "probe rotation does not enter Sensor gravity compensation");
    const Eigen::Vector3d expected_force = r_sensor_tool.transpose()*force_sensor;
    const Eigen::Vector3d contact_tool = r_sensor_tool.transpose()
        *(contact_sensor-rotated.translation_sensor_to_tool_m);
    ok &= Check(Near(wrench.tool.head<3>(), expected_force)
                && Near(wrench.tool.tail<3>(), contact_tool.cross(expected_force))
                && Near(rotated_chain.ContactPointRelativeTcpTool(contact_sensor),
                        r_sensor_tool.transpose()*(contact_sensor-rotated.probe_tcp_sensor_m)),
                "force, shifted torque and TCP-relative contact share probe axes");
    std::string error;
    const auto json_path = std::filesystem::temp_directory_path()
        / ("uspilot-probe-rotation-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
    ForceCalibration loaded;
    ok &= Check(rotated.SaveJson(json_path.string(), &error)
                && loaded.LoadJson(json_path.string(), &error)
                && Near(loaded.rotation_sensor_from_probe, r_sensor_tool),
                "schema v2 preserves independent probe rotation");
    if (!std::filesystem::exists(json_path)) {
        std::cerr << "Calibration save failed: " << error << '\n';
        return 1;
    }
    nlohmann::json legacy;
    { std::ifstream stream(json_path); stream >> legacy; }
    legacy["schema_version"] = 1;
    legacy.erase("probe_to_sensor");
    { std::ofstream stream(json_path); stream << legacy; }
    ok &= Check(loaded.LoadJson(json_path.string(), &error)
                && Near(loaded.rotation_sensor_from_probe, Eigen::Matrix3d::Identity()),
                "loading legacy v1 resets probe axes to identity");
    legacy["schema_version"] = 2;
    { std::ofstream stream(json_path); stream << legacy; }
    ok &= Check(!loaded.LoadJson(json_path.string(), &error),
                "v2 cannot silently omit independent probe rotation");
    std::filesystem::remove(json_path);
    rotated.rotation_sensor_from_probe(0,0) = 2;
    ok &= Check(!rotated.Validate(&error), "invalid probe rotation is rejected");

    Eigen::Matrix<double, 7, 1> lhs =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> rhs = lhs;
    lhs[0] = M_PI - 0.01;
    rhs[0] = -M_PI + 0.01;
    ok &= Check(std::abs(MaximumWrappedJointDeltaDeg(lhs, rhs)
                         - 0.02 * 180.0 / M_PI) <= 1e-10,
                "stationary joint delta keeps wrapped-angle semantics");


    ok &= Check(Near(chain.ArmTipPoseFromTcp(chain.ToolTcpPoseBase(pose)),pose,1e-10),
                "TCP-to-ArmTip roundtrip includes mount and lever arm");
    std::deque<CalibratedFrameChain::CameraSample> history;
    Eigen::Matrix4d ta=Eigen::Matrix4d::Identity(), tb=ta, ti;
    tb(0,3)=.02;
    history.push_back({1000000000,ta}); history.push_back({1020000000,tb});
    double span=0;
    ok &= Check(CalibratedFrameChain::InterpolateCamera(history,1010000000,ti,span)
                && std::abs(ti(0,3)-.01)<1e-12 && span==20, "wrist camera pose interpolation");
    ok &= Check(!CalibratedFrameChain::InterpolateCamera(history,1030000000,ti,span),
                "wrist history never extrapolates");
    history.back().time_ns=1051000000;
    ok &= Check(!CalibratedFrameChain::InterpolateCamera(history,1010000000,ti,span),
                "wrist pose span above 50 ms rejected");
    return ok ? 0 : 1;
}
