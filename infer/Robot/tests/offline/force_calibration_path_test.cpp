#include <force_calibration_path.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>

#include <Eigen/Geometry>
#include <realman_kinematics.hpp>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

bool Check(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

}  // namespace

int main() {
    bool ok = true;
    RMKinematics kinematics;
    Eigen::Matrix<double, 7, 1> center;
    center << 0, -20, 0, 60, 0, 40, 0;
    center *= kDegToRad;
    const Eigen::Matrix3d sensor_rotation =
        Eigen::AngleAxisd(30.0 * kDegToRad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    ForceCalibrationPathOptions options;
    ForceCalibrationPath path;
    std::string error;
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                             options, path, &error), error.c_str());
    ok &= Check(path.joints_rad.size() == 1000 && path.gravity_directions_sensor.size() == 1000,
                "default plan contains exactly 1000 static samples");
    std::set<std::string> unique;
    for (std::size_t i = 0; i < path.joints_rad.size(); ++i) {
        const auto& joints = path.joints_rad[i];
        ok &= Check((joints.head<4>() - center.head<4>()).cwiseAbs().maxCoeff() <= 20.0*kDegToRad + 1e-12,
                    "J1-J4 sweeps stay within configured amplitudes");
        std::ostringstream key;
        key.precision(15);
        key << joints.tail<3>().transpose();
        unique.insert(key.str());
        for (int joint = 0; joint < 7; ++joint) {
            ok &= Check(joints[joint] >= kinematics.JointMinimums()[joint] + 2.0*kDegToRad - 1e-12
                            && joints[joint] <= kinematics.JointMaximums()[joint] - 2.0*kDegToRad + 1e-12,
                        "every planned sample respects joint margins");
        }
        if (i > 0) {
            const auto delta = (joints - path.joints_rad[i-1]).cwiseAbs();
            ok &= Check(delta.maxCoeff() <= (120.0/9.0)*kDegToRad + 1e-12,
                        "serpentine adjacency never jumps across a row/slab");
            ok &= Check((delta.tail<3>().array() > 1e-10).count() == 1,
                        "only one wrist coordinate changes between adjacent grid points");
        }
        ok &= Check(std::abs(path.gravity_directions_sensor[i].norm() - 1.0) < 1e-12,
                    "Sensor gravity direction is normalized");
    }
    ok &= Check(unique.size() == 1000, "all 1000 wrist poses are unique");
    for (int axis = 0; axis < 7; ++axis) {
        double lower = path.joints_rad.front()[axis], upper = lower;
        for (const auto& q : path.joints_rad) {
            lower = std::min(lower, q[axis]); upper = std::max(upper, q[axis]);
        }
        ok &= Check(upper - lower > 15.0*kDegToRad, "all seven joints participate in the path");
    }
    ok &= Check(std::abs(path.maximum_step_deg - 120.0/9.0) < 1e-10,
                "reported maximum step matches J7 grid interval");
    ok &= Check(path.maximum_gravity_separation_deg > 45.0,
                "FK Sensor gravity directions span multiple orientations");
    auto first_joints = path.joints_rad.front();
    Eigen::Matrix4d first_fk;
    kinematics.GetKinematics(first_fk, first_joints);
    const Eigen::Vector3d expected_direction =
        (first_fk.topLeftCorner<3, 3>() * sensor_rotation).transpose()
        * Eigen::Vector3d(0.0, 0.0, -1.0);
    ok &= Check((path.gravity_directions_sensor.front() - expected_direction).norm() < 1e-12,
                "gravity direction includes the measured ArmTip-Sensor installation rotation");

    options.pose_count = 27;
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                             options, path, &error)
                    && path.joints_rad.size() == 27,
                "other cubic grids, including odd level counts, are supported");
    for (std::size_t i = 1; i < path.joints_rad.size(); ++i) {
        ok &= Check(((path.joints_rad[i] - path.joints_rad[i-1]).tail<3>().cwiseAbs().array()
                     > 1e-10).count() == 1,
                    "odd-sized grids remain serpentine across slab boundaries");
    }
    options.pose_count = 9261;
    ok &= Check(!GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                              options, path, &error),
                "cubic counts above the CLI maximum of 8000 are rejected");
    options.pose_count = 1000;

    center[4] = kinematics.JointMaximums()[4] - kDegToRad;
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                             options, path, &error), error.c_str());
    ok &= Check(std::abs(path.wrist_max_deg[0]
                        - (kinematics.JointMaximums()[4]/kDegToRad - 2.0)) < 1e-10,
                "wrist grid clips against joint limits without losing the margin");
    options.wrist_half_ranges_deg[0] = 0.1;
    ok &= Check(!GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                              options, path, &error)
                    && error.find("amplitude") != std::string::npos,
                "an empty clipped wrist interval is rejected");
    options = ForceCalibrationPathOptions{};

    options.pose_count = 999;
    ok &= Check(!GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                              options, path, &error) && path.joints_rad.empty(),
                "non-cubic counts fail without a partial path");
    options.pose_count = 1000;
    options.wrist_half_ranges_deg.setConstant(0.01);
    options.arm_half_ranges_deg.setZero();
    center[4] = 0.0;
    ok &= Check(!GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                              options, path, &error)
                    && error.find("excitation") != std::string::npos,
                "nearly fixed gravity directions fail production fit conditioning");
    options = ForceCalibrationPathOptions{};
    center[0] = kinematics.JointMaximums()[0];
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                             options, path, &error),
                "moving arm joint range is clipped near a limit");
    for (const auto& q : path.joints_rad)
        ok &= Check(q[0] <= kinematics.JointMaximums()[0] - 2.0*kDegToRad + 1e-12,
                    "J1 sweep respects the clipped upper bound");
    center[0] = std::numeric_limits<double>::quiet_NaN();
    ok &= Check(!GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                              options, path, &error),
                "non-finite centers are rejected");
    // Workspace mode samples globally rather than oscillating around the center.
    center << 0, -20, 0, 60, 0, 40, 0;
    center *= kDegToRad;
    options = ForceCalibrationPathOptions{};
    options.workspace_coverage = true;
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                             options, path, &error), error.c_str());
    ok &= Check(path.joints_rad.size() == 1000 && path.tcp_positions_base_m.size() == 1000,
                "workspace has exactly 1000 joint and TCP samples");
    ok &= Check(path.workspace_candidate_count == 64000, "workspace explores 64000 global candidates");
    ok &= Check((path.workspace_max_m - path.workspace_min_m).minCoeff() > 1.0,
                "global TCP candidate coverage spans all three spatial axes");
    ok &= Check(path.candidate_cover_radius_m > 0 && path.candidate_cover_radius_m < .5,
                "selected samples bound distances to the finite candidate set");
    unique.clear();
    for (std::size_t i = 0; i < path.joints_rad.size(); ++i) {
        auto q = path.joints_rad[i];
        std::ostringstream key; key.precision(15); key << q.transpose();
        unique.insert(key.str());
        for (int axis = 0; axis < 7; ++axis)
            ok &= Check(q[axis] >= kinematics.JointMinimums()[axis] + 2*kDegToRad - 1e-12
                        && q[axis] <= kinematics.JointMaximums()[axis] - 2*kDegToRad + 1e-12,
                        "workspace joints respect model ranges");
        Eigen::Matrix4d fk;
        kinematics.GetKinematics(fk, q);
        const Eigen::Vector3d tcp = fk.topRightCorner<3, 1>()
            + fk.topLeftCorner<3, 3>() * options.tcp_armtip_m;
        ok &= Check((tcp - path.tcp_positions_base_m[i]).norm() < 1e-12,
                    "workspace TCP uses the configured ArmTip offset");
    }
    ok &= Check(unique.size() == 1000, "workspace poses are distinct");
    const auto original_targets = path.joints_rad;
    options.preserved_prefix_joints_rad.assign(original_targets.begin(), original_targets.begin() + 707);
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                             options, path, &error), error.c_str());
    ok &= Check(path.preserved_prefix_count == 707 && path.joints_rad.size() == 1000,
                "workspace supplements 293 targets after a preserved 707 prefix");
    for (std::size_t i = 0; i < 707; ++i)
        ok &= Check((path.joints_rad[i] - original_targets[i]).norm() == 0,
                    "preserved targets remain bit-for-bit unchanged");
    options.preserved_prefix_joints_rad.clear();
    options.pose_count = 31;
    options.tcp_armtip_m.setZero();
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics,
                                             options, path, &error) && path.joints_rad.size() == 31,
                "workspace supports non-cubic sample counts and custom TCP");
    options.preserved_prefix_joints_rad.clear();
    options.pose_count = 31;
    options.allowed_regions = {{{-.9,-.9,.2}, {-.05,.9,1.1}}, {{.05,-.9,.2}, {.9,.9,1.1}}};
    options.forbidden_spheres = {{{.2,0,.4}, .15}};
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics, options, path, &error), error.c_str());
    for (const auto& point : path.tcp_positions_base_m)
        ok &= Check(std::abs(point.x()) >= .05 && point.z() >= .2
                    && (point - Eigen::Vector3d(.2,0,.4)).norm() > .15,
                    "configured multi-region samples exclude forbidden neighborhoods");
    options = ForceCalibrationPathOptions{};
    options.workspace_coverage = true;
    options.minimum_body_z_m = .3;
    options.body_spheres = {{6, Eigen::Vector3d(0,0,.1), .1, "end_effector"}};
    center.setZero();
    ok &= Check(GenerateForceCalibrationPath(center, sensor_rotation, kinematics, options, path, &error), error.c_str());
    bool saw_detour = false;
    auto previous = center;
    for (std::size_t i = 0; i < path.joints_rad.size(); ++i) {
        const auto& q = path.joints_rad[i];
        ok &= Check(ForceCalibrationAbovePlane(q,q,kinematics,options), "component envelope stays above 30 cm");
        if (path.via_center[i]) {
            saw_detour = true;
            ok &= Check(ForceCalibrationAbovePlane(previous,center,kinematics,options)
                && ForceCalibrationAbovePlane(center,q,kinematics,options), "unsafe direct segment has checked center detour");
            ok &= Check(!ForceCalibrationAbovePlane(previous,q,kinematics,options),
                "endpoint-only checking would accept an unsafe or unproven segment");
        } else {
            ok &= Check(ForceCalibrationAbovePlane(previous,q,kinematics,options), "complete direct segment passes height bound");
        }
        previous = q;
    }
    ok &= Check(saw_detour, "global workspace exercises the checked detour branch");
    options.body_spheres.clear();
    ok &= Check(!ForceCalibrationAbovePlane(center,center,kinematics,options), "height constraint without envelope fails closed");
    options.body_spheres = {{7, Eigen::Vector3d::Zero(), .1, "invalid"}};
    ok &= Check(!ForceCalibrationAbovePlane(center,center,kinematics,options), "invalid body frame rejected");
    options.body_spheres = {{6, Eigen::Vector3d::Zero(), .1, "tip"}};
    options.minimum_body_z_m = .8;
    ok &= Check(!GenerateForceCalibrationPath(center,sensor_rotation,kinematics,options,path,&error),
                "unsafe initial envelope prevents planning");
    return ok ? 0 : 1;
}
