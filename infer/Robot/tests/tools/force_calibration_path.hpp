#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

class RMKinematics;

struct ForceCalibrationWorkspaceBox {
    Eigen::Vector3d minimum_m, maximum_m;
};

struct ForceCalibrationWorkspaceSphere {
    Eigen::Vector3d center_m;
    double radius_m = 0;
};

// Conservative mesh envelopes in the corresponding MDH frame (J1..J7).
struct ForceCalibrationBodySphere {
    int joint_index = 0;
    Eigen::Vector3d center_m = Eigen::Vector3d::Zero();
    double radius_m = 0;
    std::string name;
};

struct ForceCalibrationPathOptions {
    // Workspace selection uses arbitrary counts; local mode uses a cubic wrist grid.
    std::size_t pose_count = 1000;
    Eigen::Matrix<double, 4, 1> arm_half_ranges_deg = Eigen::Matrix<double, 4, 1>::Constant(20.0);
    Eigen::Vector3d wrist_half_ranges_deg{45.0, 45.0, 60.0};
    double joint_limit_margin_deg = 2.0;
    bool workspace_coverage = false;
    Eigen::Vector3d tcp_armtip_m{0.0, 0.0, 0.2005};
    std::vector<Eigen::Matrix<double, 7, 1>> preserved_prefix_joints_rad;
    std::vector<ForceCalibrationWorkspaceBox> allowed_regions;
    std::vector<ForceCalibrationWorkspaceBox> forbidden_boxes;
    std::vector<ForceCalibrationWorkspaceSphere> forbidden_spheres;
    double minimum_body_z_m = -std::numeric_limits<double>::infinity();
    std::vector<ForceCalibrationBodySphere> body_spheres;
};

struct ForceCalibrationPath {
    std::vector<Eigen::Matrix<double, 7, 1>> joints_rad;
    std::vector<Eigen::Vector3d> gravity_directions_sensor;
    Eigen::Vector3d wrist_min_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d wrist_max_deg = Eigen::Vector3d::Zero();
    // Raw joint-leg span; the CLI subdivides actual MoveJ legs to at most 10 deg.
    double maximum_step_deg = 0.0;
    double maximum_gravity_separation_deg = 0.0;
    std::vector<Eigen::Vector3d> tcp_positions_base_m;
    Eigen::Vector3d workspace_min_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d workspace_max_m = Eigen::Vector3d::Zero();
    std::size_t workspace_candidate_count = 0;
    double candidate_cover_radius_m = 0.0;
    std::size_t preserved_prefix_count = 0;
    std::vector<bool> via_center;
};

// Conservative bound for the entire joint-linear segment, including endpoints.
bool ForceCalibrationAbovePlane(const Eigen::Matrix<double, 7, 1>& from,
    const Eigen::Matrix<double, 7, 1>& to, RMKinematics& kinematics,
    const ForceCalibrationPathOptions& options);

// Workspace mode selects TCP-space samples; local mode uses seven-joint sweeps.
// FK supplies actual Sensor gravity directions; a synthetic noiseless fit
// exercises the production calibration's force/torque conditioning criteria.
// No robot connection, joint command, or environment collision check occurs.
bool GenerateForceCalibrationPath(
    const Eigen::Matrix<double, 7, 1>& center_joints_rad,
    const Eigen::Matrix3d& rotation_armtip_from_sensor,
    RMKinematics& kinematics,
    const ForceCalibrationPathOptions& options,
    ForceCalibrationPath& path,
    std::string* error = nullptr);
