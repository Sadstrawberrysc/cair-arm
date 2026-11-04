#pragma once
#include "collision/collision_checker.hpp"
#include "collision/voxel_scene.hpp"

namespace uspilot::collision {
struct CameraCalibration {
    Eigen::Isometry3d base_from_color = Eigen::Isometry3d::Identity();
    std::string serial;
    std::string sha256;
};
CameraCalibration LoadCameraCalibration(const std::filesystem::path &file);

struct MonitorFrame {
    std::uint64_t sequence = 0;
    std::string camera_serial;
    std::int64_t captured_at_ns = 0;
    std::int64_t received_at_ns = 0;
    std::int64_t robot_at_ns = 0;
    bool capture_time_trusted = false;
    bool robot_valid = false;
    JointPositions joints;
    // Points and depth-ray origin are in the COLOR optical frame, even when
    // the source stream is depth. Live source applies depth->color extrinsics.
    Eigen::Vector3d ray_origin_color_m = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> points_color_m;
    std::vector<std::array<unsigned char, 3>> colors;
};
struct MonitorOptions {
    Bounds region{{-1, -1, -.2}, {1, 1, 1.5}};
    double voxel_m = .03;
    double min_depth_m = .15;
    double max_depth_m = 3;
    double mask_padding_m = .005;
    bool robot_occlusion_filter = false;
    double outlier_radius_m = .03;
    // Other non-robot points inside radius; zero disables density filtering.
    std::size_t outlier_min_neighbors = 0;
    std::size_t endpoint_padding_voxels = 1;
    std::size_t max_points = 30000;
    std::size_t max_voxels = 300000;
    std::int64_t max_feedback_offset_ns = 100000000;
    std::int64_t max_frame_age_ns = 500000000;
    CheckOptions collision;
};
struct MonitorResult {
    StateCheck check;
    CollisionScene scene;
    std::vector<Eigen::Vector3d> points_base_m;
    std::vector<bool> robot_mask;
    std::vector<bool> occluded_mask; // Camera rays intercepted by robot geometry.
    std::vector<bool> outlier_mask; // Same indexing as points_base_m; never free evidence.
    std::size_t invalid_points = 0;
    std::size_t masked_points = 0;
    std::size_t outlier_points = 0;
    std::size_t occluded_points = 0;
    std::size_t occluded_voxels = 0;
    std::size_t occupied_voxels = 0;
    bool input_valid = false;
};
MonitorResult ProcessMonitorFrame(const CollisionChecker &checker, const CameraCalibration &calibration,
                                  const MonitorFrame &frame, std::int64_t now_ns,
                                  const MonitorOptions &options = {});
} // namespace uspilot::collision
