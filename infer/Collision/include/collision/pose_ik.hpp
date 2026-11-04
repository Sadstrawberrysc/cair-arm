#pragma once
#include "collision/joint_planner.hpp"
#include <limits>

namespace uspilot::collision {
struct PoseIkOptions {
    std::size_t attempts = 8, iterations = 80;
    double timeout_s = 0.1;
    double position_tolerance_m = 0.001, rotation_tolerance_rad = 0.01;
    std::uint32_t seed = 42;
    std::vector<double> yaw_offsets_rad{0}; // Rotate about outward normal.
    std::vector<Eigen::Vector2d> tilt_offsets_rad{Eigen::Vector2d::Zero()};
    double max_normal_deviation_rad = 0.17453292519943295; // 10 degrees, endpoint only.
    std::size_t max_candidates_per_pose = 4;
};
struct PoseIkResult {
    std::vector<JointPositions> candidates;
    std::vector<double> candidate_yaw_offsets_rad, candidate_normal_deviation_rad;
    std::vector<Eigen::Isometry3d> candidate_targets;
    std::size_t attempts = 0, iterations = 0;
    double best_position_error_m = std::numeric_limits<double>::infinity();
    double best_rotation_error_rad = std::numeric_limits<double>::infinity();
};
// FK is taken from the same URDF as collision checking. TCP is an explicit
// calibrated transform from tip_link, not the STL/CAD origin of the probe.
PoseIkResult SolvePoseIk(const RobotModel& model, const JointPositions& seed,
    const std::string& tip_link, const Eigen::Isometry3d& tip_from_tcp,
    const Eigen::Isometry3d& base_from_tcp, const PoseIkOptions& options = {},
    const std::function<bool()>& cancelled = {});

struct CandidateSearch {
    std::size_t candidate_index = 0;
    std::string status = "not_searched", detail;
    double budget_s = 0, elapsed_ms = 0;
    double yaw_offset_rad = 0, normal_deviation_rad = 0;
    std::size_t waypoint_count = 0;
    std::optional<double> joint_path_length_rad;
};

struct PosePlanResult {
    PlanResult plan;
    PoseIkResult ik;
    std::vector<StateCheck> candidate_checks;
    std::vector<CandidateSearch> candidate_searches;
    std::optional<std::size_t> preview_candidate_index;
};
PosePlanResult PlanPosePath(const RobotModel& model, const CollisionChecker& checker,
    const JointPositions& start, const std::string& tip_link, const Eigen::Isometry3d& tip_from_tcp,
    const Eigen::Isometry3d& base_from_tcp, const CollisionScene& scene, std::int64_t now_ns,
    const PlannerOptions& planning = {}, const PoseIkOptions& ik = {},
    const std::function<bool()>& cancelled = {});
} // namespace uspilot::collision
