#pragma once
#include "collision/collision_checker.hpp"

namespace uspilot::collision {
struct PlannerOptions {
    double timeout_s = 2.0;
    double range = 0.3;  // Euclidean joint-space distance; RM75 coordinates are rad.
    std::size_t max_queries = 20000;  // Shared state/segment query budget.
    std::size_t shortcut_attempts = 50;
    std::uint32_t seed = 42;
    bool skip_start_precheck = false;  // Does not bypass OMPL or segment validity.
    CheckOptions collision;
};
struct PlanResult {
    bool unknown_space_allowed = false;
    std::string status = "invalid_input";
    std::string detail;
    std::string model_id, scene_id, calibration_id;
    std::int64_t captured_at_ns = 0, checked_at_ns = 0;
    std::uint32_t seed = 0;
    StateCheck start_check, goal_check;
    StateCheck validation_failure;
    SegmentCheck last_segment_failure;
    std::size_t state_queries = 0, segment_queries = 0;
    std::size_t segment_state_checks = 0;
    double elapsed_ms = 0;
    std::vector<JointPositions> waypoints;
    bool Exact() const { return status == "exact_solution"; }
};

// Offline snapshot planning only. Caller-provided now_ns remains in the scene's
// clock domain; a separate steady-clock budget bounds computation. No device I/O.
// A fresh model/checker pair and immutable scene must describe the same geometry.
PlanResult PlanJointPath(const RobotModel& model, const CollisionChecker& checker,
    const JointPositions& start, const JointPositions& goal, const CollisionScene& scene,
    std::int64_t now_ns, const PlannerOptions& options = {},
    const std::function<bool()>& cancelled = {});
} // namespace uspilot::collision
