#pragma once

#include "collision/robot_model.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

namespace uspilot::collision {
struct VoxelObservation;

// Explicit, reviewed link pairs only. Never auto-exclude neighbours or tools.
struct SelfCollisionExclusion {
    std::string first_link;
    std::string second_link;
    std::string reason;
};

struct Obstacle {
    std::string id;
    Geometry geometry;  // Primitive solid; link_from_geometry is Base-from-shape.
};

// One immutable scene per query: either a fully surveyed manual known_space
// with primitive solids, OR a ray-built observation. Missing observation cells
// are unknown; a manual box cannot override camera unknowns.
struct CollisionScene {
    std::string id;
    std::string calibration_id;
    std::string frame = "base_link";
    std::int64_t captured_at_ns = 0;
    std::optional<Bounds> known_space;
    std::shared_ptr<const VoxelObservation> observation;
    std::vector<Obstacle> obstacles;
};

struct CheckOptions {
    bool allow_unknown_space = false; // Planning policy only; does not erase occupancy.
    double clearance_m = 0.005;  // Example setting, not a measured safety margin.
    double numerical_guard_m = 1e-6;
    std::int64_t max_scene_age_ns = 500000000;
    std::size_t max_state_checks = 2048;
    std::size_t max_subdivision_depth = 20;
};

enum class CheckStatus {
    kValid, kCollision, kClearanceViolation, kUnknownSpace, kStaleScene,
    kInvalidInput, kQueryFailure, kBudgetExceeded, kCancelled
};
const char* CheckStatusName(CheckStatus status);

struct PairViolation {
    std::string first_body;
    std::string second_body;
    bool self_collision = false;
    bool intersecting = false;
    double distance_m = 0;  // Nonnegative separation; zero for intersection.
    std::optional<Eigen::Vector3d> first_point_base_m, second_point_base_m;
};

struct UnknownSpaceDiagnostic {
    std::string body, reason;
    std::optional<Eigen::Vector3d> cell_center_base_m;
    std::optional<double> body_to_cell_distance_lower_bound_m;
    double query_margin_m = 0;
    std::string first_cell_scope = "not_located";
    std::string cell_evidence = "unavailable";
    // Counterfactual only: never changes this StateCheck's validity.
    std::string geometry_only_status = "not_checked";
    std::string geometry_only_reason;
    std::string geometry_only_cell_evidence = "unavailable";
    std::optional<Eigen::Vector3d> geometry_only_failure_point_base_m;
    std::optional<bool> center_inside_captured_robot;
    std::optional<bool> capture_robot_occludes_cell;
    std::optional<bool> geometry_only_center_inside_captured_robot;
    std::optional<bool> geometry_only_capture_robot_occludes_cell;
};

struct PairClearance {
    std::size_t first_body = 0;
    std::optional<std::size_t> second_body; // Absent for a static environment.
    double distance_m = 0;
};

struct StateCheck {
    CheckStatus status = CheckStatus::kInvalidInput;
    std::string detail;
    std::string model_id;
    std::string scene_id;
    std::string calibration_id;
    std::int64_t captured_at_ns = 0;
    double minimum_distance_m = 0;
    double minimum_environment_distance_m = 0;
    double minimum_self_distance_m = 0;
    double workspace_clearance_m = 0;
    std::size_t pair_queries = 0;
    std::vector<PairViolation> violations;
    std::vector<PairClearance> pair_clearances; // Internal segment certificates; not verbose JSON.
    std::vector<std::string> unknown_space_bodies;
    // Same order as unknown_space_bodies; NaN when no individual cell was
    // reached (address/budget failure).
    std::vector<Eigen::Vector3d> unknown_space_points_base_m;
    std::vector<UnknownSpaceDiagnostic> unknown_space_diagnostics;
    bool Valid() const { return status == CheckStatus::kValid; }
};

struct SegmentCheck {
    CheckStatus status = CheckStatus::kInvalidInput;
    std::string detail;
    std::string model_id;
    std::string scene_id;
    std::string calibration_id;
    std::int64_t captured_at_ns = 0;
    std::size_t state_checks = 0;
    std::size_t certified_intervals = 0;
    std::optional<double> failure_fraction;
    double certified_clearance_lower_bound_m = 0;
    StateCheck failure;
    bool Valid() const { return status == CheckStatus::kValid; }
};

struct CollisionBodyInfo {
    std::string id;
    std::string link;
    std::string representation;
    std::string mesh_sha256;
    std::size_t hull_vertices = 0;
    std::size_t hull_faces = 0;
    double hull_expansion_m = 0;
};

// Actual FCL geometry at a supplied pose, for read-only diagnostic rendering.
struct CollisionBodyPreview {
    std::string id;
    Eigen::Isometry3d base_from_body = Eigen::Isometry3d::Identity();
    Geometry geometry;
    TriangleMesh mesh; // Filled convex hull surface for mesh bodies.
};

// Owns immutable robot geometry. Checks use local FCL objects; no device I/O.
// STL is converted to a filled conservative convex hull, not a hollow BVH mesh.
class CollisionChecker {
public:
    CollisionChecker(RobotModel model, const PackageRoots& packages = {},
                     const MeshDirectories& directories = {}, const MeshLoadOptions& mesh_options = {},
                     std::vector<SelfCollisionExclusion> exclusions = {});
    ~CollisionChecker();
    CollisionChecker(CollisionChecker&&) noexcept;
    CollisionChecker& operator=(CollisionChecker&&) noexcept;
    CollisionChecker(const CollisionChecker&) = delete;
    CollisionChecker& operator=(const CollisionChecker&) = delete;
    const std::string& ModelId() const;
    const std::string& UrdfSha256() const;
    const std::vector<CollisionBodyInfo>& Bodies() const;
    const std::vector<SelfCollisionExclusion>& Exclusions() const;
    std::vector<CollisionBodyPreview> PreviewBodies(const JointPositions& q) const;
    // Owns copies of geometry/poses; remains fixed at capture q and valid after
    // this checker is destroyed. No mask padding or blind-space extrapolation.
    std::function<bool(const Eigen::Vector3d&, double)> CaptureRobotVolume(
        const JointPositions& q) const;
    // Conservative mask for perception only. Masked endpoints MUST become
    // unknown, never free. A convex hull may also contain a nearby human point.
    std::vector<bool> PointsInsideRobot(const JointPositions& q,
        const std::vector<Eigen::Vector3d>& points_base, double padding_m = 0) const;
    // Finite camera-to-point segments. True means the padded robot intersects
    // the segment before/at its endpoint; never treat that observation as free.
    std::vector<bool> PointsOccludedByRobot(const JointPositions& q,
        const Eigen::Vector3d& camera_origin_base,
        const std::vector<Eigen::Vector3d>& points_base, double padding_m = 0) const;
    StateCheck CheckState(const JointPositions& q, const CollisionScene& scene,
                          std::int64_t now_ns, const CheckOptions& options = {}) const;
    // Linear interpolation in joint coordinates, without shortest-angle wrap.
    // Certifies intervals by conservative serial-chain displacement bounds;
    // an exhausted subdivision budget is rejected, never accepted as sampling.
    SegmentCheck CheckSegment(const JointPositions& start, const JointPositions& goal,
                              const CollisionScene& scene, std::int64_t now_ns,
                              const CheckOptions& options = {},
                              const std::function<bool()>& cancelled = {}) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace uspilot::collision
