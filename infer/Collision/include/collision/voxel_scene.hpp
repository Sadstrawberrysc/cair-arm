#pragma once
#include "collision/robot_model.hpp"
#include <cstdint>
#include <memory>
#include <functional>
#include <optional>

namespace octomap {
class OcTree;
}
namespace uspilot::collision {
// One-frame ray observation. Missing cells and robot-filtered endpoints stay
// unknown; this is not the fully surveyed manual CollisionScene workspace.
struct VoxelObservation {
    std::shared_ptr<const octomap::OcTree> tree;
    Bounds region;
    double free_margin_m = 0.03;
    std::size_t max_query_cells = 250000;
    // Immutable capture-pose solid ownership, NOT camera free space. A ball
    // must fit entirely inside one unpadded robot solid. Never includes shadows.
    std::string robot_model_id;
    std::function<bool(const Eigen::Vector3d&, double)> captured_robot_contains_ball;
    std::optional<Eigen::Vector3d> camera_origin_base_m;
    JointPositions captured_joints;
    bool robot_occlusion_filter = false;
    double mask_padding_m = 0;
    std::function<std::string(const Eigen::Vector3d&)> cell_evidence;
    // Optional conservative lower bound on distance from the actual body to a
    // voxel cube (center, half edge). AABB only enumerates candidates.
    using CellDistance = std::function<double(const Eigen::Vector3d&, double)>;
    double KnownFreeMargin(const Bounds &body_bounds, std::string* reason = nullptr,
        const CellDistance& body_cell_distance = {}, Eigen::Vector3d* failure_point = nullptr,
        double query_margin_m = -1) const;
};
} // namespace uspilot::collision
