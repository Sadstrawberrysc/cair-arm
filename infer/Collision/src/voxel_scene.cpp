#include "collision/voxel_scene.hpp"
#include <cmath>
#include <octomap/OcTree.h>
#include <stdexcept>

namespace uspilot::collision {
double VoxelObservation::KnownFreeMargin(const Bounds &body, std::string* reason,
        const CellDistance& body_cell_distance, Eigen::Vector3d* failure_point, double query_margin_m) const {
    auto reject = [&](const char* why) { if (reason) *reason = why; return 0.; };
    if (reason) reason->clear();
    if (!tree || !std::isfinite(free_margin_m) || free_margin_m <= 0 || !max_query_cells ||
        !region.minimum.allFinite() || !region.maximum.allFinite() ||
        !(region.maximum.array() > region.minimum.array()).all())
        throw std::invalid_argument("Invalid voxel observation");
    if (!std::isfinite(query_margin_m) || (query_margin_m < 0 && query_margin_m != -1))
        throw std::invalid_argument("Invalid known-space query margin");
    const double margin = query_margin_m < 0 ? free_margin_m : query_margin_m;
    const Eigen::Vector3d lo = body.minimum.array() - margin;
    const Eigen::Vector3d hi = body.maximum.array() + margin;
    octomap::OcTreeKey first, last;
    if (!tree->coordToKeyChecked(octomap::point3d(lo.x(), lo.y(), lo.z()), first) ||
        !tree->coordToKeyChecked(octomap::point3d(hi.x(), hi.y(), hi.z()), last))
        return reject("octree_address_range");
    std::uint64_t cells = 1;
    for (int i = 0; i < 3; ++i) {
        const auto count = std::uint64_t(last[i]) - first[i] + 1;
        if (count > max_query_cells || cells > max_query_cells / count)
            return reject("known_space_query_budget");
        cells *= count;
    }
    for (unsigned x = first[0]; x <= last[0]; ++x)
        for (unsigned y = first[1]; y <= last[1]; ++y)
            for (unsigned z = first[2]; z <= last[2]; ++z) {
                const auto p = tree->keyToCoord(octomap::OcTreeKey(x, y, z));
                const Eigen::Vector3d center(p.x(), p.y(), p.z());
                const double half = tree->getResolution() / 2;
                const auto *node = tree->search(octomap::OcTreeKey(x, y, z));
                const bool occupied = node && tree->isNodeOccupied(node);
                const bool within_region =
                    (center.array() - half > region.minimum.array()).all() &&
                    (center.array() + half < region.maximum.array()).all();
                // Fast paths: these cells are covered regardless of whether
                // the candidate geometry actually reaches them.
                if (node && !occupied && within_region) continue;
                const double radius = std::sqrt(3.) * half + 1e-6;
                if (!occupied && captured_robot_contains_ball && captured_robot_contains_ball(center, radius)) continue;
                if (body_cell_distance) {
                    const double distance = body_cell_distance(center, half);
                    if (!std::isfinite(distance) || distance < 0)
                        throw std::runtime_error("Invalid body-to-voxel distance");
                    // AABB corners outside the real geometry and its margin
                    // impose no visibility requirement on this candidate.
                    if (distance > margin) continue;
                }
                if (failure_point) *failure_point = center;
                // Actual observed occupancy always wins over model ownership.
                if (occupied) return reject("occupied_cell");
                // Require the whole cell inside the observation ROI. Robot
                // ownership can cover its own interior outside the ROI only.
                if (!within_region)
                    return reject("outside_observation_region");
                if (!node) return reject("unobserved_cell");
            }
    return free_margin_m;
}
} // namespace uspilot::collision
