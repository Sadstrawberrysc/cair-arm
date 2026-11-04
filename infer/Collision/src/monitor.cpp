#include "collision/monitor.hpp"
#include <cmath>
#include <fstream>
#include <json.hpp>
#include <map>
#include <octomap/OcTree.h>
#include <set>
#include <stdexcept>

namespace uspilot::collision {
namespace {
void Require(bool valid, const std::string &reason) {
    if (!valid)
        throw std::invalid_argument(reason);
}
using Key = std::array<unsigned, 3>;
Key Array(const octomap::OcTreeKey &key) { return {key[0], key[1], key[2]}; }
octomap::OcTreeKey TreeKey(const Key &key) { return octomap::OcTreeKey(key[0], key[1], key[2]); }
bool InRegion(const Eigen::Vector3d &point, const Bounds &region) {
    return (point.array() >= region.minimum.array()).all() && (point.array() <= region.maximum.array()).all();
}
std::vector<bool> FindOutliers(const std::vector<Eigen::Vector3d>& points,
                              const std::vector<bool>& excluded_mask, double radius,
                              std::size_t min_neighbors) {
    std::vector<bool> outliers(points.size(), false);
    if (!min_neighbors) return outliers;
    using Cell = std::array<std::int64_t, 3>;
    std::map<Cell, std::vector<std::size_t>> cells;
    std::vector<Cell> keys(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (excluded_mask[i]) continue;
        for (int axis = 0; axis < 3; ++axis) {
            const double value = std::floor(points[i][axis] / radius);
            Require(std::isfinite(value) && std::abs(value) < 9e18, "Point outside density grid range");
            keys[i][axis] = static_cast<std::int64_t>(value);
        }
        cells[keys[i]].push_back(i);
    }
    const double squared_radius = radius * radius;
    std::size_t queries = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (excluded_mask[i]) continue;
        std::size_t neighbors = 0;
        for (int x = -1; x <= 1 && neighbors < min_neighbors; ++x)
            for (int y = -1; y <= 1 && neighbors < min_neighbors; ++y)
                for (int z = -1; z <= 1 && neighbors < min_neighbors; ++z) {
                    const auto cell = cells.find({keys[i][0] + x, keys[i][1] + y, keys[i][2] + z});
                    if (cell == cells.end()) continue;
                    for (const auto j : cell->second) {
                        if (j == i) continue;
                        Require(++queries <= 2000000, "Point neighbor query budget exceeded");
                        if ((points[i] - points[j]).squaredNorm() <= squared_radius && ++neighbors >= min_neighbors)
                            break;
                    }
                }
        outliers[i] = neighbors < min_neighbors;
    }
    return outliers;
}
} // namespace

CameraCalibration LoadCameraCalibration(const std::filesystem::path &file) {
    Require(std::filesystem::file_size(file) <= 1024 * 1024, "Calibration exceeds size budget");
    std::ifstream input(file);
    const auto j = nlohmann::json::parse(input);
    Require(j.at("transform") == "d455_color_optical_to_rm75_base" && j.at("translation_unit") == "m" &&
                j.at("user_confirmed") == true,
            "Calibration direction, units or confirmation mismatch");
    CameraCalibration result;
    result.serial = j.at("camera").at("serial").get<std::string>();
    Require(!result.serial.empty(), "Calibration has no camera serial");
    const auto &matrix = j.at("T_base_camera");
    Require(matrix.is_array() && matrix.size() == 4, "Calibration must be 4x4");
    for (int r = 0; r < 4; ++r) {
        Require(matrix[r].is_array() && matrix[r].size() == 4, "Calibration must be 4x4");
        for (int c = 0; c < 4; ++c) {
            Require(matrix[r][c].is_number(), "Non-numeric calibration");
            result.base_from_color.matrix()(r, c) = matrix[r][c].get<double>();
        }
    }
    const auto &pose = result.base_from_color;
    Require(pose.matrix().allFinite() &&
                (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9 &&
                (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
                std::abs(pose.linear().determinant() - 1) < 1e-6,
            "Invalid calibration rigid transform");
    result.sha256 = FileSha256(file);
    return result;
}

MonitorResult ProcessMonitorFrame(const CollisionChecker &checker, const CameraCalibration &calibration,
                                  const MonitorFrame &frame, std::int64_t now,
                                  const MonitorOptions &options) {
    MonitorResult result;
    result.check.model_id = checker.ModelId();
    result.check.calibration_id = calibration.sha256;
    result.check.captured_at_ns = frame.captured_at_ns;
    result.check.scene_id = "depth/" + frame.camera_serial + "/" + std::to_string(frame.sequence);
    try {
        Require(std::isfinite(options.voxel_m) && options.voxel_m >= .005 && options.voxel_m <= .2 &&
                    options.region.minimum.allFinite() && options.region.maximum.allFinite() &&
                    (options.region.maximum.array() > options.region.minimum.array()).all() &&
                    std::isfinite(options.min_depth_m) && std::isfinite(options.max_depth_m) &&
                    options.min_depth_m > 0 && options.max_depth_m > options.min_depth_m &&
                    options.max_depth_m <= 20 && options.endpoint_padding_voxels <= 3 && options.max_points &&
                    options.max_voxels && options.max_frame_age_ns >= 0 &&
                    options.max_feedback_offset_ns >= 0 &&
                    std::isfinite(options.outlier_radius_m) && options.outlier_radius_m >= .005 &&
                    options.outlier_radius_m <= .2 && options.outlier_min_neighbors <= 64,
                "Invalid monitor options");
        Require(frame.camera_serial == calibration.serial && !calibration.sha256.empty(),
                "Camera/calibration identity mismatch");
        Require(frame.capture_time_trusted, "capture_time_untrusted");
        Require(frame.robot_valid, "robot_feedback_invalid");
        Require(now >= 0 && frame.captured_at_ns >= 0 && frame.received_at_ns >= frame.captured_at_ns &&
                    now >= frame.received_at_ns && frame.robot_at_ns >= 0,
                "Invalid/future frame times");
        if (now - frame.captured_at_ns > options.max_frame_age_ns) {
            result.check.status = CheckStatus::kStaleScene;
            result.check.detail = "camera_frame_expired";
            return result;
        }
        if (std::abs(frame.robot_at_ns - frame.captured_at_ns) > options.max_feedback_offset_ns ||
            now < frame.robot_at_ns || now - frame.robot_at_ns > options.max_frame_age_ns) {
            result.check.status = CheckStatus::kStaleScene;
            result.check.detail = "robot_feedback_not_time_matched";
            return result;
        }
        Require(frame.points_color_m.size() <= options.max_points, "Point budget exceeded");
        Require(frame.colors.empty() || frame.colors.size() == frame.points_color_m.size(),
                "Point/color size mismatch");
        Require(frame.ray_origin_color_m.allFinite(), "Invalid depth-ray origin");
        std::vector<Eigen::Vector3d> valid_points;
        for (const auto &p : frame.points_color_m) {
            if (!p.allFinite() || p.z() < options.min_depth_m || p.z() > options.max_depth_m) {
                ++result.invalid_points;
                continue;
            }
            valid_points.push_back(calibration.base_from_color * p);
        }
        result.points_base_m = std::move(valid_points);
        result.robot_mask =
            checker.PointsInsideRobot(frame.joints, result.points_base_m, options.mask_padding_m);
        const Eigen::Vector3d origin = calibration.base_from_color * frame.ray_origin_color_m;
        result.occluded_mask.assign(result.points_base_m.size(), false);
        if (options.robot_occlusion_filter)
            result.occluded_mask = checker.PointsOccludedByRobot(frame.joints, origin,
                                                               result.points_base_m, options.mask_padding_m);
        auto excluded = result.robot_mask;
        for (std::size_t i = 0; i < excluded.size(); ++i) excluded[i] = excluded[i] || result.occluded_mask[i];
        result.outlier_mask = FindOutliers(result.points_base_m, excluded,
                                          options.outlier_radius_m, options.outlier_min_neighbors);
        auto tree = std::make_shared<octomap::OcTree>(options.voxel_m);
        std::set<Key> free, occupied, protected_keys;
        const octomap::point3d ray_origin(origin.x(), origin.y(), origin.z());
        octomap::OcTreeKey origin_key(0, 0, 0);
        Require(tree->coordToKeyChecked(ray_origin, origin_key), "Depth origin outside octree address range");
        auto within = [&](const octomap::OcTreeKey &key) {
            const auto p = tree->keyToCoord(key);
            return InRegion({p.x(), p.y(), p.z()}, options.region);
        };
        for (std::size_t i = 0; i < result.points_base_m.size(); ++i) {
            if (result.robot_mask[i]) ++result.masked_points;
            if (result.occluded_mask[i]) { ++result.occluded_points; continue; }
            const auto &p = result.points_base_m[i];
            const octomap::point3d endpoint(p.x(), p.y(), p.z());
            octomap::OcTreeKey endpoint_key(0, 0, 0);
            Require(tree->coordToKeyChecked(endpoint, endpoint_key), "Endpoint outside octree address range");
            if (!result.outlier_mask[i]) {
                octomap::KeyRay ray;
                Require(tree->computeRayKeys(ray_origin, endpoint, ray), "Ray construction failed");
                for (const auto &key : ray)
                    if (within(key))
                        free.insert(Array(key));
            } else {
                ++result.outlier_points;
            }
            const int padding = static_cast<int>(options.endpoint_padding_voxels);
            for (int x = -padding; x <= padding; ++x)
                for (int y = -padding; y <= padding; ++y)
                    for (int z = -padding; z <= padding; ++z) {
                        const int a = int(endpoint_key[0]) + x, b = int(endpoint_key[1]) + y,
                                  c = int(endpoint_key[2]) + z;
                        if (a < 0 || b < 0 || c < 0 || a > 65535 || b > 65535 || c > 65535)
                            continue;
                        const octomap::OcTreeKey key(a, b, c);
                        if (!within(key))
                            continue;
                        if (result.robot_mask[i] || result.outlier_mask[i])
                            protected_keys.insert(Array(key));
                        else
                            occupied.insert(Array(key));
                    }
            Require(free.size() + occupied.size() + protected_keys.size() <= options.max_voxels,
                    "Voxel budget exceeded");
        }
        // Reject voxel footprints whose camera rays can intersect the robot.
        // Inflate by the voxel's bounding-sphere radius so grazing cells remain unknown.
        std::set<Key> hidden;
        if (options.robot_occlusion_filter) {
            std::set<Key> candidates = free;
            candidates.insert(occupied.begin(), occupied.end());
            std::vector<Eigen::Vector3d> centers;
            centers.reserve(candidates.size());
            for (const auto& key : candidates) {
                const auto center = tree->keyToCoord(TreeKey(key));
                centers.emplace_back(center.x(), center.y(), center.z());
            }
            const auto invisible = checker.PointsOccludedByRobot(frame.joints, origin, centers,
                options.mask_padding_m + std::sqrt(3.) * options.voxel_m / 2 + 1e-6);
            std::size_t i = 0;
            for (const auto& key : candidates) if (invisible[i++]) hidden.insert(key);
            result.occluded_voxels = hidden.size();
        }
        // Occupied beats free. Robot/outlier endpoints protect unknown cells
        // without erasing real occupancy. Rejected depth rays contribute no free cells.
        for (const auto &key : free)
            if (!occupied.count(key) && !protected_keys.count(key) && !hidden.count(key))
                tree->updateNode(TreeKey(key), false, true);
        for (const auto &key : occupied)
            if (!hidden.count(key)) {
                tree->updateNode(TreeKey(key), true, true);
                ++result.occupied_voxels;
            }
        tree->updateInnerOccupancy();
        auto observation = std::make_shared<VoxelObservation>();
        observation->tree = tree;
        observation->region = options.region;
        observation->robot_model_id = checker.ModelId();
        observation->captured_robot_contains_ball = checker.CaptureRobotVolume(frame.joints);
        observation->camera_origin_base_m = origin;
        observation->captured_joints = frame.joints;
        observation->robot_occlusion_filter = options.robot_occlusion_filter;
        observation->mask_padding_m = options.mask_padding_m;
        observation->cell_evidence = [tree, hidden = std::move(hidden),
            protected_keys = std::move(protected_keys)](const Eigen::Vector3d& point) -> std::string {
            octomap::OcTreeKey key;
            if (!point.allFinite() || !tree->coordToKeyChecked(
                    octomap::point3d(point.x(), point.y(), point.z()), key)) return "outside_octree";
            const auto* node = tree->search(key);
            if (node && tree->isNodeOccupied(node)) return "observed_occupied";
            if (hidden.count(Array(key))) return "robot_occlusion_withheld";
            if (protected_keys.count(Array(key))) return "robot_or_outlier_endpoint_protected";
            if (!node) return "no_retained_free_ray";
            return tree->isNodeOccupied(node) ? "observed_occupied" : "observed_free";
        };
        observation->free_margin_m =
            std::max(.03, options.collision.clearance_m + options.collision.numerical_guard_m + .005);
        result.scene.id = result.check.scene_id;
        result.scene.calibration_id = calibration.sha256;
        result.scene.captured_at_ns = frame.captured_at_ns;
        result.scene.observation = observation;
        result.check = checker.CheckState(frame.joints, result.scene, now, options.collision);
        result.input_valid = true;
    } catch (const std::exception &e) {
        result.check.status = CheckStatus::kInvalidInput;
        result.check.detail = e.what();
    }
    return result;
}
} // namespace uspilot::collision
