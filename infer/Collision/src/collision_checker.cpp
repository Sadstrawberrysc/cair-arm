#include "collision/collision_checker.hpp"
#include "collision/voxel_scene.hpp"

#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/convex.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/geometry/octree/octree.h>
#include <octomap/OcTree.h>
#include <libqhull_r/qhull_ra.h>
#include <openssl/evp.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace uspilot::collision {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kHullPadding = 1e-6;
using FclShape = std::shared_ptr<fcl::CollisionGeometryd>;
struct PlacedShape {
    FclShape shape;
    Eigen::Isometry3d transform;
};

std::string Hash(const std::string& text) {
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int length = 0;
    if (EVP_Digest(text.data(), text.size(), digest, &length, EVP_sha256(), nullptr) != 1)
        throw std::runtime_error("Cannot hash collision model");
    std::ostringstream out;
    for (unsigned int i = 0; i < length; ++i) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(digest[i]);
    return out.str();
}

void Require(bool value, const std::string& reason) {
    if (!value) throw std::invalid_argument(reason);
}

void ValidatePose(const Eigen::Isometry3d& pose) {
    Require(pose.matrix().allFinite()
        && (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-9
        && std::abs(pose.linear().determinant() - 1) < 1e-9
        && (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-12,
        "Geometry pose must be a finite rigid transform");
}

void ValidateBounds(const Bounds& b) {
    Require(b.minimum.allFinite() && b.maximum.allFinite()
        && (b.maximum.array() > b.minimum.array()).all(), "Invalid known-space bounds");
}

FclShape Primitive(const Geometry& g) {
    ValidatePose(g.link_from_geometry);
    switch (g.shape) {
    case Shape::kBox:
        Require(g.box_size.allFinite() && (g.box_size.array() > 0).all(), "Invalid box dimensions");
        return std::make_shared<fcl::Boxd>(g.box_size);
    case Shape::kSphere:
        Require(std::isfinite(g.radius) && g.radius > 0, "Invalid sphere radius");
        return std::make_shared<fcl::Sphered>(g.radius);
    case Shape::kCylinder:
        Require(std::isfinite(g.radius) && g.radius > 0 && std::isfinite(g.length) && g.length > 0,
                "Invalid cylinder dimensions");
        return std::make_shared<fcl::Cylinderd>(g.radius, g.length);
    default: throw std::invalid_argument("Scene supports primitive solids only; mesh requires robot hull conversion");
    }
}

struct QhullContext {
    qhT context;
    FILE* errors = std::tmpfile();
    QhullContext() {
        if (!errors) throw std::runtime_error("Cannot create Qhull diagnostic stream");
        qh_zero(&context, errors);
    }
    ~QhullContext() {
        qh_freeqhull(&context, !qh_ALL);
        int current = 0, total = 0;
        qh_memfreeshort(&context, &current, &total);
        std::fclose(errors);
    }
    std::string Error() const {
        std::rewind(errors);
        char buffer[2048]{};
        const auto n = std::fread(buffer, 1, sizeof(buffer) - 1, errors);
        return std::string(buffer, n);
    }
};

FclShape Hull(const TriangleMesh& mesh, Bounds& bounds, CollisionBodyInfo& info) {
    // Exact deduplication only. All original finite vertices participate, even
    // vertices belonging to faces discarded by the explicit loader option.
    std::set<std::array<double, 3>> unique;
    for (const auto& p : mesh.vertices) unique.insert({p.x(), p.y(), p.z()});
    Require(unique.size() >= 4 && unique.size() <= std::size_t(std::numeric_limits<int>::max()), "Invalid hull point count");
    std::vector<double> points;
    points.reserve(unique.size() * 3);
    for (const auto& p : unique) points.insert(points.end(), p.begin(), p.end());
    QhullContext owner;
    qhT* qh = &owner.context;
    char flags[] = "qhull Qt Qx";  // No jitter: never perturb source points.
    if (qh_new_qhull(qh, 3, static_cast<int>(unique.size()), points.data(), False, flags, nullptr, owner.errors))
        throw std::runtime_error("Cannot form a solid 3D hull: " + owner.Error());
    auto vertices = std::make_shared<std::vector<fcl::Vector3d>>();
    auto faces = std::make_shared<std::vector<int>>();
    std::map<int, int> indices;
    vertexT* vertex;
    FORALLvertices {
        indices.emplace(qh_pointid(qh, vertex->point), static_cast<int>(vertices->size()));
        vertices->emplace_back(vertex->point[0], vertex->point[1], vertex->point[2]);
    }
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& p : *vertices) center += p;
    center /= static_cast<double>(vertices->size());
    facetT* facet;
    vertexT** vertexp;
    double expansion = 1;
    double minimum_height = kInfinity;
    FORALLfacets {
        std::vector<int> triangle;
        FOREACHvertex_(facet->vertices) triangle.push_back(indices.at(qh_pointid(qh, vertex->point)));
        Require(triangle.size() == 3, "Qhull did not produce triangular faces");
        Eigen::Vector3d normal = (vertices->at(triangle[1]) - vertices->at(triangle[0])).cross(
            vertices->at(triangle[2]) - vertices->at(triangle[0]));
        Require(normal.norm() > 0 && normal.allFinite(), "Degenerate hull face");
        if (normal.dot(Eigen::Vector3d(facet->normal[0], facet->normal[1], facet->normal[2])) < 0) {
            std::swap(triangle[1], triangle[2]);
            normal = -normal;
        }
        normal.normalize();
        const double height = normal.dot(vertices->at(triangle[0]) - center);
        Require(std::isfinite(height) && height > 1e-12, "Hull has no robust 3D interior");
        minimum_height = std::min(minimum_height, height);
        for (const auto& p : unique)
            expansion = std::max(expansion, normal.dot(Eigen::Vector3d(p[0], p[1], p[2]) - center) / height);
        faces->insert(faces->end(), {3, triangle[0], triangle[1], triangle[2]});
    }
    // Expand uniformly about an interior point so every source vertex is on
    // the inner side of every face, with a small numerical outward padding.
    expansion += kHullPadding / minimum_height;
    Require(std::isfinite(expansion), "Hull expansion overflow");
    bounds.minimum = Eigen::Vector3d::Constant(kInfinity);
    bounds.maximum = -bounds.minimum;
    for (auto& p : *vertices) {
        info.hull_expansion_m = std::max(info.hull_expansion_m, (expansion - 1) * (p - center).norm());
        p = center + expansion * (p - center);
        bounds.minimum = bounds.minimum.cwiseMin(p);
        bounds.maximum = bounds.maximum.cwiseMax(p);
    }
    info.hull_vertices = vertices->size();
    info.hull_faces = faces->size() / 4;
    Require(info.hull_vertices >= 4 && info.hull_faces >= 4, "Incomplete hull");
    return std::make_shared<fcl::Convexd>(vertices, static_cast<int>(info.hull_faces), faces, true);
}

double Radius(const Bounds& b) { return b.minimum.cwiseAbs().cwiseMax(b.maximum.cwiseAbs()).norm(); }

std::pair<std::string, std::string> Ordered(std::string a, std::string b) {
    if (b < a) std::swap(a, b);
    return {std::move(a), std::move(b)};
}

void ValidateOptions(const CheckOptions& o) {
    Require(std::isfinite(o.clearance_m) && o.clearance_m >= 0
        && std::isfinite(o.numerical_guard_m) && o.numerical_guard_m >= 1e-8
        && o.max_scene_age_ns >= 0 && o.max_state_checks > 0 && o.max_subdivision_depth <= 60,
        "Invalid check options");
}
}  // namespace

struct CollisionChecker::Impl {
    struct Body {
        std::string link;
        Eigen::Isometry3d link_from_shape;
        FclShape shape;
        Bounds bounds;
        double ownership_inset_m = 0;
        std::vector<std::pair<Eigen::Vector3d, double>> mask_planes;
        std::map<std::string, double> displacement_coefficients;
    };
    RobotModel model;
    std::string id;
    std::vector<Body> bodies;
    std::vector<CollisionBodyInfo> info;
    std::vector<SelfCollisionExclusion> exclusions;
    std::set<std::pair<std::string, std::string>> excluded;

    double Displacement(const JointPositions& from, const JointPositions& to) const {
        double result = 0;
        for (const auto& body : bodies) {
            double bound = 0;
            for (const auto& [joint, coefficient] : body.displacement_coefficients)
                bound += coefficient * std::abs(to.at(joint) - from.at(joint));
            result = std::max(result, bound);
        }
        return result;
    }
    double PairDisplacement(std::size_t first, const std::optional<std::size_t>& second,
            const JointPositions& from, const JointPositions& to) const {
        const auto& a = bodies.at(first).displacement_coefficients;
        double bound = 0;
        // Common ancestor joints apply the same rigid transform to both
        // bodies, so they cannot change self-pair separation.
        for (const auto& [joint, coefficient] : a)
            if (!second || !bodies.at(*second).displacement_coefficients.count(joint))
                bound += coefficient * std::abs(to.at(joint)-from.at(joint));
        if (second)
            for (const auto& [joint, coefficient] : bodies.at(*second).displacement_coefficients)
                if (!a.count(joint)) bound += coefficient * std::abs(to.at(joint)-from.at(joint));
        return bound;
    }
};

CollisionChecker::CollisionChecker(RobotModel model, const PackageRoots& packages,
        const MeshDirectories& directories, const MeshLoadOptions& mesh_options,
        std::vector<SelfCollisionExclusion> exclusions) : impl_(std::make_unique<Impl>()) {
    auto& p = *impl_;
    p.model = std::move(model);
    p.exclusions = std::move(exclusions);
    std::set<std::string> links;
    std::map<std::string, const Joint*> parents;
    for (const auto& j : p.model.Joints()) parents.emplace(j.child, &j);
    for (const auto& link : p.model.Links()) links.insert(link.name);
    std::ostringstream identity;
    identity << "fcl_solid_hull_v2_all_source_vertices;outward_padding_m=1e-6;" << p.model.Sha256()
             << ";discard_zero_area=" << mesh_options.discard_zero_area_triangles << '\n';
    for (const auto& e : p.exclusions) {
        Require(e.first_link != e.second_link && links.count(e.first_link) && links.count(e.second_link)
            && e.reason.find_first_not_of(" \t\r\n") != std::string::npos, "Invalid self-collision exclusion");
        Require(p.excluded.insert(Ordered(e.first_link, e.second_link)).second, "Duplicate exclusion");
        identity << e.first_link.size() << ':' << e.first_link << e.second_link.size() << ':' << e.second_link
                 << e.reason.size() << ':' << e.reason << '\n';
    }
    for (const auto& link : p.model.Links()) {
        Require(!link.collisions.empty(), "Missing collision geometry: " + link.name);
        for (std::size_t i = 0; i < link.collisions.size(); ++i) {
            const auto& g = link.collisions[i];
            Impl::Body body;
            body.link = link.name;
            body.link_from_shape = g.link_from_geometry;
            ValidatePose(body.link_from_shape);
            CollisionBodyInfo info;
            info.id = link.name + "/collision/" + std::to_string(i);
            info.link = link.name;
            if (g.shape == Shape::kMesh) {
                const auto mesh = LoadStl(ResolveMesh(g.mesh_uri, p.model.SourceDirectory(), packages, directories), g.mesh_scale, mesh_options);
                info.mesh_sha256 = mesh.sha256;
                info.representation = "outward_padded_solid_convex_hull";
                body.shape = Hull(mesh, body.bounds, info);
                body.ownership_inset_m = info.hull_expansion_m;
                const auto& convex = static_cast<const fcl::Convexd&>(*body.shape);
                const auto& vertices = convex.getVertices(); const auto& faces = convex.getFaces();
                for (std::size_t j = 0; j < faces.size(); j += 4) {
                    const auto& a = vertices.at(faces[j+1]);
                    const Eigen::Vector3d normal = (vertices.at(faces[j+2])-a).cross(vertices.at(faces[j+3])-a).normalized();
                    body.mask_planes.emplace_back(normal, normal.dot(a));
                }
            } else {
                info.representation = "primitive_solid";
                body.shape = Primitive(g);
                body.bounds = GeometryBounds(g);
            }
            identity << info.id << ':' << info.mesh_sha256 << '\n';
            // Radius from each ancestor joint's axis to any point of this body.
            // Include descendant joint offsets and the full prismatic range.
            double reach = body.link_from_shape.translation().norm() + Radius(body.bounds);
            std::string child = link.name;
            while (parents.count(child)) {
                const auto& j = *parents.at(child);
                if (j.type == JointType::kRevolute || j.type == JointType::kContinuous)
                    body.displacement_coefficients[j.name] = reach;
                else if (j.type == JointType::kPrismatic) {
                    body.displacement_coefficients[j.name] = 1;
                    reach += std::max(std::abs(j.lower), std::abs(j.upper));
                }
                reach += j.parent_from_joint.translation().norm();
                child = j.parent;
            }
            Require(std::isfinite(reach), "Motion displacement bound overflow");
            p.info.push_back(std::move(info));
            p.bodies.push_back(std::move(body));
        }
    }
    p.id = Hash(identity.str());
}

CollisionChecker::~CollisionChecker() = default;
CollisionChecker::CollisionChecker(CollisionChecker&&) noexcept = default;
CollisionChecker& CollisionChecker::operator=(CollisionChecker&&) noexcept = default;
const std::string& CollisionChecker::ModelId() const { return impl_->id; }
const std::string& CollisionChecker::UrdfSha256() const { return impl_->model.Sha256(); }
const std::vector<CollisionBodyInfo>& CollisionChecker::Bodies() const { return impl_->info; }
const std::vector<SelfCollisionExclusion>& CollisionChecker::Exclusions() const { return impl_->exclusions; }

std::vector<CollisionBodyPreview> CollisionChecker::PreviewBodies(const JointPositions& q) const {
    const auto poses = impl_->model.Forward(q);
    std::vector<CollisionBodyPreview> result;
    for (std::size_t i = 0; i < impl_->bodies.size(); ++i) {
        const auto& body = impl_->bodies[i];
        CollisionBodyPreview preview;
        preview.id = impl_->info[i].id;
        preview.base_from_body = poses.at(body.link) * body.link_from_shape;
        switch (body.shape->getNodeType()) {
        case fcl::GEOM_CONVEX: {
            preview.geometry.shape = Shape::kMesh;
            const auto& hull = static_cast<const fcl::Convexd&>(*body.shape);
            preview.mesh.vertices = hull.getVertices();
            const auto& faces = hull.getFaces();
            for (std::size_t j = 0; j < faces.size(); j += 4)
                preview.mesh.triangles.push_back({std::size_t(faces[j+1]), std::size_t(faces[j+2]), std::size_t(faces[j+3])});
            preview.mesh.bounds = body.bounds;
            break;
        }
        case fcl::GEOM_BOX:
            preview.geometry.shape = Shape::kBox;
            preview.geometry.box_size = static_cast<const fcl::Boxd&>(*body.shape).side;
            break;
        case fcl::GEOM_CYLINDER: {
            preview.geometry.shape = Shape::kCylinder;
            const auto& cylinder = static_cast<const fcl::Cylinderd&>(*body.shape);
            preview.geometry.radius = cylinder.radius;
            preview.geometry.length = cylinder.lz;
            break;
        }
        case fcl::GEOM_SPHERE:
            preview.geometry.shape = Shape::kSphere;
            preview.geometry.radius = static_cast<const fcl::Sphered&>(*body.shape).radius;
            break;
        default: throw std::runtime_error("Unsupported preview geometry");
        }
        result.push_back(std::move(preview));
    }
    return result;
}

StateCheck CollisionChecker::CheckState(const JointPositions& q, const CollisionScene& scene,
        std::int64_t now_ns, const CheckOptions& options) const {
    StateCheck result;
    result.model_id = impl_->id;
    result.scene_id = scene.id;
    result.calibration_id = scene.calibration_id;
    result.captured_at_ns = scene.captured_at_ns;
    result.minimum_distance_m = kInfinity;
    result.minimum_environment_distance_m = kInfinity;
    result.minimum_self_distance_m = kInfinity;
    result.workspace_clearance_m = -kInfinity;
    try {
        ValidateOptions(options);
        Require(!scene.id.empty() && !scene.calibration_id.empty() && scene.frame == impl_->model.Root(),
            "Scene identity/calibration/frame missing or mismatched");
        Require(scene.captured_at_ns >= 0 && now_ns >= 0, "Negative time");
        if (scene.captured_at_ns > now_ns || now_ns - scene.captured_at_ns > options.max_scene_age_ns) {
            result.status = CheckStatus::kStaleScene;
            result.detail = "Scene expired or capture time is in the future";
            return result;
        }
        if (scene.known_space) ValidateBounds(*scene.known_space);
        Require(!(scene.known_space && scene.observation), "Cannot combine manual known space and camera observation");
        if (scene.observation) Require(bool(scene.observation->tree), "Missing observation tree");
        const auto poses = impl_->model.Forward(q);
        std::vector<PlacedShape> robot, obstacles;
        result.workspace_clearance_m = !options.allow_unknown_space &&
            (scene.known_space || scene.observation) ? kInfinity : -kInfinity;
        for (const auto& b : impl_->bodies) {
            const Eigen::Isometry3d transform = poses.at(b.link) * b.link_from_shape;
            robot.push_back({b.shape, transform});
            if (scene.known_space && !options.allow_unknown_space) {
                const auto world = TransformBounds(b.bounds, transform);
                result.workspace_clearance_m = std::min({result.workspace_clearance_m,
                    (world.minimum - scene.known_space->minimum).minCoeff(),
                    (scene.known_space->maximum - world.maximum).minCoeff()});
            }
            if (scene.observation && !options.allow_unknown_space) {
                const auto& observation = *scene.observation;
                const bool has_capture = bool(observation.captured_robot_contains_ball);
                Require(!has_capture || observation.robot_model_id == impl_->id,
                        "Captured robot volume/model identity mismatch");
                // Root-fixed bodies (base and support) never sweep into new
                // space. Keep them in all FCL environment/self queries below.
                if (!(has_capture && b.displacement_coefficients.empty())) {
                    std::string reason;
                    const double edge = observation.tree->getResolution();
                    const fcl::Boxd cell(edge, edge, edge);
                    auto distance_to_cell = [&](const Eigen::Vector3d& center, double) {
                        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                        pose.translation() = center;
                        fcl::CollisionRequestd request;
                        request.gjk_tolerance = 1e-9;
                        fcl::CollisionResultd collision;
                        fcl::collide(b.shape.get(), transform, &cell, pose, request, collision);
                        if (collision.isCollision()) return 0.;
                        fcl::DistanceRequestd distance_request;
                        distance_request.gjk_solver_type = fcl::GST_LIBCCD;
                        distance_request.distance_tolerance = 1e-9;
                        fcl::DistanceResultd separation;
                        fcl::distance(b.shape.get(), transform, &cell, pose, distance_request, separation);
                        Require(std::isfinite(separation.min_distance) && separation.min_distance >= 0,
                                "Failed body-to-voxel distance query");
                        // Account for the numeric tolerance before skipping a
                        // cell; errors never turn a near cell into free space.
                        return std::max(0., separation.min_distance - options.numerical_guard_m);
                    };
                    Eigen::Vector3d failure_point = Eigen::Vector3d::Constant(
                        std::numeric_limits<double>::quiet_NaN());
                    const double margin = observation.KnownFreeMargin(TransformBounds(b.bounds, transform),
                        &reason, distance_to_cell, &failure_point);
                    result.workspace_clearance_m = std::min(result.workspace_clearance_m, margin);
                    if (!reason.empty()) {
                        result.unknown_space_bodies.push_back(b.link + ":" + reason);
                        result.unknown_space_points_base_m.push_back(failure_point);
                        UnknownSpaceDiagnostic diagnostic;
                        diagnostic.body = b.link;
                        diagnostic.reason = reason;
                        diagnostic.query_margin_m = observation.free_margin_m;
                        if (failure_point.allFinite()) {
                            diagnostic.cell_center_base_m = failure_point;
                            if (observation.cell_evidence) diagnostic.cell_evidence = observation.cell_evidence(failure_point);
                            const double distance = distance_to_cell(failure_point, edge / 2);
                            diagnostic.body_to_cell_distance_lower_bound_m = distance;
                            diagnostic.first_cell_scope = distance > 0 ? "margin_only" : "body_or_numeric_guard";
                            if (has_capture) diagnostic.center_inside_captured_robot =
                                observation.captured_robot_contains_ball(failure_point, 0);
                            // Inspect the whole actual geometry, not only the
                            // first (lexicographically enumerated) margin cell.
                            Eigen::Vector3d geometry_failure = Eigen::Vector3d::Constant(
                                std::numeric_limits<double>::quiet_NaN());
                            observation.KnownFreeMargin(TransformBounds(b.bounds, transform),
                                &diagnostic.geometry_only_reason, distance_to_cell, &geometry_failure, 0);
                            diagnostic.geometry_only_status = diagnostic.geometry_only_reason.empty() ?
                                "covered" : "rejected";
                            if (geometry_failure.allFinite()) {
                                diagnostic.geometry_only_failure_point_base_m = geometry_failure;
                                if (observation.cell_evidence) diagnostic.geometry_only_cell_evidence =
                                    observation.cell_evidence(geometry_failure);
                                if (has_capture) diagnostic.geometry_only_center_inside_captured_robot =
                                    observation.captured_robot_contains_ball(geometry_failure, 0);
                            }
                        }
                        result.unknown_space_diagnostics.push_back(std::move(diagnostic));
                    }
                }
            }
        }
        if (scene.observation && scene.observation->camera_origin_base_m &&
            scene.observation->robot_occlusion_filter && !result.unknown_space_diagnostics.empty()) {
            std::vector<Eigen::Vector3d> points;
            std::vector<std::pair<std::size_t, bool>> indices;
            for (std::size_t i = 0; i < result.unknown_space_diagnostics.size(); ++i) {
                if (result.unknown_space_diagnostics[i].cell_center_base_m) {
                    points.push_back(*result.unknown_space_diagnostics[i].cell_center_base_m);
                    indices.emplace_back(i, false);
                }
                if (result.unknown_space_diagnostics[i].geometry_only_failure_point_base_m) {
                    points.push_back(*result.unknown_space_diagnostics[i].geometry_only_failure_point_base_m);
                    indices.emplace_back(i, true);
                }
            }
            const auto& observation = *scene.observation;
            const auto hidden = PointsOccludedByRobot(observation.captured_joints,
                *observation.camera_origin_base_m, points, observation.mask_padding_m +
                std::sqrt(3.) * observation.tree->getResolution() / 2 + 1e-6);
            for (std::size_t i = 0; i < indices.size(); ++i) {
                auto& diagnostic = result.unknown_space_diagnostics[indices[i].first];
                if (indices[i].second) diagnostic.geometry_only_capture_robot_occludes_cell = hidden[i];
                else diagnostic.capture_robot_occludes_cell = hidden[i];
            }
        }
        std::set<std::string> ids;
        for (const auto& o : scene.obstacles) {
            Require(!o.id.empty() && ids.insert(o.id).second, "Empty/duplicate obstacle id");
            obstacles.push_back({Primitive(o.geometry), o.geometry.link_from_geometry});
        }
        if (scene.observation) {
            bool occupied = false;
            for (auto it = scene.observation->tree->begin_leafs(); it != scene.observation->tree->end_leafs(); ++it)
                if (scene.observation->tree->isNodeOccupied(*it)) { occupied = true; break; }
            if (occupied) obstacles.push_back({std::make_shared<fcl::OcTreed>(scene.observation->tree), Eigen::Isometry3d::Identity()});
        }
        // Everything below this point is a numeric query failure, not bad input.
        result.status = CheckStatus::kQueryFailure;
        const double threshold = options.clearance_m + options.numerical_guard_m;
        auto query = [&](const PlacedShape& a, const PlacedShape& b,
                         const std::string& first, const std::string& second, bool self,
                         std::size_t first_index, std::optional<std::size_t> second_index) {
            ++result.pair_queries;
            fcl::CollisionRequestd request;
            request.gjk_tolerance = 1e-9;
            fcl::CollisionResultd collision;
            fcl::collide(a.shape.get(), a.transform, b.shape.get(), b.transform, request, collision);
            double distance = 0;
            std::optional<Eigen::Vector3d> first_point, second_point;
            if (!collision.isCollision()) {
                fcl::DistanceRequestd distance_request;
                distance_request.gjk_solver_type = fcl::GST_LIBCCD;
                distance_request.distance_tolerance = 1e-9;
                fcl::DistanceResultd separation;
                fcl::distance(a.shape.get(), a.transform, b.shape.get(), b.transform, distance_request, separation);
                distance = separation.min_distance;
                if (!std::isfinite(distance) || distance < 0)
                    throw std::runtime_error("Invalid FCL separation for " + first + " / " + second);
                if (distance <= threshold && !self) {
                    distance_request.enable_nearest_points = true;
                    separation.clear();
                    separation.nearest_points[0].setConstant(std::numeric_limits<double>::quiet_NaN());
                    separation.nearest_points[1].setConstant(std::numeric_limits<double>::quiet_NaN());
                    fcl::distance(a.shape.get(), a.transform, b.shape.get(), b.transform, distance_request, separation);
                    const auto& x = separation.nearest_points[0];
                    const auto& y = separation.nearest_points[1];
                    if (std::isfinite(separation.min_distance) && std::abs(separation.min_distance-distance) <= 1e-4 &&
                        x.allFinite() && y.allFinite() &&
                        std::abs((x-y).norm() - distance) <= 1e-4) {
                        // FCL's shape/octree dispatcher may report the octree
                        // as result.o1 even when the robot was argument one.
                        if (separation.o1 == a.shape.get() && separation.o2 == b.shape.get()) {
                            first_point = x; second_point = y;
                        } else if (separation.o1 == b.shape.get() && separation.o2 == a.shape.get()) {
                            first_point = y; second_point = x;
                        }
                    }
                }
            }
            result.minimum_distance_m = std::min(result.minimum_distance_m, distance);
            result.pair_clearances.push_back({first_index, second_index, distance});
            auto& category_distance = self ? result.minimum_self_distance_m : result.minimum_environment_distance_m;
            category_distance = std::min(category_distance, distance);
            if (collision.isCollision() || distance <= threshold)
                result.violations.push_back({first, second, self, collision.isCollision(), distance, first_point, second_point});
        };
        for (std::size_t i = 0; i < robot.size(); ++i) {
            for (std::size_t j = i + 1; j < robot.size(); ++j) {
                const auto& a = impl_->bodies[i].link;
                const auto& b = impl_->bodies[j].link;
                // Multiple collision shapes in one link describe one rigid solid.
                if (a == b || impl_->excluded.count(Ordered(a, b))) continue;
                query(robot[i], robot[j], impl_->info[i].id, impl_->info[j].id, true, i, j);
            }
            for (std::size_t j = 0; j < obstacles.size(); ++j)
                query(robot[i], obstacles[j], impl_->info[i].id,
                    j < scene.obstacles.size() ? "environment/" + scene.obstacles[j].id : "environment/depth_occupancy", false, i, std::nullopt);
        }
        if (std::any_of(result.violations.begin(), result.violations.end(), [](const PairViolation& v) { return v.intersecting; }))
            result.status = CheckStatus::kCollision;
        else if (!result.violations.empty()) result.status = CheckStatus::kClearanceViolation;
        else if (!options.allow_unknown_space && result.workspace_clearance_m <= threshold) result.status = CheckStatus::kUnknownSpace;
        else result.status = CheckStatus::kValid;
        result.detail = CheckStatusName(result.status);
        if (result.Valid() && options.allow_unknown_space) result.detail = "valid_with_unknown_space_allowed";
    } catch (const std::exception& e) {
        if (result.status != CheckStatus::kQueryFailure) result.status = CheckStatus::kInvalidInput;
        result.detail = e.what();
    }
    return result;
}

std::function<bool(const Eigen::Vector3d&, double)> CollisionChecker::CaptureRobotVolume(
        const JointPositions& q) const {
    const auto poses = impl_->model.Forward(q);
    auto bodies = impl_->bodies;
    std::vector<Eigen::Isometry3d> local_from_base;
    for (const auto& body : bodies)
        local_from_base.push_back((poses.at(body.link) * body.link_from_shape).inverse());
    return [bodies = std::move(bodies), local_from_base = std::move(local_from_base)](
            const Eigen::Vector3d& center, double radius) {
        if (!center.allFinite() || !std::isfinite(radius) || radius < 0) return false;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const auto& body = bodies[i];
            const Eigen::Vector3d p = local_from_base[i] * center;
            if ((p.array() - radius < body.bounds.minimum.array()).any() ||
                (p.array() + radius > body.bounds.maximum.array()).any()) continue;
            switch (body.shape->getNodeType()) {
            case fcl::GEOM_CONVEX: {
                bool contained = true;
                for (const auto& [normal, offset] : body.mask_planes)
                    if (normal.dot(p) + radius + body.ownership_inset_m > offset) { contained = false; break; }
                if (contained) return true;
                break;
            }
            case fcl::GEOM_BOX:
                if ((p.cwiseAbs().array() + radius <=
                    static_cast<const fcl::Boxd&>(*body.shape).side.array() / 2).all()) return true;
                break;
            case fcl::GEOM_SPHERE:
                if (p.norm() + radius <= static_cast<const fcl::Sphered&>(*body.shape).radius) return true;
                break;
            case fcl::GEOM_CYLINDER: {
                const auto& cylinder = static_cast<const fcl::Cylinderd&>(*body.shape);
                if (p.head<2>().norm() + radius <= cylinder.radius &&
                    std::abs(p.z()) + radius <= cylinder.lz / 2) return true;
                break;
            }
            default: break;
            }
        }
        return false;
    };
}

std::vector<bool> CollisionChecker::PointsInsideRobot(const JointPositions& q,
        const std::vector<Eigen::Vector3d>& points, double padding) const {
    Require(std::isfinite(padding) && padding >= 0, "Invalid robot mask padding");
    const auto poses = impl_->model.Forward(q);
    std::vector<bool> inside(points.size(), false);
    for (const auto& body : impl_->bodies) {
        const Eigen::Isometry3d local_from_base = (poses.at(body.link) * body.link_from_shape).inverse();
        const auto type = body.shape->getNodeType();
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (inside[i] || !points[i].allFinite()) continue;
            const Eigen::Vector3d point = local_from_base * points[i];
            if ((point.array() < body.bounds.minimum.array() - padding).any()
                || (point.array() > body.bounds.maximum.array() + padding).any()) continue;
            if (type == fcl::GEOM_CONVEX) {
                bool contained = true;
                for (const auto& [normal, offset] : body.mask_planes) {
                    if (normal.dot(point) - offset > padding) { contained = false; break; }
                }
                inside[i] = contained;
            } else if (type == fcl::GEOM_SPHERE) {
                inside[i] = point.norm() <= static_cast<const fcl::Sphered&>(*body.shape).radius + padding;
            } else if (type == fcl::GEOM_BOX) {
                inside[i] = (point.cwiseAbs().array() <= static_cast<const fcl::Boxd&>(*body.shape).side.array() / 2 + padding).all();
            } else if (type == fcl::GEOM_CYLINDER) {
                const auto& cylinder = static_cast<const fcl::Cylinderd&>(*body.shape);
                inside[i] = point.head<2>().norm() <= cylinder.radius + padding && std::abs(point.z()) <= cylinder.lz/2 + padding;
            }
        }
    }
    return inside;
}

std::vector<bool> CollisionChecker::PointsOccludedByRobot(const JointPositions& q,
        const Eigen::Vector3d& origin, const std::vector<Eigen::Vector3d>& points, double padding) const {
    Require(origin.allFinite() && std::isfinite(padding) && padding >= 0, "Invalid robot occlusion context");
    const auto poses = impl_->model.Forward(q);
    std::vector<bool> occluded(points.size(), false);
    std::vector<Eigen::Vector3d> directions(points.size());
    std::vector<double> ranges(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        Require(points[i].allFinite(), "Non-finite occlusion point");
        directions[i] = points[i] - origin;
        ranges[i] = directions[i].norm();
        Require(std::isfinite(ranges[i]), "Invalid occlusion range");
        if (ranges[i] > 1e-12) directions[i] /= ranges[i];
    }
    std::size_t work = 0;
    auto budget = [&] { Require(++work <= 20000000, "Robot occlusion query budget exceeded"); };
    for (const auto& body : impl_->bodies) {
        const Eigen::Isometry3d local = (poses.at(body.link) * body.link_from_shape).inverse();
        const Eigen::Vector3d camera = local * origin;
        const Eigen::Vector3d lo = body.bounds.minimum.array() - padding;
        const Eigen::Vector3d hi = body.bounds.maximum.array() + padding;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (occluded[i] || ranges[i] <= 1e-12) continue;
            budget();
            const Eigen::Vector3d direction = local.linear() * directions[i];
            double first = 0, last = ranges[i];
            bool hit = true;
            for (int axis = 0; axis < 3; ++axis) {
                if (std::abs(direction[axis]) < 1e-12) {
                    if (camera[axis] < lo[axis] || camera[axis] > hi[axis]) { hit = false; break; }
                } else {
                    double near = (lo[axis] - camera[axis]) / direction[axis];
                    double far = (hi[axis] - camera[axis]) / direction[axis];
                    if (near > far) std::swap(near, far);
                    first = std::max(first, near); last = std::min(last, far);
                    if (first > last) { hit = false; break; }
                }
            }
            if (!hit) continue;
            const auto type = body.shape->getNodeType();
            if (type == fcl::GEOM_CONVEX) {
                for (const auto& [normal, offset] : body.mask_planes) {
                    budget();
                    const double available = offset + padding - normal.dot(camera);
                    const double slope = normal.dot(direction);
                    if (std::abs(slope) < 1e-12) {
                        if (available < 0) { hit = false; break; }
                    } else {
                        const double crossing = available / slope;
                        if (slope > 0) last = std::min(last, crossing);
                        else first = std::max(first, crossing);
                        if (first > last) { hit = false; break; }
                    }
                }
            } else if (type == fcl::GEOM_SPHERE || type == fcl::GEOM_CYLINDER) {
                const bool sphere = type == fcl::GEOM_SPHERE;
                const double radius = (sphere ? static_cast<const fcl::Sphered&>(*body.shape).radius
                    : static_cast<const fcl::Cylinderd&>(*body.shape).radius) + padding;
                const double a = sphere ? direction.squaredNorm() : direction.head<2>().squaredNorm();
                const double b = 2 * (sphere ? camera.dot(direction) : camera.head<2>().dot(direction.head<2>()));
                const double c = (sphere ? camera.squaredNorm() : camera.head<2>().squaredNorm()) - radius * radius;
                if (a < 1e-16) hit = c <= 0;
                else {
                    const double discriminant = b * b - 4 * a * c;
                    if (discriminant < 0) hit = false;
                    else {
                        const double root = std::sqrt(discriminant);
                        first = std::max(first, (-b - root) / (2 * a));
                        last = std::min(last, (-b + root) / (2 * a));
                        hit = first <= last;
                    }
                }
            } else {
                Require(type == fcl::GEOM_BOX, "Unsupported robot occlusion geometry");
            }
            if (hit) occluded[i] = true;
        }
    }
    return occluded;
}

SegmentCheck CollisionChecker::CheckSegment(const JointPositions& start, const JointPositions& goal,
        const CollisionScene& scene, std::int64_t now_ns, const CheckOptions& options,
        const std::function<bool()>& cancelled) const {
    SegmentCheck result;
    result.model_id = impl_->id;
    result.scene_id = scene.id;
    result.calibration_id = scene.calibration_id;
    result.captured_at_ns = scene.captured_at_ns;
    result.certified_clearance_lower_bound_m = kInfinity;
    try {
        ValidateOptions(options);
        impl_->model.Forward(start);
        impl_->model.Forward(goal);
        auto evaluate = [&](const JointPositions& q, double fraction) -> std::optional<StateCheck> {
            if (cancelled && cancelled()) {
                result.status = CheckStatus::kCancelled;
                result.detail = "Segment check cancelled";
            } else if (result.state_checks >= options.max_state_checks) {
                result.status = CheckStatus::kBudgetExceeded;
                result.detail = "State-query budget exhausted";
            } else {
                ++result.state_checks;
                auto state = CheckState(q, scene, now_ns, options);
                if (state.Valid()) return state;
                result.status = state.status;
                result.detail = state.detail;
                result.failure = std::move(state);
            }
            result.failure_fraction = fraction;
            return std::nullopt;
        };
        if (!evaluate(start, 0) || !evaluate(goal, 1)) return result;
        std::function<bool(const JointPositions&, const JointPositions&, double, double, std::size_t)> visit;
        visit = [&](const JointPositions& a, const JointPositions& b, double lo, double hi, std::size_t depth) {
            JointPositions midpoint;
            for (const auto& [name, value] : a) midpoint[name] = .5 * value + .5 * b.at(name);
            const double fraction = .5 * lo + .5 * hi;
            const auto state = evaluate(midpoint, fraction);
            if (!state) return false;
            const double displacement = std::max(impl_->Displacement(a, midpoint), impl_->Displacement(b, midpoint));
            Require(std::isfinite(displacement), "Segment displacement bound overflow");
            // Certify each pair with its own relative motion. A close fixed
            // base/environment pair must not force subdivision of moving tips.
            double lower_bound = kInfinity;
            for (const auto& pair : state->pair_clearances) {
                const double motion = std::max(
                    impl_->PairDisplacement(pair.first_body, pair.second_body, a, midpoint),
                    impl_->PairDisplacement(pair.first_body, pair.second_body, b, midpoint));
                Require(std::isfinite(motion), "Relative displacement bound overflow");
                lower_bound = std::min(lower_bound, pair.distance_m - motion);
            }
            const double threshold = options.clearance_m + options.numerical_guard_m;
            if (lower_bound > threshold && (options.allow_unknown_space ||
                state->workspace_clearance_m - displacement > threshold)) {
                ++result.certified_intervals;
                result.certified_clearance_lower_bound_m = std::min(result.certified_clearance_lower_bound_m, lower_bound);
                return true;
            }
            if (depth >= options.max_subdivision_depth || fraction == lo || fraction == hi) {
                result.status = CheckStatus::kBudgetExceeded;
                result.detail = "Cannot certify interval within subdivision budget";
                result.failure_fraction = fraction;
                return false;
            }
            return visit(a, midpoint, lo, fraction, depth + 1) && visit(midpoint, b, fraction, hi, depth + 1);
        };
        if (visit(start, goal, 0, 1, 0)) {
            result.status = CheckStatus::kValid;
            result.detail = options.allow_unknown_space ?
                "Entire joint-linear segment certified against observed obstacles; unknown space allowed" :
                "Entire joint-linear segment certified against static known scene";
        }
    } catch (const std::exception& e) {
        result.status = CheckStatus::kInvalidInput;
        result.detail = e.what();
    }
    return result;
}

const char* CheckStatusName(CheckStatus status) {
    switch (status) {
    case CheckStatus::kValid: return "valid";
    case CheckStatus::kCollision: return "collision";
    case CheckStatus::kClearanceViolation: return "clearance_violation";
    case CheckStatus::kUnknownSpace: return "unknown_space";
    case CheckStatus::kStaleScene: return "stale_scene";
    case CheckStatus::kInvalidInput: return "invalid_input";
    case CheckStatus::kQueryFailure: return "query_failure";
    case CheckStatus::kBudgetExceeded: return "budget_exceeded";
    case CheckStatus::kCancelled: return "cancelled";
    }
    return "invalid_input";
}
}  // namespace uspilot::collision
