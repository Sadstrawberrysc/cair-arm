#include "collision/collision_checker.hpp"
#include "unknown_space_json.hpp"
#include <json.hpp>
#include <fcl/config.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

namespace c = uspilot::collision;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
namespace {
void Keys(const Json& j, const std::set<std::string>& keys) {
    if (!j.is_object()) throw std::invalid_argument("Expected JSON object");
    for (auto it = j.begin(); it != j.end(); ++it)
        if (!keys.count(it.key())) throw std::invalid_argument("Unknown JSON field: " + it.key());
}
double Number(const Json& j) {
    if (!j.is_number()) throw std::invalid_argument("Expected numeric value");
    const double v = j.get<double>();
    if (!std::isfinite(v)) throw std::invalid_argument("Expected finite value");
    return v;
}
std::int64_t Time(const Json& j) {
    if (!j.is_number_integer() || Number(j) < 0 || Number(j) >= 9223372036854775808.0)
        throw std::invalid_argument("Expected nonnegative int64 nanoseconds");
    return j.get<std::int64_t>();
}
std::size_t Count(const Json& j) {
    const auto v = Time(j);
    if (static_cast<std::uint64_t>(v) > std::numeric_limits<std::size_t>::max()) throw std::invalid_argument("Count overflow");
    return static_cast<std::size_t>(v);
}
Eigen::Vector3d Vector(const Json& j) {
    if (!j.is_array() || j.size() != 3) throw std::invalid_argument("Expected three-vector");
    return {Number(j[0]), Number(j[1]), Number(j[2])};
}
c::JointPositions Joints(const Json& j) {
    if (!j.is_object()) throw std::invalid_argument("Expected named joint positions");
    c::JointPositions values;
    for (auto it = j.begin(); it != j.end(); ++it) values.emplace(it.key(), Number(it.value()));
    return values;
}
Json Finite(double x) { return std::isfinite(x) ? Json(x) : Json(nullptr); }
Json State(const c::StateCheck& s) {
    Json pairs = Json::array();
    Json unknown_points = Json::array();
    for (const auto& p : s.unknown_space_points_base_m)
        unknown_points.push_back(p.allFinite() ? Json::array({p.x(), p.y(), p.z()}) : Json(nullptr));
    for (const auto& p : s.violations) pairs.push_back({{"first_body", p.first_body}, {"second_body", p.second_body},
        {"self_collision", p.self_collision}, {"intersecting", p.intersecting}, {"distance_m", p.distance_m}});
    return {{"status", c::CheckStatusName(s.status)}, {"valid", s.Valid()}, {"detail", s.detail},
        {"minimum_distance_m", Finite(s.minimum_distance_m)}, {"workspace_clearance_m", Finite(s.workspace_clearance_m)},
        {"pair_queries", s.pair_queries}, {"violations", pairs}, {"unknown_space_bodies", s.unknown_space_bodies},
        {"unknown_space_diagnostics", c::app::UnknownSpaceDiagnostics(s)},
        {"unknown_space_points_base_m", unknown_points}};
}
double Milliseconds(Clock::time_point start) { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); }
}  // namespace

int main(int argc, char** argv) {
    try {
        std::filesystem::path urdf, request_path;
        c::MeshDirectories directories;
        c::MeshLoadOptions mesh_options;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "Offline FCL solid collision audit. No robot/network/device access.\n"
                    "collision_check --urdf FILE --request JSON [--mesh-dir DIR] [--drop-zero-area-triangles]\n"
                    "Exit 0: requested offline checks valid; 2: rejected; 1: invalid input/loading failure.\n"
                    "Scene timestamps and now_ns must share the caller's clock domain.\n";
                return 0;
            }
            if (arg == "--drop-zero-area-triangles") { mesh_options.discard_zero_area_triangles = true; continue; }
            if (arg != "--urdf" && arg != "--request" && arg != "--mesh-dir") throw std::invalid_argument("Unknown option: " + arg);
            if (++i >= argc) throw std::invalid_argument("Missing option value");
            if (arg == "--urdf") { if (!urdf.empty()) throw std::invalid_argument("Repeated --urdf"); urdf = argv[i]; }
            else if (arg == "--request") { if (!request_path.empty()) throw std::invalid_argument("Repeated --request"); request_path = argv[i]; }
            else if (!directories.emplace("package://rm_description/meshes/rm_75_arm/", std::filesystem::canonical(argv[i])).second)
                throw std::invalid_argument("Repeated --mesh-dir");
        }
        if (urdf.empty() || request_path.empty()) throw std::invalid_argument("Specify --urdf and --request");
        if (std::filesystem::file_size(request_path) > 4 * 1024 * 1024) throw std::invalid_argument("Request exceeds 4 MiB");
        std::ifstream file(request_path);
        Json request = Json::parse(file);
        Keys(request, {"schema_version", "scene", "now_ns", "joint_positions_rad", "goal_joint_positions_rad", "options", "self_collision_exclusions"});
        if (!request.at("schema_version").is_number_integer() || request.at("schema_version") != 1)
            throw std::invalid_argument("Unsupported schema_version");
        const auto now = Time(request.at("now_ns"));
        const auto& sj = request.at("scene");
        Keys(sj, {"id", "calibration_id", "frame", "captured_at_ns", "known_space", "obstacles"});
        c::CollisionScene scene;
        scene.id = sj.at("id").get<std::string>();
        scene.calibration_id = sj.at("calibration_id").get<std::string>();
        scene.frame = sj.at("frame").get<std::string>();
        scene.captured_at_ns = Time(sj.at("captured_at_ns"));
        if (sj.contains("known_space") && !sj.at("known_space").is_null()) {
            const auto& b = sj.at("known_space");
            Keys(b, {"min_m", "max_m"});
            scene.known_space = c::Bounds{Vector(b.at("min_m")), Vector(b.at("max_m"))};
        }
        if (!sj.at("obstacles").is_array() || sj.at("obstacles").size() > 10000)
            throw std::invalid_argument("Expected bounded obstacle array");
        for (const auto& j : sj.at("obstacles")) {
            c::Obstacle obstacle;
            obstacle.id = j.at("id").get<std::string>();
            const auto type = j.at("shape").get<std::string>();
            if (type == "box") {
                Keys(j, {"id", "shape", "size_m", "xyz_m", "rpy_rad"});
                obstacle.geometry.shape = c::Shape::kBox;
                obstacle.geometry.box_size = Vector(j.at("size_m"));
            } else if (type == "sphere") {
                Keys(j, {"id", "shape", "radius_m", "xyz_m", "rpy_rad"});
                obstacle.geometry.shape = c::Shape::kSphere;
                obstacle.geometry.radius = Number(j.at("radius_m"));
            } else if (type == "cylinder") {
                Keys(j, {"id", "shape", "radius_m", "length_m", "xyz_m", "rpy_rad"});
                obstacle.geometry.shape = c::Shape::kCylinder;
                obstacle.geometry.radius = Number(j.at("radius_m"));
                obstacle.geometry.length = Number(j.at("length_m"));
            } else throw std::invalid_argument("Unsupported obstacle shape");
            obstacle.geometry.link_from_geometry.translation() = Vector(j.at("xyz_m"));
            const auto rpy = Vector(j.at("rpy_rad"));
            obstacle.geometry.link_from_geometry.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
            scene.obstacles.push_back(std::move(obstacle));
        }
        c::CheckOptions options;
        if (request.contains("options")) {
            const auto& o = request.at("options");
            Keys(o, {"clearance_m", "numerical_guard_m", "max_scene_age_ns", "max_state_checks", "max_subdivision_depth"});
            if (o.contains("clearance_m")) options.clearance_m = Number(o.at("clearance_m"));
            if (o.contains("numerical_guard_m")) options.numerical_guard_m = Number(o.at("numerical_guard_m"));
            if (o.contains("max_scene_age_ns")) options.max_scene_age_ns = Time(o.at("max_scene_age_ns"));
            if (o.contains("max_state_checks")) options.max_state_checks = Count(o.at("max_state_checks"));
            if (o.contains("max_subdivision_depth")) options.max_subdivision_depth = Count(o.at("max_subdivision_depth"));
        }
        std::vector<c::SelfCollisionExclusion> exclusions;
        if (request.contains("self_collision_exclusions")) {
            if (!request.at("self_collision_exclusions").is_array()) throw std::invalid_argument("Expected exclusions array");
            for (const auto& e : request.at("self_collision_exclusions")) {
                Keys(e, {"first_link", "second_link", "reason"});
                exclusions.push_back({e.at("first_link").get<std::string>(), e.at("second_link").get<std::string>(), e.at("reason").get<std::string>()});
            }
        }
        const auto load_start = Clock::now();
        auto model = c::RobotModel::Load(urdf);
        const auto urdf_sha256 = model.Sha256();
        c::CollisionChecker checker(std::move(model), {}, directories, mesh_options, exclusions);
        const double load_ms = Milliseconds(load_start);
        const auto q = Joints(request.at("joint_positions_rad"));
        const auto state_start = Clock::now();
        const auto state = checker.CheckState(q, scene, now, options);
        Json output = {{"mode", "offline_fcl_solid_audit"}, {"fcl_version", FCL_VERSION},
            {"robot_motion_enabled", false}, {"ready_for_motion_planning", false},
            {"urdf_sha256", urdf_sha256}, {"model_id", checker.ModelId()},
            {"scene_id", scene.id}, {"calibration_id", scene.calibration_id}, {"frame", scene.frame},
            {"captured_at_ns", scene.captured_at_ns}, {"now_ns", now},
            {"clearance_m", options.clearance_m}, {"numerical_guard_m", options.numerical_guard_m},
            {"max_scene_age_ns", options.max_scene_age_ns}, {"max_state_checks", options.max_state_checks},
            {"max_subdivision_depth", options.max_subdivision_depth},
            {"self_collision_exclusions", request.value("self_collision_exclusions", Json::array())},
            {"state", State(state)}, {"load_ms", load_ms}, {"state_check_ms", Milliseconds(state_start)},
            {"bodies", Json::array()}};
        for (const auto& b : checker.Bodies()) output["bodies"].push_back({{"id", b.id}, {"link", b.link},
            {"representation", b.representation}, {"mesh_sha256", b.mesh_sha256}, {"hull_vertices", b.hull_vertices},
            {"hull_faces", b.hull_faces}, {"hull_expansion_m", b.hull_expansion_m}});
        bool valid = state.Valid();
        if (request.contains("goal_joint_positions_rad")) {
            const auto segment_start = Clock::now();
            const auto s = checker.CheckSegment(q, Joints(request.at("goal_joint_positions_rad")), scene, now, options);
            output["segment"] = {{"status", c::CheckStatusName(s.status)}, {"valid", s.Valid()}, {"detail", s.detail},
                {"state_checks", s.state_checks}, {"certified_intervals", s.certified_intervals},
                {"failure_fraction", s.failure_fraction ? Json(*s.failure_fraction) : Json(nullptr)},
                {"certified_clearance_lower_bound_m", s.Valid() ? Finite(s.certified_clearance_lower_bound_m) : Json(nullptr)},
                {"failure", s.Valid() || s.failure.model_id.empty() ? Json(nullptr) : State(s.failure)},
                {"check_ms", Milliseconds(segment_start)}};
            valid = valid && s.Valid();
        }
        std::cout << output.dump(2) << '\n';
        return valid ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << "collision_check: " << e.what() << '\n';
        return 1;
    }
}
