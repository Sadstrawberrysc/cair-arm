#include "collision/joint_planner.hpp"
#include "unknown_space_json.hpp"
#include <json.hpp>
#include <ompl/config.h>
#include <ompl/util/Console.h>
#ifdef COLLISION_HAS_VIEWER
#include "monitor_viewer.hpp"
#endif
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <set>
#include <thread>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

namespace c = uspilot::collision;
using Json = nlohmann::json;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void Signal(int) { interrupted = 1; }
void Keys(const Json& j, const std::set<std::string>& keys) {
    if (!j.is_object()) throw std::invalid_argument("Expected JSON object");
    for (auto it = j.begin(); it != j.end(); ++it)
        if (!keys.count(it.key())) throw std::invalid_argument("Unknown field: " + it.key());
}
double Number(const Json& j) {
    if (!j.is_number() || !std::isfinite(j.get<double>())) throw std::invalid_argument("Expected finite number");
    return j.get<double>();
}
std::int64_t Count(const Json& j) {
    if (!j.is_number_integer() || Number(j) < 0 || Number(j) >= 9e18)
        throw std::invalid_argument("Expected nonnegative integer");
    return j.get<std::int64_t>();
}
Eigen::Vector3d Vec(const Json& j) {
    if (!j.is_array() || j.size() != 3) throw std::invalid_argument("Expected three-vector");
    return {Number(j[0]), Number(j[1]), Number(j[2])};
}
c::JointPositions Joints(const Json& j) {
    if (!j.is_object()) throw std::invalid_argument("Expected named joint map");
    c::JointPositions q;
    for (auto it = j.begin(); it != j.end(); ++it) q[it.key()] = Number(it.value());
    return q;
}
c::CollisionScene Scene(const Json& j) {
    Keys(j, {"id", "calibration_id", "frame", "captured_at_ns", "known_space", "obstacles"});
    c::CollisionScene s;
    s.id = j.at("id"); s.calibration_id = j.at("calibration_id"); s.frame = j.at("frame");
    s.captured_at_ns = Count(j.at("captured_at_ns"));
    if (j.contains("known_space") && !j.at("known_space").is_null()) {
        const auto& b = j.at("known_space"); Keys(b, {"min_m", "max_m"});
        s.known_space = c::Bounds{Vec(b.at("min_m")), Vec(b.at("max_m"))};
    }
    const auto& obstacles = j.at("obstacles");
    if (!obstacles.is_array() || obstacles.size() > 10000) throw std::invalid_argument("Invalid obstacle array");
    for (const auto& o : obstacles) {
        c::Obstacle body; body.id = o.at("id");
        const std::string shape = o.at("shape");
        if (shape == "box") {
            Keys(o, {"id", "shape", "size_m", "xyz_m", "rpy_rad"});
            body.geometry.shape = c::Shape::kBox; body.geometry.box_size = Vec(o.at("size_m"));
        } else if (shape == "sphere") {
            Keys(o, {"id", "shape", "radius_m", "xyz_m", "rpy_rad"});
            body.geometry.shape = c::Shape::kSphere; body.geometry.radius = Number(o.at("radius_m"));
        } else if (shape == "cylinder") {
            Keys(o, {"id", "shape", "radius_m", "length_m", "xyz_m", "rpy_rad"});
            body.geometry.shape = c::Shape::kCylinder; body.geometry.radius = Number(o.at("radius_m"));
            body.geometry.length = Number(o.at("length_m"));
        } else throw std::invalid_argument("Unsupported obstacle shape");
        body.geometry.link_from_geometry.translation() = Vec(o.at("xyz_m"));
        const auto r = Vec(o.at("rpy_rad"));
        body.geometry.link_from_geometry.linear() = (Eigen::AngleAxisd(r.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(r.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(r.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
        s.obstacles.push_back(std::move(body));
    }
    return s;
}
Json Finite(double d) { return std::isfinite(d) ? Json(d) : Json(nullptr); }
Json Diagnostic(const c::StateCheck& s) {
    Json violations = Json::array();
    Json unknown_points = Json::array();
    for (const auto& p : s.unknown_space_points_base_m)
        unknown_points.push_back(p.allFinite() ? Json::array({p.x(), p.y(), p.z()}) : Json(nullptr));
    for (const auto& p : s.violations) violations.push_back({{"first_body", p.first_body}, {"second_body", p.second_body},
        {"self_collision", p.self_collision}, {"intersecting", p.intersecting}, {"distance_m", p.distance_m}});
    return {{"status", s.detail == "skipped_start_precheck" ? "skipped" : c::CheckStatusName(s.status)}, {"detail", s.detail}, {"violations", violations},
        {"unknown_space_bodies", s.unknown_space_bodies}, {"unknown_space_points_base_m", unknown_points},
        {"unknown_space_diagnostics", c::app::UnknownSpaceDiagnostics(s)},
        {"environment_distance_m", s.model_id.empty() ? Json(nullptr) : Finite(s.minimum_environment_distance_m)},
        {"self_distance_m", s.model_id.empty() ? Json(nullptr) : Finite(s.minimum_self_distance_m)}};
}
void WriteNew(const std::filesystem::path& path, const std::string& bytes) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (fd < 0) throw std::runtime_error("Cannot create new output file (existing files are never overwritten)");
    std::size_t offset = 0;
    while (offset < bytes.size()) {
        const auto n = ::write(fd, bytes.data() + offset, bytes.size() - offset);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) { ::close(fd); ::unlink(path.c_str()); throw std::runtime_error("Output write failed"); }
        offset += n;
    }
    if (::close(fd)) throw std::runtime_error("Output close failed");
}
} // namespace

int main(int argc, char** argv) {
    try {
        std::filesystem::path urdf, request_file, output, saved_path;
        c::MeshDirectories directories; c::MeshLoadOptions meshes;
        bool preview = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "Offline joint planning; no robot, camera or network access.\n"
                    "collision_plan --urdf FILE --request JSON [--mesh-dir DIR]\n"
                    "  [--drop-zero-area-triangles] [--output NEW_JSON] [--preview] [--load-plan JSON]\n"
                    "Exit 0: exact verified offline path; 2: planning rejected; 1: input/output failure.\n";
                return 0;
            }
            if (arg == "--preview") { preview = true; continue; }
            if (arg == "--drop-zero-area-triangles") { meshes.discard_zero_area_triangles = true; continue; }
            if (arg != "--urdf" && arg != "--request" && arg != "--mesh-dir" && arg != "--output" && arg != "--load-plan")
                throw std::invalid_argument("Unknown option: " + arg);
            if (++i >= argc) throw std::invalid_argument("Missing option value");
            if (arg == "--urdf") urdf = argv[i];
            else if (arg == "--request") request_file = argv[i];
            else if (arg == "--output") output = argv[i];
            else if (arg == "--load-plan") saved_path = argv[i];
            else directories["package://rm_description/meshes/rm_75_arm/"] = std::filesystem::canonical(argv[i]);
        }
        if (urdf.empty() || request_file.empty()) throw std::invalid_argument("Specify --urdf and --request");
#ifndef COLLISION_HAS_VIEWER
        if (preview) throw std::invalid_argument("Preview not compiled; enable BUILD_MONITOR_VIEWER");
#endif
        if (!output.empty() && std::filesystem::exists(output)) throw std::invalid_argument("Output already exists");
        if (std::filesystem::file_size(request_file) > 4 * 1024 * 1024) throw std::invalid_argument("Request exceeds 4 MiB");
        std::ifstream stream(request_file); const auto request = Json::parse(stream);
        const auto request_sha = c::FileSha256(request_file);
        Keys(request, {"schema_version", "scene", "now_ns", "joint_positions_rad", "goal_joint_positions_rad",
            "options", "planning_options", "self_collision_exclusions"});
        if (request.at("schema_version") != 1) throw std::invalid_argument("Unsupported schema_version");
        const auto scene = Scene(request.at("scene")); const auto now = Count(request.at("now_ns"));
        const auto start = Joints(request.at("joint_positions_rad")), goal = Joints(request.at("goal_joint_positions_rad"));
        c::PlannerOptions options;
        if (request.contains("options")) {
            const auto& o = request.at("options");
            Keys(o, {"clearance_m", "numerical_guard_m", "max_scene_age_ns", "max_state_checks", "max_subdivision_depth"});
            if (o.contains("clearance_m")) options.collision.clearance_m = Number(o.at("clearance_m"));
            if (o.contains("numerical_guard_m")) options.collision.numerical_guard_m = Number(o.at("numerical_guard_m"));
            if (o.contains("max_scene_age_ns")) options.collision.max_scene_age_ns = Count(o.at("max_scene_age_ns"));
            if (o.contains("max_state_checks")) options.collision.max_state_checks = Count(o.at("max_state_checks"));
            if (o.contains("max_subdivision_depth")) options.collision.max_subdivision_depth = Count(o.at("max_subdivision_depth"));
        }
        if (request.contains("planning_options")) {
            const auto& p = request.at("planning_options");
            Keys(p, {"timeout_s", "range", "max_queries", "shortcut_attempts", "seed", "skip_start_precheck", "allow_unknown_space"});
            if (p.contains("allow_unknown_space")) options.collision.allow_unknown_space = p.at("allow_unknown_space").get<bool>();
            if (p.contains("skip_start_precheck")) options.skip_start_precheck = p.at("skip_start_precheck").get<bool>();
            if (p.contains("timeout_s")) options.timeout_s = Number(p.at("timeout_s"));
            if (p.contains("range")) options.range = Number(p.at("range"));
            if (p.contains("max_queries")) options.max_queries = Count(p.at("max_queries"));
            if (p.contains("shortcut_attempts")) options.shortcut_attempts = Count(p.at("shortcut_attempts"));
            if (p.contains("seed")) {
                const auto seed = Count(p.at("seed"));
                if (seed > UINT32_MAX) throw std::invalid_argument("Seed exceeds uint32");
                options.seed = seed;
            }
        }
        std::vector<c::SelfCollisionExclusion> exclusions;
        if (request.contains("self_collision_exclusions")) {
            if (!request.at("self_collision_exclusions").is_array()) throw std::invalid_argument("Expected exclusions array");
            for (const auto& e : request.at("self_collision_exclusions")) {
                Keys(e, {"first_link", "second_link", "reason"});
                exclusions.push_back({e.at("first_link"), e.at("second_link"), e.at("reason")});
            }
        }
        const auto model = c::RobotModel::Load(urdf);
        c::CollisionChecker checker(model, {}, directories, meshes, exclusions);
        std::signal(SIGINT, Signal); std::signal(SIGTERM, Signal);
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE); // stdout is a single machine-readable JSON document.
        auto plan = c::PlanResult{};
        plan.unknown_space_allowed = options.collision.allow_unknown_space;
        if (saved_path.empty())
            plan = c::PlanJointPath(model, checker, start, goal, scene, now, options, [] { return interrupted != 0; });
        else {
            if (std::filesystem::file_size(saved_path) > 8 * 1024 * 1024) throw std::invalid_argument("Saved path exceeds 8 MiB");
            std::ifstream saved_stream(saved_path); const auto saved = Json::parse(saved_stream);
            if (saved.at("model_id") != checker.ModelId() || saved.at("request_sha256") != request_sha ||
                saved.at("scene_id") != scene.id || saved.at("calibration_id") != scene.calibration_id ||
                saved.at("interpolation") != "linear_joint_coordinates" || saved.at("status") != "exact_solution")
                throw std::invalid_argument("Saved path identity, request or status mismatch");
            if (!std::isfinite(options.timeout_s) || options.timeout_s <= 0 || options.timeout_s > 300 ||
                options.max_queries < 1 || options.max_queries > 1000000)
                throw std::invalid_argument("Invalid replay validation budget");
            const auto& path = saved.at("waypoints");
            if (!path.is_array() || path.empty() || path.size() > 10000) throw std::invalid_argument("Invalid saved waypoint array");
            for (const auto& q : path) plan.waypoints.push_back(Joints(q));
            if (plan.waypoints.front() != start || plan.waypoints.back() != goal)
                throw std::invalid_argument("Saved path endpoints differ from request");
            plan.model_id = checker.ModelId(); plan.scene_id = scene.id; plan.calibration_id = scene.calibration_id;
            plan.captured_at_ns = scene.captured_at_ns; plan.checked_at_ns = now; plan.seed = options.seed;
            const auto begin = std::chrono::steady_clock::now();
            auto stopped = [&] {
                return interrupted || std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() >= options.timeout_s;
            };
            bool valid = true;
            for (std::size_t i = 0; valid && i < plan.waypoints.size(); ++i) {
                if (stopped() || plan.state_queries + plan.segment_queries >= options.max_queries) { valid = false; break; }
                ++plan.state_queries;
                const auto state = checker.CheckState(plan.waypoints[i], scene, now, options.collision);
                if (i == 0) plan.start_check = state;
                if (i + 1 == plan.waypoints.size()) plan.goal_check = state;
                valid = state.Valid();
                if (!valid) plan.validation_failure = state;
                if (valid && i) {
                    if (plan.state_queries + plan.segment_queries >= options.max_queries) { valid = false; break; }
                    ++plan.segment_queries;
                    const auto segment = checker.CheckSegment(plan.waypoints[i - 1], plan.waypoints[i], scene, now, options.collision, stopped);
                    valid = segment.Valid();
                    if (!valid) plan.last_segment_failure = segment;
                }
            }
            valid = valid && !stopped();
            plan.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
            plan.status = valid ? "exact_solution" : "path_validation_failed";
            if (!valid && interrupted) plan.status = "cancelled";
            else if (!valid && plan.elapsed_ms >= options.timeout_s * 1000) plan.status = "timeout";
            else if (!valid && plan.state_queries + plan.segment_queries >= options.max_queries) plan.status = "budget_exceeded";
            plan.detail = valid ? "Saved path fully rechecked without rerunning OMPL" : "Saved path failed bounded revalidation";
            if (!valid) plan.waypoints.clear();
        }
        Json report = {{"schema_version", 1}, {"mode", saved_path.empty() ? "offline_joint_planning" : "offline_joint_path_replay"}, {"ompl_version", OMPL_VERSION},
            {"robot_motion_enabled", false}, {"ready_for_execution", false}, {"trajectory_time_parameterized", false},
            {"unknown_space_allowed", plan.unknown_space_allowed},
            {"status", plan.status}, {"detail", plan.detail}, {"exact", plan.Exact()},
            {"model_id", plan.model_id}, {"scene_id", plan.scene_id}, {"calibration_id", plan.calibration_id},
            {"request_sha256", request_sha}, {"urdf_sha256", model.Sha256()},
            {"captured_at_ns", plan.captured_at_ns}, {"checked_at_ns", plan.checked_at_ns}, {"clock_mode", "offline_snapshot"},
            {"seed", plan.seed}, {"elapsed_ms", plan.elapsed_ms}, {"state_queries", plan.state_queries},
            {"segment_queries", plan.segment_queries}, {"start_check", Diagnostic(plan.start_check)},
            {"segment_state_checks", plan.segment_state_checks},
            {"goal_check", Diagnostic(plan.goal_check)}, {"joint_order", model.ActiveJointNames()},
            {"waypoints", plan.waypoints}, {"interpolation", "linear_joint_coordinates"},
            {"collision_options", {{"clearance_m", options.collision.clearance_m},
                {"numerical_guard_m", options.collision.numerical_guard_m},
                {"max_scene_age_ns", options.collision.max_scene_age_ns},
                {"max_state_checks", options.collision.max_state_checks},
                {"max_subdivision_depth", options.collision.max_subdivision_depth}}},
            {"planning_options", {{"allow_unknown_space", options.collision.allow_unknown_space},
                {"timeout_s", options.timeout_s}, {"range", options.range},
                {"max_queries", options.max_queries}, {"shortcut_attempts", options.shortcut_attempts}}},
            {"self_collision_exclusions", request.value("self_collision_exclusions", Json::array())}};
        if (!plan.last_segment_failure.model_id.empty()) report["last_segment_failure"] = {
            {"status", c::CheckStatusName(plan.last_segment_failure.status)},
            {"detail", plan.last_segment_failure.detail}, {"failure", Diagnostic(plan.last_segment_failure.failure)}};
        if (!plan.validation_failure.model_id.empty()) report["path_validation_failure"] = Diagnostic(plan.validation_failure);
        const auto bytes = report.dump(2) + "\n";
        if (!output.empty()) WriteNew(output, bytes);
        std::cout << bytes;
#ifdef COLLISION_HAS_VIEWER
        if (preview && plan.Exact()) {
            MonitorViewer viewer(model, directories, meshes.discard_zero_area_triangles);
            viewer.SetPlannedPath(plan.waypoints);
            const auto begin = std::chrono::steady_clock::now();
            while (!interrupted) {
                const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
                const double cursor = std::fmod(seconds * 2, double(std::max<std::size_t>(1, plan.waypoints.size() - 1)));
                const auto index = std::min<std::size_t>(cursor, plan.waypoints.size() - 1);
                const auto next = std::min(index + 1, plan.waypoints.size() - 1);
                c::MonitorFrame frame; frame.robot_valid = true;
                for (const auto& [name, q] : plan.waypoints[index])
                    frame.joints[name] = q + (cursor - index) * (plan.waypoints[next].at(name) - q);
                c::MonitorResult result; result.input_valid = true; result.scene = scene;
                result.check = checker.CheckState(frame.joints, scene, now, options.collision);
                if (!viewer.Draw(frame, result)) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
        }
#endif
        return plan.Exact() ? 0 : 2;
    } catch (const std::exception& e) { std::cerr << "collision_plan: " << e.what() << '\n'; return 1; }
}
