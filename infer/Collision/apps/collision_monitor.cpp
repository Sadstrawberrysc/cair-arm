#include "collision/monitor.hpp"
#include "unknown_space_json.hpp"
#include "collision/robot_read_only.hpp"
#ifdef COLLISION_HAS_PLANNING
#include "collision/joint_planner.hpp"
#include "coarse_target.hpp"
#include <ompl/util/Console.h>
#endif
#ifdef COLLISION_HAS_REALSENSE
#include "collision/live_source.hpp"
#include <librealsense2/rs.h>
#endif
#ifdef COLLISION_HAS_VIEWER
#include "monitor_viewer.hpp"
#endif
#include <cmath>
#include <algorithm>
#include <array>
#include <csignal>
#include <fstream>
#include <iostream>
#include <json.hpp>
#include <set>
#include <thread>

namespace c = uspilot::collision;
using Json = nlohmann::json;
namespace {
volatile std::sig_atomic_t stopped = 0;
void Signal(int) { stopped = 1; }
void Keys(const Json &j, const std::set<std::string> &allowed) {
    if (!j.is_object())
        throw std::invalid_argument("Expected JSON object");
    for (auto i = j.begin(); i != j.end(); ++i)
        if (!allowed.count(i.key()))
            throw std::invalid_argument("Unknown monitor field: " + i.key());
}
double Number(const Json &j) {
    if (!j.is_number())
        throw std::invalid_argument("Expected number");
    double x = j.get<double>();
    if (!std::isfinite(x))
        throw std::invalid_argument("Nonfinite number");
    return x;
}
std::int64_t Time(const Json &j) {
    if (!j.is_number_integer() || Number(j) < 0 || Number(j) >= 9e18)
        throw std::invalid_argument("Invalid integer time/count");
    return j.get<std::int64_t>();
}
Eigen::Vector3d Vector(const Json &j) {
    if (!j.is_array() || j.size() != 3)
        throw std::invalid_argument("Expected three-vector");
    return {Number(j[0]), Number(j[1]), Number(j[2])};
}
Json Vec(const Eigen::Vector3d &v) { return Json::array({v.x(), v.y(), v.z()}); }
Json Finite(double x) { return std::isfinite(x) ? Json(x) : Json(nullptr); }
Json Load(const std::filesystem::path &path) {
    if (std::filesystem::file_size(path) > 4 * 1024 * 1024)
        throw std::invalid_argument("Configuration too large");
    std::ifstream input(path);
    return Json::parse(input);
}
c::MonitorFrame ParseFrame(const Json &j, std::size_t limit) {
    Keys(j, {"sequence", "camera_serial", "captured_at_ns", "received_at_ns", "robot_at_ns",
             "capture_time_trusted", "robot_valid", "joint_positions_rad", "ray_origin_color_m",
             "points_color_m", "now_ns"});
    c::MonitorFrame f;
    f.sequence = Time(j.at("sequence"));
    f.camera_serial = j.at("camera_serial").get<std::string>();
    f.captured_at_ns = Time(j.at("captured_at_ns"));
    f.received_at_ns = Time(j.at("received_at_ns"));
    f.robot_at_ns = Time(j.at("robot_at_ns"));
    f.capture_time_trusted = j.at("capture_time_trusted").get<bool>();
    f.robot_valid = j.at("robot_valid").get<bool>();
    const auto &joints = j.at("joint_positions_rad");
    if (!joints.is_object())
        throw std::invalid_argument("Expected joint map");
    for (auto i = joints.begin(); i != joints.end(); ++i)
        f.joints[i.key()] = Number(i.value());
    f.ray_origin_color_m = Vector(j.at("ray_origin_color_m"));
    const auto &points = j.at("points_color_m");
    if (!points.is_array() || points.size() > limit)
        throw std::invalid_argument("Replay point budget exceeded");
    for (const auto &p : points)
        f.points_color_m.push_back(Vector(p));
    return f;
}
Json FrameJson(const c::MonitorFrame &f, std::int64_t now) {
    Json points = Json::array();
    for (const auto &p : f.points_color_m)
        points.push_back(Vec(p));
    return {{"sequence", f.sequence},
            {"camera_serial", f.camera_serial},
            {"captured_at_ns", f.captured_at_ns},
            {"received_at_ns", f.received_at_ns},
            {"robot_at_ns", f.robot_at_ns},
            {"capture_time_trusted", f.capture_time_trusted},
            {"robot_valid", f.robot_valid},
            {"joint_positions_rad", f.joints},
            {"ray_origin_color_m", Vec(f.ray_origin_color_m)},
            {"points_color_m", points},
            {"now_ns", now}};
}
Json UnknownSpacePoints(const c::StateCheck& check) {
    Json points = Json::array();
    for (const auto& p : check.unknown_space_points_base_m)
        points.push_back(p.allFinite() ? Json::array({p.x(), p.y(), p.z()}) : Json(nullptr));
    return points;
}
Json CompactFailure(const c::StateCheck& check) {
    if (!check.violations.empty()) {
        const auto pair = std::min_element(check.violations.begin(), check.violations.end(),
            [](const auto& a, const auto& b) { return a.distance_m < b.distance_m; });
        return {{"body", pair->first_body}, {"other_body", pair->second_body},
            {"distance_mm", std::round(pair->distance_m * 100000) / 100}};
    }
    if (!check.unknown_space_diagnostics.empty()) {
        const auto& reason = check.unknown_space_diagnostics.front();
        return {{"body", reason.body}, {"reason", reason.reason}};
    }
    return nullptr;
}
#ifdef COLLISION_HAS_PLANNING
Json StateDiagnostic(const c::StateCheck& check) {
    Json pairs = Json::array();
    for (const auto& p : check.violations)
        pairs.push_back({{"first_body", p.first_body}, {"second_body", p.second_body},
            {"self_collision", p.self_collision}, {"distance_m", p.distance_m},
            {"robot_point_base_m", p.first_point_base_m ? Vec(*p.first_point_base_m) : Json(nullptr)},
            {"environment_point_base_m", p.second_point_base_m ? Vec(*p.second_point_base_m) : Json(nullptr)}});
    return {{"status", check.detail == "skipped_start_precheck" ? "skipped" :
        (check.detail == "candidate_not_checked" ? "not_checked" : c::CheckStatusName(check.status))}, {"detail", check.detail}, {"violations", pairs},
        {"unknown_space_bodies", check.unknown_space_bodies},
        {"unknown_space_diagnostics", c::app::UnknownSpaceDiagnostics(check)},
        {"unknown_space_points_base_m", UnknownSpacePoints(check)}};
}
c::PlanResult PlanCameraFrame(const c::RobotModel& model, const c::CollisionChecker& checker,
    const c::MonitorFrame& frame, const c::MonitorResult& observation, const c::MonitorOptions& monitor,
    const std::filesystem::path& goal_path, const c::CameraCalibration& calibration,
    bool live, std::int64_t replay_now, Json& target_report) {
    c::PlanResult result;
    result.model_id = checker.ModelId(); result.scene_id = observation.check.scene_id;
    result.calibration_id = observation.check.calibration_id; result.captured_at_ns = frame.captured_at_ns;
    result.checked_at_ns = live ? c::MonitorSteadyNowNs() : replay_now;
    try {
        if (!observation.input_valid) {
            result.detail = observation.check.detail;
            return result;
        }
        const auto goal_file = Load(goal_path);
        Keys(goal_file, {"schema_version", "goal_joint_positions_rad", "coarse_target", "planning_options"});
        if (goal_file.at("schema_version") != 1) throw std::invalid_argument("Unknown goal schema");
        const bool coarse = goal_file.contains("coarse_target");
        if (coarse == goal_file.contains("goal_joint_positions_rad"))
            throw std::invalid_argument("Specify exactly one of coarse_target and goal_joint_positions_rad");
        c::JointPositions goal;
        if (!coarse) {
            const auto& values = goal_file.at("goal_joint_positions_rad");
            if (!values.is_object()) throw std::invalid_argument("Expected named goal joints in rad");
            for (auto it = values.begin(); it != values.end(); ++it) goal[it.key()] = Number(it.value());
        }
        c::PlannerOptions options;
        options.collision = monitor.collision; // Same clearance/exclusions and actual camera observation.
        if (goal_file.contains("planning_options")) {
            const auto& p = goal_file.at("planning_options");
            Keys(p, {"timeout_s", "range", "max_queries", "shortcut_attempts", "seed", "skip_start_precheck", "allow_unknown_space"});
            if (p.contains("allow_unknown_space")) options.collision.allow_unknown_space = p.at("allow_unknown_space").get<bool>();
            if (p.contains("skip_start_precheck")) options.skip_start_precheck = p.at("skip_start_precheck").get<bool>();
            if (p.contains("timeout_s")) options.timeout_s = Number(p.at("timeout_s"));
            if (p.contains("range")) options.range = Number(p.at("range"));
            if (p.contains("max_queries")) options.max_queries = Time(p.at("max_queries"));
            if (p.contains("shortcut_attempts")) options.shortcut_attempts = Time(p.at("shortcut_attempts"));
            if (p.contains("seed")) {
                const auto seed = Time(p.at("seed"));
                if (seed > UINT32_MAX) throw std::invalid_argument("Seed exceeds uint32");
                options.seed = seed;
            }
        }
        result.unknown_space_allowed = options.collision.allow_unknown_space;
        const auto now = live ? c::MonitorSteadyNowNs() : replay_now;
        result.checked_at_ns = now;
        const auto remaining = monitor.max_frame_age_ns - (now - frame.captured_at_ns);
        if (remaining < 0 || (live && remaining == 0)) {
            result.status = "stale_scene"; result.detail = "Camera snapshot expired before planning"; return result;
        }
        // Offline replay uses its recorded clock. Live work must fit the
        // snapshot's remaining lifetime and never refresh the capture timestamp.
        if (live) options.timeout_s = std::min(options.timeout_s, remaining / 1e9);
        auto expired = [&] { return live && c::MonitorSteadyNowNs() - frame.captured_at_ns > monitor.max_frame_age_ns; };
        if (!coarse) {
            result = c::PlanJointPath(model, checker, frame.joints, goal, observation.scene, now, options,
                [&] { return stopped != 0 || expired(); });
        } else {
            const auto prepared = c::app::LoadCoarseTarget(goal_file.at("coarse_target"), goal_path, calibration);
            target_report = prepared.metadata;
            if (monitor.robot_occlusion_filter) {
                const Eigen::Vector3d camera_origin = calibration.base_from_color * frame.ray_origin_color_m;
                if (checker.PointsOccludedByRobot(frame.joints, camera_origin,
                        {prepared.surface_base_m}, monitor.mask_padding_m).front()) {
                    result.status = "target_occluded";
                    result.detail = "Robot geometry occludes the carotid surface target from the depth camera";
                    return result;
                }
            }
            std::array<double, 4> nearest;
            nearest.fill(std::numeric_limits<double>::infinity());
            Eigen::Vector3d nearest_kept_point = Eigen::Vector3d::Zero();
            for (std::size_t i = 0; i < observation.points_base_m.size(); ++i) {
                const double distance = (observation.points_base_m[i] - prepared.surface_base_m).norm();
                nearest[0] = std::min(nearest[0], distance);
                if (i >= observation.robot_mask.size() || observation.robot_mask[i]) continue;
                nearest[1] = std::min(nearest[1], distance);
                if (i < observation.occluded_mask.size() && observation.occluded_mask[i]) continue;
                nearest[2] = std::min(nearest[2], distance);
                if (i < observation.outlier_mask.size() && observation.outlier_mask[i]) continue;
                if (distance < nearest[3]) {
                    nearest[3] = distance;
                    nearest_kept_point = observation.points_base_m[i];
                }
            }
            target_report["surface_observation_error_m"] = Finite(nearest[3]);
            target_report["surface_match_tolerance_m"] = prepared.surface_match_tolerance_m;
            target_report["surface_match_required"] = prepared.surface_match_tolerance_m > 0;
            target_report["surface_observation_stages_m"] = {
                {"valid_depth", Finite(nearest[0])}, {"after_robot_mask", Finite(nearest[1])},
                {"after_occlusion_filter", Finite(nearest[2])}, {"after_outlier_filter", Finite(nearest[3])}};
            target_report["nearest_surface_point_base_m"] = std::isfinite(nearest[3])
                ? Vec(nearest_kept_point) : Json(nullptr);
            if (prepared.surface_match_tolerance_m > 0 && nearest[3] > prepared.surface_match_tolerance_m) {
                result.status = "target_not_observed";
                result.detail = "Carotid surface point has no nearby unmasked point in the current camera frame";
                return result;
            }
            auto ik_options = prepared.ik; ik_options.seed = options.seed;
            const auto pose_plan = c::PlanPosePath(model, checker, frame.joints, prepared.tip_link,
                prepared.tip_from_tcp, prepared.base_from_tcp, observation.scene, now, options, ik_options,
                [&] { return stopped != 0 || expired() || prepared.Expired(); });
            result = pose_plan.plan;
            target_report["ik_attempts"] = pose_plan.ik.attempts;
            target_report["ik_iterations"] = pose_plan.ik.iterations;
            target_report["ik_candidates_rad"] = pose_plan.ik.candidates;
            target_report["orientation_sample_count"] = prepared.ik.yaw_offsets_rad.size() * prepared.ik.tilt_offsets_rad.size();
            target_report["max_normal_deviation_deg"] = prepared.ik.max_normal_deviation_rad * 180 / std::acos(-1.);
            target_report["ik_best_position_error_m"] = Finite(pose_plan.ik.best_position_error_m);
            target_report["ik_best_rotation_error_rad"] = Finite(pose_plan.ik.best_rotation_error_rad);
            target_report["candidate_checks"] = Json::array();
            for (const auto& check : pose_plan.candidate_checks) target_report["candidate_checks"].push_back(StateDiagnostic(check));
            target_report["candidate_searches"] = Json::array();
            for (const auto& candidate : pose_plan.candidate_searches)
                target_report["candidate_searches"].push_back({{"candidate_index", candidate.candidate_index},
                    {"status", candidate.status}, {"detail", candidate.detail},
                    {"yaw_offset_deg", candidate.yaw_offset_rad * 180 / std::acos(-1.)},
                    {"normal_deviation_deg", candidate.normal_deviation_rad * 180 / std::acos(-1.)},
                    {"budget_s", candidate.budget_s}, {"elapsed_ms", candidate.elapsed_ms},
                    {"waypoint_count", candidate.waypoint_count}, {"joint_path_length_rad",
                        candidate.joint_path_length_rad ? Json(*candidate.joint_path_length_rad) : Json(nullptr)}});
            if (pose_plan.preview_candidate_index) {
                target_report["preview_candidate_index"] = *pose_plan.preview_candidate_index;
                const auto selected = *pose_plan.preview_candidate_index;
                target_report["T_base_tcp_nominal"] = target_report["T_base_tcp_target"];
                target_report["T_base_tcp_target"] = c::app::TargetMatrixJson(pose_plan.ik.candidate_targets.at(selected));
                target_report["selected_yaw_offset_deg"] = pose_plan.ik.candidate_yaw_offsets_rad.at(selected) * 180 / std::acos(-1.);
                target_report["selected_normal_deviation_deg"] = pose_plan.ik.candidate_normal_deviation_rad.at(selected) * 180 / std::acos(-1.);
            }
            if (prepared.Expired()) {
                result.status = "stale_target"; result.detail = "Carotid target expired during planning"; result.waypoints.clear();
            }
        }
        if (expired()) {
            result.status = "stale_scene"; result.detail = "Camera snapshot expired during planning";
            result.waypoints.clear();
        }
    } catch (const c::app::TargetError& e) { result.status = e.status; result.detail = e.what(); result.waypoints.clear(); }
      catch (const std::exception& e) { result.status = "invalid_input"; result.detail = e.what(); result.waypoints.clear(); }
    return result;
}
#endif
} // namespace
int main(int argc, char **argv) {
    try {
        std::filesystem::path urdf, calibration_path, config_path, replay_path, record_path, goal_path;
        std::string robot_ip;
        int robot_port = 8080, stride = 8;
        bool live = false, headless = false, drop = false, plan_once = false, verbose = false;
        std::string last_monitor_message;
        std::size_t max_frames = 0;
        c::MeshDirectories directories;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--sdk-info") {
#ifdef COLLISION_HAS_REALSENSE
                rs2_error *error = nullptr;
                const int version = rs2_get_api_version(&error);
                if (error) {
                    const std::string message = rs2_get_error_message(error);
                    rs2_free_error(error);
                    throw std::runtime_error(message);
                }
                std::cout << Json({{"sdk_header_version", RS2_API_VERSION_STR},
                                   {"sdk_runtime_api", version},
                                   {"robot_motion_enabled", false}})
                                 .dump()
                          << '\n';
                return 0;
#else
                throw std::runtime_error("Live RealSense SDK not linked in this build");
#endif
            }
            if (arg == "--help") {
                std::cout
                    << "Read-only collision monitor; never sends motion or stop commands.\n"
                       "collision_monitor --urdf FILE --calibration FILE --config FILE [--mesh-dir DIR]\n"
                       "  (--replay JSONL | --live --robot-ip IP [--robot-port 8080])\n"
                       "  [--headless] [--max-frames N] [--pixel-stride 8] [--record NEW_JSONL]\n"
                       "  [--drop-zero-area-triangles] [--sdk-info]\n"
                       "  [--goal JSON] [--plan-once] [--verbose] (planning requires planning build)\n"
                       "Default output: monitor status changes and compact planning results.\n"
                       "--verbose: print every frame and full planning JSON, including paths and IK candidates.\n"
                       "With --goal: press P to plan using the current camera snapshot.\n"
                       "Goal JSON accepts joint angles or coarse_target (carotid file + calibrated probe TCP).\n"
                       "--plan-once: plan on the first valid frame, then exit; requires --goal.\n"
                       "Replay performs no device/network access. Live is standalone state-only "
                       "diagnostics.\n"
                       "Viewer: left drag orbit, wheel zoom, Esc exit; red environment risk, amber self "
                       "pairs, grey robot points, orange density outliers.\n"
                       "Exit 0: input frames processed (not a clearance approval); 1: source/input failure; "
                       "2: all frames invalid.\n";
                return 0;
            }
            if (arg == "--live") {
                live = true;
                continue;
            }
            if (arg == "--headless") {
                headless = true;
                continue;
            }
            if (arg == "--plan-once") { plan_once = true; continue; }
            if (arg == "--verbose") { verbose = true; continue; }
            if (arg == "--drop-zero-area-triangles") {
                drop = true;
                continue;
            }
            if (arg != "--urdf" && arg != "--calibration" && arg != "--config" && arg != "--replay" &&
                arg != "--record" && arg != "--mesh-dir" && arg != "--robot-ip" && arg != "--robot-port" &&
                arg != "--max-frames" && arg != "--pixel-stride" && arg != "--goal")
                throw std::invalid_argument("Unknown option: " + arg);
            if (++i >= argc)
                throw std::invalid_argument("Missing option value");
            const std::string value = argv[i];
            if (arg == "--urdf")
                urdf = value;
            else if (arg == "--calibration")
                calibration_path = value;
            else if (arg == "--config")
                config_path = value;
            else if (arg == "--replay")
                replay_path = value;
            else if (arg == "--record")
                record_path = value;
            else if (arg == "--goal")
                goal_path = value;
            else if (arg == "--mesh-dir")
                directories["package://rm_description/meshes/rm_75_arm/"] = std::filesystem::canonical(value);
            else if (arg == "--robot-ip")
                robot_ip = value;
            else {
                std::size_t consumed = 0;
                const auto n = std::stoll(value, &consumed);
                if (consumed != value.size() || n < 0 || n > 1000000000)
                    throw std::invalid_argument("Invalid integer option");
                if (arg == "--robot-port") {
                    if (n < 1 || n > 65535)
                        throw std::invalid_argument("Invalid robot port");
                    robot_port = int(n);
                } else if (arg == "--pixel-stride")
                    stride = int(n);
                else
                    max_frames = std::size_t(n);
            }
        }
        if (urdf.empty() || calibration_path.empty() || config_path.empty() || live == !replay_path.empty())
            throw std::invalid_argument(
                "Specify URDF, calibration, config and exactly one of --live/--replay");
        if (live && robot_ip.empty())
            throw std::invalid_argument("Live mode requires explicit --robot-ip");
        if (!live && !robot_ip.empty())
            throw std::invalid_argument("Robot address is not accepted in replay mode");
        if (plan_once && goal_path.empty()) throw std::invalid_argument("--plan-once requires --goal JSON");
        if (!goal_path.empty() && !std::filesystem::is_regular_file(goal_path))
            throw std::invalid_argument("Goal file missing: " + goal_path.string());
#ifndef COLLISION_HAS_PLANNING
        if (!goal_path.empty()) throw std::invalid_argument("Enable BUILD_COLLISION_PLANNING to use --goal");
#else
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
#endif
        if (plan_once && !max_frames) max_frames = 30; // Bounded startup wait for trustworthy feedback/time.
#ifndef COLLISION_HAS_REALSENSE
        if (live)
            throw std::runtime_error(
                "Live source not compiled; enable BUILD_REALSENSE_SOURCE with the native C++ SDK");
#endif
        const auto calibration = c::LoadCameraCalibration(calibration_path);
        const auto config = Load(config_path);
        Keys(config,
             {"schema_version", "region_min_m", "region_max_m", "voxel_m", "min_depth_m", "max_depth_m",
              "mask_padding_m", "robot_occlusion_filter", "outlier_radius_m", "outlier_min_neighbors", "endpoint_padding_voxels", "max_points", "max_voxels",
              "max_feedback_offset_ns", "max_frame_age_ns", "clearance_m", "self_collision_exclusions"});
        if (config.at("schema_version") != 1)
            throw std::invalid_argument("Unknown monitor schema");
        c::MonitorOptions options;
        options.region = {Vector(config.at("region_min_m")), Vector(config.at("region_max_m"))};
        if (config.contains("voxel_m"))
            options.voxel_m = Number(config.at("voxel_m"));
        if (config.contains("min_depth_m"))
            options.min_depth_m = Number(config.at("min_depth_m"));
        if (config.contains("max_depth_m"))
            options.max_depth_m = Number(config.at("max_depth_m"));
        if (config.contains("mask_padding_m"))
            options.mask_padding_m = Number(config.at("mask_padding_m"));
        if (config.contains("robot_occlusion_filter"))
            options.robot_occlusion_filter = config.at("robot_occlusion_filter").get<bool>();
        if (config.contains("outlier_radius_m"))
            options.outlier_radius_m = Number(config.at("outlier_radius_m"));
        if (config.contains("outlier_min_neighbors"))
            options.outlier_min_neighbors = Time(config.at("outlier_min_neighbors"));
        if (config.contains("clearance_m"))
            options.collision.clearance_m = Number(config.at("clearance_m"));
        if (config.contains("max_points"))
            options.max_points = Time(config.at("max_points"));
        if (config.contains("max_voxels"))
            options.max_voxels = Time(config.at("max_voxels"));
        if (config.contains("endpoint_padding_voxels"))
            options.endpoint_padding_voxels = Time(config.at("endpoint_padding_voxels"));
        if (config.contains("max_feedback_offset_ns"))
            options.max_feedback_offset_ns = Time(config.at("max_feedback_offset_ns"));
        if (config.contains("max_frame_age_ns"))
            options.max_frame_age_ns = Time(config.at("max_frame_age_ns"));
        options.collision.max_scene_age_ns = options.max_frame_age_ns;
        std::vector<c::SelfCollisionExclusion> exclusions;
        if (config.contains("self_collision_exclusions")) {
            if (!config.at("self_collision_exclusions").is_array())
                throw std::invalid_argument("Expected exclusions array");
            for (const auto &e : config.at("self_collision_exclusions")) {
                Keys(e, {"first_link", "second_link", "reason"});
                exclusions.push_back({e.at("first_link"), e.at("second_link"), e.at("reason")});
            }
        }
        auto model = c::RobotModel::Load(urdf);
        c::CollisionChecker checker(model, {}, directories, {drop}, exclusions);
        std::ifstream replay;
        if (!live) {
            if (std::filesystem::file_size(replay_path) > 512 * 1024 * 1024)
                throw std::invalid_argument("Replay exceeds 512 MiB");
            replay.open(replay_path);
            if (!replay)
                throw std::runtime_error("Cannot open replay");
        }
        std::ofstream record;
        if (!record_path.empty()) {
            if (std::filesystem::exists(record_path))
                throw std::invalid_argument("Record path exists; choose a new file");
            record.open(record_path);
            if (!record)
                throw std::runtime_error("Cannot create record file");
        }
#ifdef COLLISION_HAS_VIEWER
        std::unique_ptr<MonitorViewer> viewer;
        if (!headless)
            viewer = std::make_unique<MonitorViewer>(model, directories, drop);
        if (viewer && !goal_path.empty()) viewer->SetPlanningStatus("P: plan camera snapshot");
#else
        if (!headless)
            throw std::runtime_error("Viewer not compiled; use --headless");
#endif
#ifdef COLLISION_HAS_REALSENSE
        std::unique_ptr<c::LiveMonitorSource> source;
        if (live)
            source = std::make_unique<c::LiveMonitorSource>(
                c::LiveSourceOptions{robot_ip, robot_port, calibration.serial, stride});
#endif
        std::signal(SIGINT, Signal);
        std::signal(SIGTERM, Signal);
        std::size_t processed = 0, valid_inputs = 0;
        std::string last_input_status, last_input_detail;
#ifdef COLLISION_HAS_VIEWER
        std::int64_t displayed_plan_capture = -1;
#endif
        while (!stopped && (!max_frames || processed < max_frames)) {
            c::MonitorFrame frame;
            std::int64_t now;
            if (!live) {
                std::string line;
                if (!std::getline(replay, line))
                    break;
                if (line.empty())
                    continue;
                if (line.size() > 8 * 1024 * 1024)
                    throw std::invalid_argument("Replay frame exceeds 8 MiB");
                const auto j = Json::parse(line);
                frame = ParseFrame(j, options.max_points);
                now = Time(j.at("now_ns"));
            }
#ifdef COLLISION_HAS_REALSENSE
            else {
                frame = source->Next();
                now = c::MonitorSteadyNowNs();
            }
#endif
            const auto begin = c::MonitorSteadyNowNs();
            auto result = c::ProcessMonitorFrame(checker, calibration, frame, now, options);
            const auto finished = c::MonitorSteadyNowNs();
            if (live && finished - frame.captured_at_ns > options.max_frame_age_ns) {
                result.check.status = c::CheckStatus::kStaleScene;
                result.check.detail = "camera_frame_expired_during_processing";
                result.input_valid = false;
            }
            ++processed;
            last_input_status = c::CheckStatusName(result.check.status);
            last_input_detail = result.check.detail;
            if (result.input_valid)
                ++valid_inputs;
            Json violations = Json::array();
            for (const auto &p : result.check.violations)
                violations.push_back({{"robot_body", p.first_body},
                                      {"other_body", p.second_body},
                                      {"self_collision", p.self_collision},
                                      {"intersecting", p.intersecting},
                                      {"distance_m", p.distance_m}});
            Json output = {
                {"mode", live ? "live_read_only_monitor" : "replay_read_only_monitor"},
                {"robot_motion_enabled", false},
                {"ready_for_motion_planning", false},
                {"sequence", frame.sequence},
                {"model_id", checker.ModelId()},
                {"calibration_id", calibration.sha256},
                {"camera_serial", frame.camera_serial},
                {"scene_id", result.check.scene_id},
                {"captured_at_ns", frame.captured_at_ns},
                {"robot_at_ns", frame.robot_at_ns},
                {"input_valid", result.input_valid},
                {"status", c::CheckStatusName(result.check.status)},
                {"detail", result.check.detail},
                {"environment_distance_m",
                 result.input_valid ? Finite(result.check.minimum_environment_distance_m) : Json(nullptr)},
                {"self_distance_m",
                 result.input_valid ? Finite(result.check.minimum_self_distance_m) : Json(nullptr)},
                {"points", result.points_base_m.size()},
                {"masked_points", result.masked_points},
                {"robot_occlusion_filter", options.robot_occlusion_filter},
                {"occluded_points", result.occluded_points},
                {"occluded_voxels", result.occluded_voxels},
                {"outlier_points", result.outlier_points},
                {"outlier_radius_m", options.outlier_radius_m},
                {"outlier_min_neighbors", options.outlier_min_neighbors},
                {"invalid_points", result.invalid_points},
                {"occupied_voxels", result.occupied_voxels},
                {"processing_ms", (finished - begin) / 1e6},
                {"unknown_space_bodies", result.check.unknown_space_bodies},
                {"unknown_space_diagnostics", c::app::UnknownSpaceDiagnostics(result.check)},
                {"unknown_space_points_base_m", UnknownSpacePoints(result.check)},
                {"violations", violations}};
            if (verbose) std::cout << output.dump() << std::endl;
            else if ((goal_path.empty() || !result.input_valid) &&
                (result.check.detail != "capture_time_untrusted" || processed >= 30)) {
                Json risk_bodies = Json::array();
                for (const auto& p : result.check.violations)
                    risk_bodies.push_back({p.first_body, p.second_body});
                const std::string message = std::string(c::CheckStatusName(result.check.status)) + ":" +
                    result.check.detail + risk_bodies.dump() + Json(result.check.unknown_space_bodies).dump();
                if (message != last_monitor_message) {
                    Json message_json = {{"event", "monitor_status"},
                        {"status", c::CheckStatusName(result.check.status)}};
                    if (result.check.detail != c::CheckStatusName(result.check.status)) message_json["detail"] = result.check.detail;
                    const auto failure = CompactFailure(result.check);
                    if (!failure.is_null()) message_json["failure"] = failure;
                    std::cout << message_json.dump() << std::endl;
                    last_monitor_message = message;
                }
            } else last_monitor_message.clear();
            if (record.is_open()) {
                record << FrameJson(frame, now).dump() << '\n';
                if (!record)
                    throw std::runtime_error("Record write failed");
            }
#ifdef COLLISION_HAS_VIEWER
            if (viewer && displayed_plan_capture >= 0 &&
                (live ? c::MonitorSteadyNowNs() : now) - displayed_plan_capture > options.max_frame_age_ns) {
                viewer->MarkPlannedPathStale(); viewer->SetPlanningStatus("stale_scene; historical curve retained; P: replan");
                displayed_plan_capture = -1;
            }
            if (viewer && !viewer->Draw(frame, result))
                break;
#endif
#ifdef COLLISION_HAS_PLANNING
            bool requested = plan_once && result.input_valid;
#ifdef COLLISION_HAS_VIEWER
            if (viewer && viewer->TakePlanningRequest() && !goal_path.empty()) requested = true;
#endif
            if (requested) {
                Json target_report;
                const auto plan = PlanCameraFrame(model, checker, frame, result, options, goal_path, calibration, live, now, target_report);
                const auto completed = live ? c::MonitorSteadyNowNs() : now;
                Json report = {{"event", "camera_plan"}, {"mode", live ? "live_camera_joint_planning" : "replay_camera_joint_planning"},
                    {"point_filter", {{"outlier_radius_m", options.outlier_radius_m},
                        {"outlier_min_neighbors", options.outlier_min_neighbors}, {"outlier_points", result.outlier_points},
                        {"robot_occlusion_filter", options.robot_occlusion_filter},
                        {"occluded_points", result.occluded_points}, {"occluded_voxels", result.occluded_voxels}}},
                    {"robot_motion_enabled", false}, {"ready_for_execution", false},
                    {"unknown_space_allowed", plan.unknown_space_allowed},
                    {"status", plan.status}, {"detail", plan.detail}, {"exact", plan.Exact()},
                    {"model_id", plan.model_id}, {"scene_id", plan.scene_id}, {"calibration_id", plan.calibration_id},
                    {"sequence", frame.sequence}, {"captured_at_ns", frame.captured_at_ns}, {"robot_at_ns", frame.robot_at_ns},
                    {"checked_at_ns", plan.checked_at_ns}, {"completed_at_ns", completed},
                    {"clock_mode", live ? "live_snapshot" : "recorded_snapshot"}, {"seed", plan.seed},
                    {"start_joint_positions_rad", frame.joints}, {"joint_order", model.ActiveJointNames()},
                    {"elapsed_ms", plan.elapsed_ms}, {"state_queries", plan.state_queries}, {"segment_queries", plan.segment_queries},
                    {"segment_state_checks", plan.segment_state_checks},
                    {"start_check", StateDiagnostic(plan.start_check)}, {"goal_check", StateDiagnostic(plan.goal_check)},
                    {"waypoints", plan.waypoints}, {"interpolation", "linear_joint_coordinates"}};
                if (!target_report.is_null()) report["target"] = target_report;
                if (!plan.last_segment_failure.model_id.empty()) report["last_segment_failure"] = {
                    {"status", c::CheckStatusName(plan.last_segment_failure.status)}, {"detail", plan.last_segment_failure.detail}};
                if (verbose) std::cout << report.dump() << std::endl;
                else {
                    Json summary = {{"event", "camera_plan"}, {"status", plan.status},
                        {"elapsed_ms", std::round(plan.elapsed_ms * 10) / 10},
                        {"ready_for_execution", false},
                        {"unknown_space_allowed", plan.unknown_space_allowed},
                        {"waypoint_count", plan.waypoints.size()}};
                    if (!plan.Exact()) {
                        summary["detail"] = plan.detail;
                        auto failure = CompactFailure(plan.goal_check);
                        if (failure.is_null()) failure = CompactFailure(plan.validation_failure);
                        if (failure.is_null()) failure = CompactFailure(plan.last_segment_failure.failure);
                        if (failure.is_null()) failure = CompactFailure(plan.start_check);
                        if (!failure.is_null()) summary["failure"] = failure;
                    }
                    std::cout << summary.dump() << std::endl;
                }
#ifdef COLLISION_HAS_VIEWER
                if (viewer) {
                    if (target_report.contains("preview_candidate_index")) {
                        const auto index = target_report.at("preview_candidate_index").get<std::size_t>();
                        const auto q = target_report.at("ik_candidates_rad").at(index).get<c::JointPositions>();
                        const auto& matrix = target_report.at("T_base_tcp_target");
                        const Eigen::Vector3d tcp(Number(matrix.at(0).at(3)), Number(matrix.at(1).at(3)), Number(matrix.at(2).at(3)));
                        viewer->SetGoalPreview(checker.PreviewBodies(q), plan.goal_check, result.scene, frame.joints, tcp);
                    } else viewer->ClearGoalPreview();
                    if (target_report.contains("T_armtip_tcp")) {
                        Eigen::Isometry3d tip_from_tcp = Eigen::Isometry3d::Identity();
                        const auto& matrix = target_report.at("T_armtip_tcp");
                        for (int r = 0; r < 4; ++r)
                            for (int c = 0; c < 4; ++c) tip_from_tcp.matrix()(r,c) = Number(matrix.at(r).at(c));
                        viewer->SetPlannedPath(plan.waypoints, target_report.at("arm_tip_link").get<std::string>(), tip_from_tcp);
                    } else viewer->SetPlannedPath(plan.waypoints);
                    viewer->SetPlanningStatus(plan.status + " (snapshot #" + std::to_string(frame.sequence) + "; P: replan)");
                    displayed_plan_capture = plan.Exact() ? frame.captured_at_ns : -1;
                }
#endif
                if (plan_once) return plan.Exact() ? 0 : 2;
            }
#endif
            if (!live && !headless)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!processed && !stopped)
            throw std::runtime_error("Source has no frames");
        if (plan_once) {
            std::cout << Json({{"event", "camera_plan"},
                {"status", stopped ? "cancelled" : "no_valid_frame"},
                {"detail", last_input_detail.empty() ? "No valid camera frame for planning" : last_input_detail},
                {"input_status", last_input_status}, {"processed_frames", processed},
                {"exact", false}, {"robot_motion_enabled", false}, {"ready_for_execution", false}}).dump() << std::endl;
            return 2;
        }
        return valid_inputs || stopped ? 0 : 2;
    } catch (const std::exception &e) {
        std::cerr << "collision_monitor: " << e.what() << '\n';
        return 1;
    }
}
