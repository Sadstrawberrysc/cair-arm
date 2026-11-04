#pragma once
#include "collision/monitor.hpp"
#include "collision/pose_ik.hpp"
#include <json.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

namespace uspilot::collision::app {
using Json = nlohmann::json;
struct TargetError : std::runtime_error {
    std::string status;
    TargetError(std::string state, const std::string& why) : std::runtime_error(why), status(std::move(state)) {}
};
inline std::int64_t UnixNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
inline void TargetKeys(const Json& j, const std::set<std::string>& allowed) {
    if (!j.is_object()) throw std::invalid_argument("Expected coarse target object");
    for (auto i = j.begin(); i != j.end(); ++i)
        if (!allowed.count(i.key())) throw std::invalid_argument("Unknown coarse target field: " + i.key());
}
inline double TargetNumber(const Json& j) {
    if (!j.is_number() || !std::isfinite(j.get<double>())) throw std::invalid_argument("Expected finite target number");
    return j.get<double>();
}
inline Eigen::Vector3d TargetVector(const Json& j) {
    if (!j.is_array() || j.size() != 3) throw std::invalid_argument("Expected target XYZ vector");
    return {TargetNumber(j[0]), TargetNumber(j[1]), TargetNumber(j[2])};
}
inline Eigen::Matrix3d TargetRotation(const Json& j) {
    if (!j.is_array() || j.size() != 9) throw std::invalid_argument("Expected row-major 3x3 tool rotation");
    Eigen::Matrix3d r;
    for (int i = 0; i < 9; ++i) r(i / 3, i % 3) = TargetNumber(j[i]);
    if ((r.transpose() * r - Eigen::Matrix3d::Identity()).norm() > 1e-6 || std::abs(r.determinant() - 1) > 1e-6)
        throw std::invalid_argument("Invalid calibrated tool rotation");
    return r;
}
inline Json TargetVecJson(const Eigen::Vector3d& v) { return Json::array({v.x(), v.y(), v.z()}); }
inline Json TargetMatrixJson(const Eigen::Isometry3d& t) {
    Json rows = Json::array();
    for (int r = 0; r < 4; ++r) rows.push_back({t(r, 0), t(r, 1), t(r, 2), t(r, 3)});
    return rows;
}
struct CoarseTarget {
    std::string tip_link = "Link7";
    Eigen::Isometry3d tip_from_tcp = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d base_from_tcp = Eigen::Isometry3d::Identity();
    Eigen::Vector3d surface_base_m;
    PoseIkOptions ik;
    double surface_match_tolerance_m = 0; // Zero disables the point-cloud matching gate.
    std::int64_t valid_until_unix_ns = 0;
    Json metadata;
    bool Expired() const { return valid_until_unix_ns > 0 && UnixNowNs() > valid_until_unix_ns; }
};
inline CoarseTarget LoadCoarseTarget(const Json& config, const std::filesystem::path& goal_path,
                                     const CameraCalibration& active) {
    TargetKeys(config, {"artery_path_file", "camera_calibration_file", "tool_calibration_file", "arm_tip_link",
        "max_target_age_s", "standoff_m", "surface_match_tolerance_m", "ik_options"});
    auto resolve = [&](const std::string& key) {
        const std::filesystem::path file = config.at(key).get<std::string>();
        return file.is_absolute() ? file : goal_path.parent_path() / file;
    };
    const auto camera = LoadCameraCalibration(resolve("camera_calibration_file"));
    if (camera.sha256 != active.sha256 || camera.serial != active.serial)
        throw TargetError("target_context_mismatch", "Target camera calibration differs from active scene calibration");
    const auto source = resolve("artery_path_file");
    struct stat before{}, after{};
    if (::stat(source.c_str(), &before) || before.st_size <= 0 || before.st_size > 1024 * 1024)
        throw std::invalid_argument("Missing/oversized artery_path_file; save a new camera snapshot first");
    CoarseTarget result;
    const double max_age = config.contains("max_target_age_s") ? TargetNumber(config.at("max_target_age_s")) : 0;
    const double standoff = config.contains("standoff_m") ? TargetNumber(config.at("standoff_m")) : .20;
    result.surface_match_tolerance_m = config.contains("surface_match_tolerance_m") ? TargetNumber(config.at("surface_match_tolerance_m")) : 0;
    if (max_age < 0 || max_age > 3600 || standoff <= 0 || standoff > .3 ||
        result.surface_match_tolerance_m < 0 || result.surface_match_tolerance_m > .1)
        throw std::invalid_argument("Invalid target age, standoff or surface observation tolerance");
    const auto saved_ns = std::int64_t(before.st_mtim.tv_sec) * 1000000000 + before.st_mtim.tv_nsec;
    if (max_age > 0) {
        result.valid_until_unix_ns = saved_ns + static_cast<std::int64_t>(max_age * 1e9);
        if (saved_ns > UnixNowNs() || result.Expired()) throw TargetError("stale_target", "Carotid file is old/future-dated; acquire and save a new snapshot");
    }
    std::ifstream input(source);
    std::vector<Eigen::Vector3d> rows;
    std::string line;
    while (std::getline(input, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::istringstream stream(line); Eigen::Vector3d v; std::string extra;
        if (!(stream >> v.x() >> v.y() >> v.z()) || (stream >> extra) || !v.allFinite())
            throw std::invalid_argument("Artery path requires three finite values per row");
        rows.push_back(v);
    }
    const auto source_sha = FileSha256(source);
    if (::stat(source.c_str(), &after) || before.st_dev != after.st_dev || before.st_ino != after.st_ino ||
        before.st_size != after.st_size || before.st_mtim.tv_sec != after.st_mtim.tv_sec || before.st_mtim.tv_nsec != after.st_mtim.tv_nsec ||
        before.st_ctim.tv_sec != after.st_ctim.tv_sec || before.st_ctim.tv_nsec != after.st_ctim.tv_nsec)
        throw std::invalid_argument("Artery path changed while loading; save it again");
    if (rows.size() < 3 || rows[0].norm() < 1e-9 || rows[0].z() > 1e-6 || rows[1].z() <= 0)
        throw std::invalid_argument("Expected camera-facing outward normal followed by at least two optical-frame points in m");
    Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
    for (std::size_t i = 2; i < rows.size(); ++i)
        if ((rows[i] - rows[1]).norm() > 1e-6) { tangent = rows[i] - rows[1]; break; }
    if (tangent.norm() < 1e-6) throw std::invalid_argument("Repeated carotid points cannot define the vessel tangent");
    const Eigen::Vector3d z = -(active.base_from_color.linear() * rows[0]).normalized();
    Eigen::Vector3d x = active.base_from_color.linear() * tangent.normalized();
    x -= x.dot(z) * z;
    if (x.norm() < 1e-3) throw std::invalid_argument("Vessel tangent is parallel to surface normal");
    x.normalize(); const Eigen::Vector3d y = z.cross(x).normalized(); x = y.cross(z);
    result.surface_base_m = active.base_from_color * rows[1];
    result.base_from_tcp.linear().col(0) = x; result.base_from_tcp.linear().col(1) = y;
    result.base_from_tcp.linear().col(2) = z;
    result.base_from_tcp.translation() = result.surface_base_m - standoff * z;
    const auto tool_file = resolve("tool_calibration_file");
    if (std::filesystem::file_size(tool_file) > 1024 * 1024) throw std::invalid_argument("Tool calibration too large");
    std::ifstream tool_stream(tool_file); const auto tool = Json::parse(tool_stream);
    if (tool.at("schema_version") != 2 || tool.at("units").at("length") != "m" || !tool.value("tool_chain_verified", false))
        throw std::invalid_argument("Need Robot schema-v2 tool calibration with verified geometric chain and SI units");
    const auto r_tip_sensor = TargetRotation(tool.at("sensor_to_tool").at("rotation_row_major"));
    const Eigen::Matrix3d r_sensor_tool = tool.contains("probe_to_sensor")
        ? TargetRotation(tool.at("probe_to_sensor").at("rotation_row_major")) : Eigen::Matrix3d::Identity();
    result.tip_from_tcp.linear() = r_tip_sensor * r_sensor_tool;
    result.tip_from_tcp.translation() = r_tip_sensor * (TargetVector(tool.at("probe_tcp_sensor_m")) -
        TargetVector(tool.at("sensor_to_tool").at("translation_sensor_to_tool_m")));
    result.tip_link = config.value("arm_tip_link", std::string("Link7"));
    if (config.contains("ik_options")) {
        const auto& ik = config.at("ik_options");
        TargetKeys(ik, {"attempts", "iterations", "timeout_s", "position_tolerance_m", "rotation_tolerance_rad",
            "yaw_offsets_deg", "tilt_offsets_deg", "max_normal_deviation_deg", "max_candidates_per_pose"});
        const double radians_per_degree = std::acos(-1.) / 180;
        if (ik.contains("yaw_offsets_deg")) {
            const auto& samples = ik.at("yaw_offsets_deg");
            if (!samples.is_array() || samples.empty() || samples.size() > 12)
                throw std::invalid_argument("Expected 1..12 yaw offsets");
            result.ik.yaw_offsets_rad.clear();
            for (const auto& sample : samples) result.ik.yaw_offsets_rad.push_back(TargetNumber(sample) * radians_per_degree);
        }
        if (ik.contains("tilt_offsets_deg")) {
            const auto& samples = ik.at("tilt_offsets_deg");
            if (!samples.is_array() || samples.empty() || samples.size() > 5)
                throw std::invalid_argument("Expected 1..5 tilt vectors");
            result.ik.tilt_offsets_rad.clear();
            for (const auto& sample : samples) {
                if (!sample.is_array() || sample.size() != 2) throw std::invalid_argument("Expected XY tilt rotation vector");
                result.ik.tilt_offsets_rad.emplace_back(TargetNumber(sample[0]) * radians_per_degree,
                    TargetNumber(sample[1]) * radians_per_degree);
            }
        }
        if (ik.contains("max_normal_deviation_deg"))
            result.ik.max_normal_deviation_rad = TargetNumber(ik.at("max_normal_deviation_deg")) * radians_per_degree;
        if (ik.contains("max_candidates_per_pose")) {
            const auto& count = ik.at("max_candidates_per_pose");
            if (!count.is_number_integer() || count.get<double>() < 1 || count.get<double>() > 4)
                throw std::invalid_argument("Per-pose IK candidate count must be 1..4");
            result.ik.max_candidates_per_pose = count.get<std::size_t>();
        }
        if (ik.contains("attempts")) {
            if (!ik.at("attempts").is_number_integer() || ik.at("attempts").get<double>() < 1 || ik.at("attempts").get<double>() > 32)
                throw std::invalid_argument("IK attempts must be 1..32");
            result.ik.attempts = ik.at("attempts");
        }
        if (ik.contains("iterations")) {
            if (!ik.at("iterations").is_number_integer() || ik.at("iterations").get<double>() < 1 || ik.at("iterations").get<double>() > 500)
                throw std::invalid_argument("IK iterations must be 1..500");
            result.ik.iterations = ik.at("iterations");
        }
        if (ik.contains("timeout_s")) result.ik.timeout_s = TargetNumber(ik.at("timeout_s"));
        if (ik.contains("position_tolerance_m")) result.ik.position_tolerance_m = TargetNumber(ik.at("position_tolerance_m"));
        if (ik.contains("rotation_tolerance_rad")) result.ik.rotation_tolerance_rad = TargetNumber(ik.at("rotation_tolerance_rad"));
    }
    result.metadata = {{"kind", "carotid_coarse"}, {"source_path", source.string()}, {"source_sha256", source_sha},
        {"sample_saved_at_unix_ns", saved_ns}, {"target_clock", "file_save_time_not_exposure"},
        {"target_age_check_enabled", max_age > 0}, {"max_target_age_s", max_age},
        {"camera_serial", active.serial}, {"camera_calibration_id", active.sha256},
        {"tool_calibration_id", FileSha256(tool_file)}, {"tool_sensor_id", tool.value("sensor_id", std::string())},
        {"surface_point_base_m", TargetVecJson(result.surface_base_m)}, {"point_color_m", TargetVecJson(rows[1])},
        {"standoff_m", standoff}, {"outward_normal_base", TargetVecJson(-z)},
        {"tcp_offset_base_m", TargetVecJson(-standoff * z)}, {"arm_tip_link", result.tip_link},
        {"T_base_tcp_target", TargetMatrixJson(result.base_from_tcp)}, {"T_armtip_tcp", TargetMatrixJson(result.tip_from_tcp)}};
    return result;
}
} // namespace uspilot::collision::app
