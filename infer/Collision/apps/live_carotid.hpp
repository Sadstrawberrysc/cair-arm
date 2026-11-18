#pragma once
#include "coarse_target.hpp"
#include "collision/robot_read_only.hpp"

namespace uspilot::collision::app {
inline std::optional<LiveCarotidTarget> MapLiveCarotid(const std::filesystem::path& directory,
        const MonitorFrame& frame, const MonitorResult& observation, const MonitorOptions& options,
        const CameraCalibration& calibration, std::string& detail) {
    detail="waiting for carotid detection";
    if (!observation.input_valid) { detail="camera input invalid"; return std::nullopt; }
    const auto path=directory/"target.json";
    try {
        if (!std::filesystem::exists(path)) return std::nullopt;
        if (std::filesystem::file_size(path)>65536) throw std::runtime_error("target packet too large");
        std::ifstream input(path); const auto packet=Json::parse(input);
        if (packet.at("schema_version")!=1 || packet.at("session_id")!=directory.filename().string() ||
            packet.at("camera_serial")!=calibration.serial || packet.at("calibration_id")!=calibration.sha256)
            throw std::runtime_error("target session/calibration mismatch");
        if (!packet.at("valid").get<bool>()) { detail="carotid target unavailable"; return std::nullopt; }
        LiveCarotidTarget target;
        target.session_id=packet.at("session_id").get<std::string>();
        target.source_sequence=packet.at("sequence").get<std::uint64_t>();
        target.scene_sequence=frame.sequence;
        target.captured_at_ns=packet.at("captured_at_ns").get<std::int64_t>();
        const auto age=MonitorSteadyNowNs()-target.captured_at_ns;
        if (!packet.at("capture_time_trusted").get<bool>() || age<0 || age>2000000000 ||
            target.source_sequence>frame.sequence) {
            detail="carotid observation stale/untrusted"; return std::nullopt;
        }
        target.point_color_m=TargetVector(packet.at("point_color_m"));
        target.outward_normal_color=TargetVector(packet.at("outward_normal_color"));
        target.tangent_color=TargetVector(packet.at("tangent_color"));
        if (target.point_color_m.z()<=0 || target.outward_normal_color.norm()<1e-9 ||
            target.outward_normal_color.z()>1e-6 || target.tangent_color.norm()<1e-9)
            throw std::runtime_error("invalid live carotid point/normal/tangent");
        target.outward_normal_color.normalize(); target.tangent_color.normalize();
        const Eigen::Vector3d source_base=calibration.base_from_color*target.point_color_m;
        double best=std::numeric_limits<double>::infinity();
        for (std::size_t i=0; i<observation.points_base_m.size(); ++i) {
            if (i>=observation.robot_mask.size() || observation.robot_mask[i] ||
                (i<observation.occluded_mask.size() && observation.occluded_mask[i]) ||
                (i<observation.outlier_mask.size() && observation.outlier_mask[i])) continue;
            const auto& p=observation.points_base_m[i];
            if ((p-options.region.minimum).minCoeff()<0 || (options.region.maximum-p).minCoeff()<0) continue;
            const double distance=(p-source_base).squaredNorm();
            if (distance<best) { best=distance; target.matched_base_m=p; }
        }
        if (!std::isfinite(best)) { detail="no retained environment point"; return std::nullopt; }
        target.match_distance_m=std::sqrt(best);
        detail="mapped red point: "+std::to_string(int(target.match_distance_m*1000+.5))+"mm; source "+
            std::to_string(age/1000000)+"ms";
        return target;
    } catch (const std::exception& error) {
        detail="carotid target: "+std::string(error.what()); return std::nullopt;
    }
}
}
