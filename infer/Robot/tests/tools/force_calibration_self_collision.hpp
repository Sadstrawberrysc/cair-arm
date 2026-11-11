#pragma once

// Shared by the offline filter and capture-time guard. No device/network I/O.
#include <collision/collision_checker.hpp>
#include <json.hpp>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <cmath>

namespace calibration_collision {
using Json = nlohmann::json;
using Joints = Eigen::Matrix<double,7,1>;
namespace c = uspilot::collision;

inline Json Read(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open collision input: " + path);
    return Json::parse(input);
}
inline c::JointPositions Named(const Joints& q) {
    c::JointPositions result;
    for (int i=0;i<7;++i) result["joint"+std::to_string(i+1)] = q[i];
    return result;
}
inline Joints Radians(const Json& values) {
    if (!values.is_array() || values.size()!=7) throw std::runtime_error("Expected seven joint angles");
    Joints q;
    for (int i=0;i<7;++i) q[i]=values.at(i).get<double>()*std::acos(-1.0)/180;
    if (!q.allFinite()) throw std::runtime_error("Nonfinite collision joints");
    return q;
}
inline Json Diagnostic(const c::StateCheck& state) {
    Json pairs=Json::array();
    for (const auto& p:state.violations) pairs.push_back({{"first",p.first_body},{"second",p.second_body},
        {"intersecting",p.intersecting},{"distance_m",p.distance_m}});
    return {{"status",c::CheckStatusName(state.status)},{"detail",state.detail},{"pairs",pairs}};
}

class Guard {
public:
    explicit Guard(const Json& config) {
        std::vector<c::SelfCollisionExclusion> exclusions;
        const auto policy=Read(config.at("exclusions_file").get<std::string>());
        for (const auto& e:policy.at("self_collision_exclusions")) {
            const auto a=e.at("first_link").get<std::string>(),b=e.at("second_link").get<std::string>();
            if ((a=="camera_link" && (b=="base_link" || b=="support_link" ||
                  (b.rfind("Link",0)==0 && b!="Link7"))) ||
                (b=="camera_link" && (a=="base_link" || a=="support_link" ||
                  (a.rfind("Link",0)==0 && a!="Link7"))))
                throw std::runtime_error("Camera-to-arm collision checks cannot be excluded");
            exclusions.push_back({a,b,e.at("reason").get<std::string>()});
        }
        c::MeshLoadOptions mesh; mesh.discard_zero_area_triangles=true;
        c::MeshDirectories directories{{"package://rm_description/meshes/rm_75_arm/",
            config.at("mesh_directory").get<std::string>()}};
        checker=std::make_shared<c::CollisionChecker>(c::RobotModel::Load(config.at("urdf").get<std::string>()),
            c::PackageRoots{},directories,mesh,exclusions);
        if (config.contains("model_id") && config.at("model_id")!=checker->ModelId())
            throw std::runtime_error("Self-collision geometry/policy changed; refilter the remaining path");
        options.clearance_m=config.value("clearance_m",.005);
        if (!std::isfinite(options.clearance_m) || options.clearance_m<0)
            throw std::runtime_error("Invalid self-collision clearance");
        options.allow_unknown_space=true; // Self collision ONLY; no invented environment/free-space box.
        scene.id="calibration_self_collision_only";
        scene.calibration_id=checker->ModelId();
    }
    c::StateCheck State(const Joints& q) const {return checker->CheckState(Named(q),scene,0,options);}
    c::SegmentCheck Segment(const Joints& a,const Joints& b) const {
        return checker->CheckSegment(Named(a),Named(b),scene,0,options);
    }
    std::string ModelId() const {return checker->ModelId();}
private:
    std::shared_ptr<c::CollisionChecker> checker;
    c::CheckOptions options;
    c::CollisionScene scene;
};
} // namespace calibration_collision
