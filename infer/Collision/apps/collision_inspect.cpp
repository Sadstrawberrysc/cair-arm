#include "collision/robot_model.hpp"
#include <realman_kinematics.hpp>
#include <json.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <locale>
#include <random>
#include <sstream>
#include <stdexcept>

namespace c = uspilot::collision;
using Json = nlohmann::json;

namespace {
Json Matrix(const Eigen::Matrix4d& matrix) {
    Json rows = Json::array();
    for (int i = 0; i < 4; ++i) rows.push_back({matrix(i, 0), matrix(i, 1), matrix(i, 2), matrix(i, 3)});
    return rows;
}

Json Vector(const Eigen::Vector3d& v) { return Json::array({v.x(), v.y(), v.z()}); }

Eigen::Matrix<double, 7, 1> ParseJoints(std::string text) {
    std::replace(text.begin(), text.end(), ',', ' ');
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    Eigen::Matrix<double, 7, 1> q;
    for (int i = 0; i < 7; ++i) if (!(stream >> q[i]) || !std::isfinite(q[i]))
        throw std::runtime_error("--joints requires seven finite radians");
    std::string extra;
    if (stream >> extra) throw std::runtime_error("Too many --joints values");
    return q;
}

c::JointPositions Positions(const Eigen::Matrix<double, 7, 1>& q) {
    c::JointPositions positions;
    for (int i = 0; i < 7; ++i) positions.emplace("joint" + std::to_string(i + 1), q[i]);
    return positions;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        std::filesystem::path urdf;
        c::PackageRoots packages;
        c::MeshDirectories mesh_directories;
        c::MeshLoadOptions mesh_options;
        bool kinematics_only = false;
        Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
        for (int i = 1; i < argc; ++i) {
            const std::string arg(argv[i]);
            if (arg == "--help") {
                std::cout << "Offline geometry audit only; no robot/network/device access.\n"
                    "collision_inspect --urdf FILE [--package NAME=DIR ...] [--mesh-dir DIR]\n"
                    "  [--joints q1,q2,q3,q4,q5,q6,q7] [--kinematics-only]\n"
                    "  [--drop-zero-area-triangles]\n"
                    "Default joints: zero radians. JSON on stdout.\n"
                    "Exit 0: selected audit passed; 2: declared geometry incomplete;\n"
                    "     3: RM75 model/MDH mismatch; 1: input/parse error.\n"
                    "--kinematics-only permits missing geometry, explicitly reported in JSON.\n";
                std::cout << "--mesh-dir maps package://rm_description/meshes/rm_75_arm/ to DIR;\n"
                    "it preserves original URDF paths and cannot be combined with --package rm_description.\n"
                    "--drop-zero-area-triangles explicitly discards exactly zero-area faces in memory;\n"
                    "counts and raw SHA256 are reported; source STL files are never rewritten.\n";
                return 0;
            }
            if (arg == "--kinematics-only") { kinematics_only = true; continue; }
            if (arg == "--drop-zero-area-triangles") {
                mesh_options.discard_zero_area_triangles = true;
                continue;
            }
            if (arg != "--urdf" && arg != "--package" && arg != "--joints" && arg != "--mesh-dir")
                throw std::runtime_error("Unknown argument: " + arg);
            if (++i >= argc) throw std::runtime_error("Missing value for " + arg);
            const std::string value(argv[i]);
            if (arg == "--urdf") urdf = value;
            else if (arg == "--joints") q = ParseJoints(value);
            else if (arg == "--mesh-dir") {
                const auto directory = std::filesystem::canonical(value);
                if (!std::filesystem::is_directory(directory)) throw std::runtime_error("--mesh-dir must be a directory");
                if (!mesh_directories.emplace("package://rm_description/meshes/rm_75_arm/", directory).second)
                    throw std::runtime_error("Duplicate --mesh-dir");
            }
            else {
                const auto equal = value.find('=');
                if (equal == std::string::npos || equal == 0 || equal + 1 == value.size())
                    throw std::runtime_error("--package requires NAME=DIR");
                const auto directory = std::filesystem::absolute(value.substr(equal + 1));
                if (!std::filesystem::is_directory(directory)) throw std::runtime_error("Package directory does not exist");
                if (!packages.emplace(value.substr(0, equal), directory).second)
                    throw std::runtime_error("Duplicate package mapping");
            }
        }
        if (!mesh_directories.empty() && packages.count("rm_description"))
            throw std::runtime_error("Use either --mesh-dir or --package rm_description, not both");
        if (urdf.empty()) throw std::runtime_error("Specify --urdf FILE (use --help)");
        const auto model = c::RobotModel::Load(urdf);
        if (model.Root() != "base_link" || model.ActiveJointNames().size() != 7)
            throw std::runtime_error("Inspector requires the RM75 seven-joint chain with optional fixed attachments");
        std::array<const c::Joint*, 7> arm_joints{};
        for (int i = 0; i < 7; ++i) {
            const auto found = std::find_if(model.Joints().begin(), model.Joints().end(), [&](const c::Joint& j) {
                return j.name == "joint" + std::to_string(i + 1);
            });
            if (found == model.Joints().end()) throw std::runtime_error("Missing RM75 arm joint");
            const auto& j = *found;
            arm_joints[i] = &j;
            if (j.type != c::JointType::kRevolute
                || j.parent != (i == 0 ? "base_link" : "Link" + std::to_string(i))
                || j.child != "Link" + std::to_string(i + 1))
                throw std::runtime_error("RM75 joint-name/order/type/link mapping mismatch");
        }
        RMKinematics reference;
        Json limits = Json::array();
        Eigen::Matrix<double, 7, 1> lower, upper;
        bool limits_differ = false;
        for (int i = 0; i < 7; ++i) {
            const auto& j = *arm_joints[i];
            lower[i] = std::max(j.lower, reference.JointMinimums()[i]);
            upper[i] = std::min(j.upper, reference.JointMaximums()[i]);
            if (lower[i] >= upper[i] || q[i] < lower[i] || q[i] > upper[i])
                throw std::runtime_error("No common limits or requested joint outside intersection: " + j.name);
            limits_differ = limits_differ || std::abs(j.lower - reference.JointMinimums()[i]) > 1e-9
                || std::abs(j.upper - reference.JointMaximums()[i]) > 1e-9;
            limits.push_back({{"joint", j.name}, {"urdf_lower_rad", j.lower}, {"urdf_upper_rad", j.upper},
                {"mdh_lower_rad", reference.JointMinimums()[i]}, {"mdh_upper_rad", reference.JointMaximums()[i]},
                {"intersection_lower_rad", lower[i]}, {"intersection_upper_rad", upper[i]}});
        }
        double max_position = 0.0, max_angle = 0.0;
        std::size_t sample_count = 0;
        auto compare = [&](Eigen::Matrix<double, 7, 1> sample) {
            const Eigen::Matrix4d terminal = model.Forward(Positions(sample)).at("Link7").matrix();
            Eigen::Matrix4d expected;
            reference.GetKinematics(expected, sample);
            max_position = std::max(max_position, (terminal.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm());
            max_angle = std::max(max_angle, std::abs(Eigen::AngleAxisd(
                terminal.block<3, 3>(0, 0) * expected.block<3, 3>(0, 0).transpose()).angle()));
            ++sample_count;
        };
        compare(q);
        compare(Eigen::Matrix<double, 7, 1>::Zero());
        compare(lower);
        compare(upper);
        for (int i = 0; i < 7; ++i) {
            Eigen::Matrix<double, 7, 1> sample = Eigen::Matrix<double, 7, 1>::Zero();
            sample[i] = lower[i]; compare(sample);
            sample[i] = upper[i]; compare(sample);
        }
        std::mt19937 random(75);
        for (int s = 0; s < 256; ++s) {
            Eigen::Matrix<double, 7, 1> sample;
            for (int i = 0; i < 7; ++i)
                sample[i] = std::uniform_real_distribution<double>(lower[i], upper[i])(random);
            compare(sample);
        }
        // CAD-export rounding comparison only, not a physical clearance margin.
        constexpr double position_tolerance = 2e-5;
        // URDF uses six rounded +/-1.5708 frame rotations and rounded axes;
        // allow their accumulated CAD-export rounding, without rewriting them.
        constexpr double angle_tolerance = 5e-5;
        const bool mdh_match = max_position <= position_tolerance && max_angle <= angle_tolerance;
        const auto poses = model.Forward(Positions(q));
        const auto audit = c::AuditGeometry(model, packages, mesh_directories, mesh_options);
        Json output = {{"mode", "offline_geometry_audit"}, {"robot_motion_enabled", false},
            {"urdf", std::filesystem::absolute(urdf).string()}, {"urdf_sha256", model.Sha256()},
            {"mesh_directory_mappings", Json::object()},
            {"discard_zero_area_triangles", mesh_options.discard_zero_area_triangles},
            {"robot", model.Name()}, {"root_frame", model.Root()}, {"joint_unit", "rad"},
            {"translation_unit", "m"}, {"kinematics_only", kinematics_only},
            {"declared_geometry_ready", audit.DeclaredGeometryReady()},
            {"physical_geometry_validated", false}, {"self_collision_checked", false},
            {"end_effector_attachments_validated", false}, {"ready_for_motion_planning", false},
            {"limits_differ", limits_differ}, {"joint_limits", limits},
            {"links_without_collision", audit.links_without_collision},
            {"mdh_comparison", {{"terminal", "Link7 vs existing RMKinematics end frame"},
                {"samples", sample_count}, {"max_position_error_m", max_position},
                {"max_orientation_error_rad", max_angle}, {"position_tolerance_m", position_tolerance},
                {"orientation_tolerance_rad", angle_tolerance}, {"passed", mdh_match}}},
            {"link_poses", Json::object()}, {"joint_positions_rad", Json::object()}, {"collision_bodies", Json::array()}};
        for (const auto& [prefix, directory] : mesh_directories)
            output["mesh_directory_mappings"][prefix] = directory.string();
        for (const auto& [name, value] : Positions(q)) output["joint_positions_rad"][name] = value;
        for (const auto& [name, pose] : poses) output["link_poses"][name] = Matrix(pose.matrix());
        for (const auto& body : audit.bodies) {
            Json item = {{"link", body.link}, {"collision_index", body.collision_index}, {"ready", body.ready},
                {"mesh_path", body.mesh_path.string()}, {"mesh_sha256", body.mesh_sha256},
                {"triangles", body.triangles}, {"error", body.error},
                {"input_triangles", body.input_triangles},
                {"discarded_zero_area_triangles", body.discarded_zero_area_triangles},
                {"base_from_collision", Matrix(model.CollisionPose(poses, body.link, body.collision_index).matrix())}};
            if (body.ready) {
                const auto bounds = c::TransformBounds(body.local_bounds, model.CollisionPose(poses, body.link, body.collision_index));
                item["world_aabb_m"] = {{"min", Vector(bounds.minimum)}, {"max", Vector(bounds.maximum)}};
            }
            output["collision_bodies"].push_back(item);
        }
        std::cout << output.dump(2) << '\n';
        if (!mdh_match) return 3;
        return kinematics_only || audit.DeclaredGeometryReady() ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << "collision_inspect: " << e.what() << '\n';
        return 1;
    }
}
