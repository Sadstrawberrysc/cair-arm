#pragma once

#include <Eigen/Geometry>
#include <array>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace uspilot::collision {

enum class Shape { kBox, kSphere, kCylinder, kMesh };
enum class JointType { kFixed, kRevolute, kContinuous, kPrismatic };

struct Geometry {
    std::string name;
    Shape shape = Shape::kBox;
    Eigen::Isometry3d link_from_geometry = Eigen::Isometry3d::Identity();
    Eigen::Vector3d box_size = Eigen::Vector3d::Zero();
    double radius = 0.0;
    double length = 0.0;
    std::string mesh_uri;
    Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones();
};

struct Link {
    std::string name;
    std::vector<Geometry> collisions;
};

struct Joint {
    std::string name;
    JointType type = JointType::kFixed;
    std::string parent;
    std::string child;
    Eigen::Isometry3d parent_from_joint = Eigen::Isometry3d::Identity();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    double lower = 0.0;
    double upper = 0.0;
    double velocity = 0.0;
};

using JointPositions = std::map<std::string, double>;
using LinkPoses = std::map<std::string, Eigen::Isometry3d>;
using PackageRoots = std::map<std::string, std::filesystem::path>;
// Explicit URI directory prefixes (ending in '/') mapped to local directories.
// The longest matching prefix wins; package roots handle other package URIs.
using MeshDirectories = std::map<std::string, std::filesystem::path>;

// A geometry-only URDF subset. No visual fallback, Xacro evaluation,
// device I/O, inferred joint offsets or implicit package search paths.
class RobotModel {
public:
    static RobotModel Load(const std::filesystem::path& urdf);
    static RobotModel Parse(const std::string& xml);
    const std::string& Name() const { return name_; }
    const std::string& Root() const { return root_; }
    const std::string& Sha256() const { return sha256_; }
    const std::filesystem::path& SourceDirectory() const { return source_directory_; }
    const std::vector<Link>& Links() const { return links_; }
    const std::vector<Joint>& Joints() const { return joints_; }
    const std::vector<std::string>& ActiveJointNames() const { return active_names_; }
    LinkPoses Forward(const JointPositions& positions) const;
    Eigen::Isometry3d CollisionPose(const LinkPoses& poses,
                                    const std::string& link,
                                    std::size_t collision_index) const;

private:
    std::string name_, root_, sha256_;
    std::filesystem::path source_directory_;
    std::vector<Link> links_;
    std::vector<Joint> joints_;  // Topological, parent before child.
    std::vector<std::string> active_names_;
};

struct Bounds {
    Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
    Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
};

struct TriangleMesh {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<std::size_t, 3>> triangles;
    Bounds bounds;
    std::string sha256;
    std::size_t input_triangles = 0;
    std::size_t discarded_zero_area_triangles = 0;
};

struct MeshLoadOptions {
    // Explicit opt-in. Never changes source bytes or drops near-zero faces.
    bool discard_zero_area_triangles = false;
};

// package roots point to the package directory itself, not its parent.
std::filesystem::path ResolveMesh(const std::string& uri,
                                  const std::filesystem::path& source_directory,
                                  const PackageRoots& packages,
                                  const MeshDirectories& directories = {});
TriangleMesh LoadStl(const std::filesystem::path& path,
                     const Eigen::Vector3d& scale = Eigen::Vector3d::Ones(),
                     const MeshLoadOptions& options = {});
Bounds GeometryBounds(const Geometry& geometry, const TriangleMesh* mesh = nullptr);
Bounds TransformBounds(const Bounds& local, const Eigen::Isometry3d& transform);
std::string FileSha256(const std::filesystem::path& path);

struct BodyAudit {
    std::string link;
    std::size_t collision_index = 0;
    bool ready = false;
    std::filesystem::path mesh_path;
    std::string mesh_sha256;
    std::size_t triangles = 0;
    std::size_t input_triangles = 0;
    std::size_t discarded_zero_area_triangles = 0;
    Bounds local_bounds;
    std::string error;
};

struct GeometryAudit {
    std::vector<BodyAudit> bodies;
    std::vector<std::string> links_without_collision;
    // Means declared geometry loaded, not physically validated or collision-free.
    bool DeclaredGeometryReady() const;
};

GeometryAudit AuditGeometry(const RobotModel& model, const PackageRoots& packages,
                            const MeshDirectories& directories = {},
                            const MeshLoadOptions& options = {});

}  // namespace uspilot::collision
