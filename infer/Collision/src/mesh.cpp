#include "collision/robot_model.hpp"

#include <openssl/evp.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace uspilot::collision {
namespace {

constexpr std::size_t kMaxTriangles = 2000000;

std::string HashBytes(const std::string& bytes) {
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int size = 0;
    if (EVP_Digest(bytes.data(), bytes.size(), digest, &size, EVP_sha256(), nullptr) != 1)
        throw std::runtime_error("SHA256 failed");
    std::ostringstream result;
    for (unsigned int i = 0; i < size; ++i)
        result << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(digest[i]);
    return result.str();
}

std::string Read(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file || file.tellg() < 0 || file.tellg() > 256 * 1024 * 1024)
        throw std::runtime_error("Cannot read mesh or file exceeds 256 MiB: " + path.string());
    std::string bytes(static_cast<std::size_t>(file.tellg()), '\0');
    file.seekg(0);
    if (!file.read(bytes.data(), static_cast<std::streamsize>(bytes.size()))) throw std::runtime_error("Mesh read failed");
    return bytes;
}

std::uint32_t U32(const char* p) {
    const auto* b = reinterpret_cast<const unsigned char*>(p);
    return std::uint32_t(b[0]) | (std::uint32_t(b[1]) << 8)
        | (std::uint32_t(b[2]) << 16) | (std::uint32_t(b[3]) << 24);
}

double Float32(const char* p) {
    const auto bits = U32(p);
    float value;
    static_assert(sizeof(value) == sizeof(bits) && std::numeric_limits<float>::is_iec559,
                  "STL requires IEEE754 float32");
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void AddTriangle(TriangleMesh& mesh, const std::array<Eigen::Vector3d, 3>& points,
                  const Eigen::Vector3d& scale, const MeshLoadOptions& options) {
    if (mesh.input_triangles >= kMaxTriangles) throw std::runtime_error("STL exceeds triangle budget");
    ++mesh.input_triangles;
    std::array<Eigen::Vector3d, 3> scaled;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Eigen::Vector3d v = points[i].cwiseProduct(scale);
        if (!v.allFinite()) throw std::runtime_error("STL has non-finite vertex/scale product");
        // Retain a conservative bound of ALL original finite vertices,
        // including vertices of an explicitly discarded zero-area face.
        if (mesh.input_triangles == 1 && i == 0) mesh.bounds.minimum = mesh.bounds.maximum = v;
        else {
            mesh.bounds.minimum = mesh.bounds.minimum.cwiseMin(v);
            mesh.bounds.maximum = mesh.bounds.maximum.cwiseMax(v);
        }
        scaled[i] = v;
    }
    // No epsilon: preserve narrow but nonzero faces. Extended precision
    // avoids treating a double-precision norm underflow as a zero-area face.
    using WideVector = Eigen::Matrix<long double, 3, 1>;
    const WideVector a = scaled[0].cast<long double>();
    const WideVector area = (scaled[1].cast<long double>() - a).cross(scaled[2].cast<long double>() - a);
    if (!area.allFinite()) throw std::runtime_error("STL triangle area overflow");
    // Convex hulls and conservative self masks must retain ALL finite source
    // vertices, including isolated points on explicitly discarded faces.
    const auto base = mesh.vertices.size();
    mesh.vertices.insert(mesh.vertices.end(), scaled.begin(), scaled.end());
    if ((area.array() == 0).all()) {
        if (!options.discard_zero_area_triangles)
            throw std::runtime_error("STL contains zero-area triangle (strict mode)");
        ++mesh.discarded_zero_area_triangles;
        return;
    }
    mesh.triangles.push_back({base, base + 1, base + 2});
}

void Expect(std::istream& stream, const std::string& expected) {
    std::string token;
    if (!(stream >> token) || token != expected) throw std::runtime_error("Malformed ASCII STL: expected " + expected);
}

Eigen::Vector3d Point(std::istream& stream) {
    Eigen::Vector3d p;
    if (!(stream >> p.x() >> p.y() >> p.z()) || !p.allFinite())
        throw std::runtime_error("Invalid ASCII STL vector");
    return p;
}

std::filesystem::path ContainedPath(const std::filesystem::path& directory,
                                    const std::string& suffix) {
    const auto root = std::filesystem::canonical(directory);
    if (!std::filesystem::is_directory(root)) throw std::runtime_error("Mesh mapping must point to a directory");
    const auto relative = std::filesystem::path(suffix);
    if (relative.empty() || relative.is_absolute()) throw std::runtime_error("Invalid path inside mesh mapping");
    const auto path = std::filesystem::weakly_canonical(root / relative);
    const auto inside = path.lexically_relative(root);
    if (inside.empty() || *inside.begin() == "..") throw std::runtime_error("Mesh escapes mapped root");
    return path;
}

}  // namespace

std::string FileSha256(const std::filesystem::path& path) { return HashBytes(Read(path)); }

std::filesystem::path ResolveMesh(const std::string& uri,
                                  const std::filesystem::path& source_directory,
                                  const PackageRoots& packages,
                                  const MeshDirectories& directories) {
    if (uri.empty()) throw std::runtime_error("Empty mesh URI");
    std::filesystem::path path;
    std::size_t prefix_size = 0;
    const std::filesystem::path* mapped_directory = nullptr;
    for (const auto& [prefix, directory] : directories) {
        if (prefix.empty() || prefix.back() != '/' || prefix.find("://") == std::string::npos)
            throw std::runtime_error("Mesh directory prefix must be an explicit URI ending in '/'");
        if (uri.rfind(prefix, 0) == 0 && prefix.size() > prefix_size) {
            prefix_size = prefix.size();
            mapped_directory = &directory;
        }
    }
    if (mapped_directory) {
        path = ContainedPath(*mapped_directory, uri.substr(prefix_size));
    } else if (uri.rfind("package://", 0) == 0) {
        const auto rest = uri.substr(10);
        const auto slash = rest.find('/');
        if (slash == std::string::npos || slash == 0 || slash + 1 == rest.size())
            throw std::runtime_error("Invalid package URI: " + uri);
        const auto package = rest.substr(0, slash);
        const auto found = packages.find(package);
        if (found == packages.end()) throw std::runtime_error("Unmapped package: " + package + " (use --package NAME=DIR)");
        path = ContainedPath(found->second, rest.substr(slash + 1));
    } else if (uri.rfind("file://", 0) == 0) {
        path = uri.substr(7);
        if (!path.is_absolute()) throw std::runtime_error("file:// mesh path must be absolute");
    } else if (uri.find("://") != std::string::npos) {
        throw std::runtime_error("Unsupported mesh URI scheme: " + uri);
    } else {
        path = uri;
        if (path.is_relative()) {
            if (source_directory.empty()) throw std::runtime_error("Relative mesh path needs a URDF source directory");
            path = source_directory / path;
        }
    }
    if (!std::filesystem::is_regular_file(path)) throw std::runtime_error("Missing mesh file: " + path.string());
    return std::filesystem::canonical(path);
}

TriangleMesh LoadStl(const std::filesystem::path& path, const Eigen::Vector3d& scale,
                     const MeshLoadOptions& options) {
    if (!scale.allFinite() || (scale.array() <= 0).any()) throw std::runtime_error("STL scale must be finite and positive");
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    if (ext != ".stl") throw std::runtime_error("Only STL meshes are supported at this stage: " + path.string());
    const auto bytes = Read(path);
    TriangleMesh mesh;
    mesh.sha256 = HashBytes(bytes);
    const bool binary = bytes.size() >= 84 && 84ULL + 50ULL * U32(bytes.data() + 80) == bytes.size();
    if (binary) {
        const auto count = U32(bytes.data() + 80);
        if (count > kMaxTriangles) throw std::runtime_error("STL exceeds triangle budget");
        mesh.vertices.reserve(3ULL * count);
        mesh.triangles.reserve(count);
        for (std::uint32_t i = 0; i < count; ++i) {
            const char* record = bytes.data() + 84 + 50ULL * i;
            std::array<Eigen::Vector3d, 3> points;
            for (int k = 0; k < 3; ++k) for (int a = 0; a < 3; ++a)
                points[k][a] = Float32(record + 12 + 12 * k + 4 * a);
            AddTriangle(mesh, points, scale, options);
        }
    } else {
        std::istringstream stream(bytes);
        stream.imbue(std::locale::classic());
        Expect(stream, "solid");
        std::string line, token;
        std::getline(stream, line);
        bool ended = false;
        while (stream >> token) {
            if (token == "endsolid") {
                std::getline(stream, line);
                if (stream >> token) throw std::runtime_error("Unexpected data after ASCII STL");
                ended = true;
                break;
            }
            if (token != "facet") throw std::runtime_error("Malformed ASCII STL facet");
            Expect(stream, "normal");
            Point(stream);
            Expect(stream, "outer"); Expect(stream, "loop");
            std::array<Eigen::Vector3d, 3> points;
            for (auto& p : points) { Expect(stream, "vertex"); p = Point(stream); }
            Expect(stream, "endloop"); Expect(stream, "endfacet");
            AddTriangle(mesh, points, scale, options);
        }
        if (!ended) throw std::runtime_error("ASCII STL missing endsolid");
    }
    if (mesh.triangles.empty()) throw std::runtime_error("Empty STL");
    return mesh;
}

Bounds GeometryBounds(const Geometry& g, const TriangleMesh* mesh) {
    Eigen::Vector3d half;
    if (g.shape == Shape::kMesh) {
        if (!mesh || mesh->triangles.empty()) throw std::runtime_error("Mesh geometry has not been loaded");
        return mesh->bounds;
    }
    if (g.shape == Shape::kBox) half = g.box_size * 0.5;
    else if (g.shape == Shape::kSphere) half.setConstant(g.radius);
    else half = Eigen::Vector3d(g.radius, g.radius, g.length * 0.5);
    if (!half.allFinite() || (half.array() <= 0).any()) throw std::runtime_error("Invalid primitive geometry");
    return {-half, half};
}

Bounds TransformBounds(const Bounds& local, const Eigen::Isometry3d& transform) {
    if (!local.minimum.allFinite() || !local.maximum.allFinite()
        || (local.minimum.array() > local.maximum.array()).any()
        || !transform.matrix().allFinite()
        || !transform.linear().isUnitary(1e-9)
        || std::abs(transform.linear().determinant() - 1.0) > 1e-9
        || !transform.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-12))
        throw std::runtime_error("Invalid bounds/rigid transform");
    Bounds result;
    result.minimum.setConstant(std::numeric_limits<double>::infinity());
    result.maximum = -result.minimum;
    for (int i = 0; i < 8; ++i) {
        Eigen::Vector3d p;
        for (int a = 0; a < 3; ++a) p[a] = (i & (1 << a)) ? local.maximum[a] : local.minimum[a];
        p = transform * p;
        result.minimum = result.minimum.cwiseMin(p);
        result.maximum = result.maximum.cwiseMax(p);
    }
    if (!result.minimum.allFinite() || !result.maximum.allFinite()) throw std::runtime_error("Bounds overflow");
    return result;
}

bool GeometryAudit::DeclaredGeometryReady() const {
    return !bodies.empty() && links_without_collision.empty()
        && std::all_of(bodies.begin(), bodies.end(), [](const BodyAudit& b) { return b.ready; });
}

GeometryAudit AuditGeometry(const RobotModel& model, const PackageRoots& packages,
                            const MeshDirectories& directories, const MeshLoadOptions& options) {
    GeometryAudit result;
    for (const auto& link : model.Links()) {
        if (link.collisions.empty()) result.links_without_collision.push_back(link.name);
        for (std::size_t i = 0; i < link.collisions.size(); ++i) {
            BodyAudit audit;
            audit.link = link.name;
            audit.collision_index = i;
            const auto& g = link.collisions[i];
            try {
                if (g.shape == Shape::kMesh) {
                    audit.mesh_path = ResolveMesh(g.mesh_uri, model.SourceDirectory(), packages, directories);
                    const auto mesh = LoadStl(audit.mesh_path, g.mesh_scale, options);
                    audit.local_bounds = GeometryBounds(g, &mesh);
                    audit.mesh_sha256 = mesh.sha256;
                    audit.triangles = mesh.triangles.size();
                    audit.input_triangles = mesh.input_triangles;
                    audit.discarded_zero_area_triangles = mesh.discarded_zero_area_triangles;
                } else audit.local_bounds = GeometryBounds(g);
                audit.ready = true;
            } catch (const std::exception& e) { audit.error = e.what(); }
            result.bodies.push_back(std::move(audit));
        }
    }
    return result;
}

}  // namespace uspilot::collision
