#include <contact_sensing.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

namespace {

constexpr double kGeometryEpsilon = 1e-12;
constexpr double kSvdRelativeTolerance = 1e-10;
constexpr double kBarycentricTolerance = 1e-8;

struct PlaneSolution {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double residual = std::numeric_limits<double>::infinity();
    bool valid = false;
};

std::string lowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool finiteVector(const Eigen::Vector3d &value)
{
    return value.array().isFinite().all();
}

bool finiteVertices(const Eigen::Matrix3d &vertices)
{
    return vertices.array().isFinite().all();
}

bool orientedTriangleNormal(const Eigen::Matrix3d &vertices,
                            const Eigen::Vector3d *stl_normal,
                            Eigen::Vector3d &unit_normal)
{
    if (!finiteVertices(vertices)) {
        return false;
    }

    const Eigen::Vector3d edge_1 = vertices.row(1).transpose() - vertices.row(0).transpose();
    const Eigen::Vector3d edge_2 = vertices.row(2).transpose() - vertices.row(0).transpose();
    const double edge_scale = std::max(edge_1.norm(), edge_2.norm());
    Eigen::Vector3d geometric_normal = edge_1.cross(edge_2);
    const double geometric_norm = geometric_normal.norm();
    const double degeneracy_tolerance =
        64.0 * std::numeric_limits<double>::epsilon() * edge_scale * edge_scale;
    if (!std::isfinite(edge_scale) || edge_scale <= std::numeric_limits<double>::min() ||
        !std::isfinite(geometric_norm) || geometric_norm <= degeneracy_tolerance) {
        return false;
    }
    geometric_normal /= geometric_norm;

    if (stl_normal != nullptr && finiteVector(*stl_normal)) {
        const double supplied_norm = stl_normal->norm();
        if (std::isfinite(supplied_norm) && supplied_norm > kGeometryEpsilon &&
            geometric_normal.dot(*stl_normal) < 0.0) {
            geometric_normal = -geometric_normal;
        }
    }

    unit_normal = geometric_normal;
    return true;
}

PlaneSolution solveContactOnPlane(const Eigen::Vector3d &force,
                                  const Eigen::Vector3d &moment,
                                  const Eigen::Vector3d &unit_normal,
                                  double plane_offset)
{
    PlaneSolution result;
    Eigen::Matrix4d system;
    system << 0.0, force.z(), -force.y(), unit_normal.x(),
              -force.z(), 0.0, force.x(), unit_normal.y(),
              force.y(), -force.x(), 0.0, unit_normal.z(),
              unit_normal.x(), unit_normal.y(), unit_normal.z(), 0.0;

    Eigen::Vector4d rhs;
    rhs << moment, -plane_offset;

    Eigen::JacobiSVD<Eigen::Matrix4d> svd(
        system, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector4d singular_values = svd.singularValues();
    const double largest = singular_values.maxCoeff();
    if (!std::isfinite(largest) || largest <= kGeometryEpsilon) {
        return result;
    }

    svd.setThreshold(kSvdRelativeTolerance);
    if (svd.rank() < 4) {
        return result;
    }

    const Eigen::Vector4d solution = svd.solve(rhs);
    if (!solution.array().isFinite().all()) {
        return result;
    }

    result.point = solution.head<3>();
    const double algebraic_residual = (system * solution - rhs).norm();
    result.residual = (result.point.cross(force) - moment).norm();
    result.valid = std::isfinite(algebraic_residual) &&
                   algebraic_residual <=
                       kSvdRelativeTolerance * (1.0 + rhs.norm()) &&
                   std::isfinite(result.residual);
    return result;
}

bool pointInTriangle(const Eigen::Vector3d &point,
                     const Eigen::Matrix3d &vertices,
                     const Eigen::Vector3d &unit_normal)
{
    const Eigen::Vector3d a = vertices.row(0).transpose();
    const Eigen::Vector3d b = vertices.row(1).transpose();
    const Eigen::Vector3d c = vertices.row(2).transpose();
    const Eigen::Vector3d ab = b - a;
    const Eigen::Vector3d ac = c - a;
    const Eigen::Vector3d ap = point - a;

    const double scale = std::max({ab.norm(), ac.norm(), (c - b).norm()});
    if (!std::isfinite(scale) || scale <= std::numeric_limits<double>::min()) {
        return false;
    }
    if (std::abs(unit_normal.dot(ap)) > kBarycentricTolerance * scale) {
        return false;
    }

    const double d00 = ab.dot(ab);
    const double d01 = ab.dot(ac);
    const double d11 = ac.dot(ac);
    const double d20 = ap.dot(ab);
    const double d21 = ap.dot(ac);
    const double denominator = d00 * d11 - d01 * d01;
    if (!std::isfinite(denominator) ||
        std::abs(denominator) <=
            64.0 * std::numeric_limits<double>::epsilon() *
                scale * scale * scale * scale) {
        return false;
    }

    const double bary_b = (d11 * d20 - d01 * d21) / denominator;
    const double bary_c = (d00 * d21 - d01 * d20) / denominator;
    const double bary_a = 1.0 - bary_b - bary_c;
    return std::isfinite(bary_a) && std::isfinite(bary_b) && std::isfinite(bary_c) &&
           bary_a >= -kBarycentricTolerance &&
           bary_b >= -kBarycentricTolerance &&
           bary_c >= -kBarycentricTolerance;
}

}  // namespace

const char *ContactEstimateErrorString(ContactEstimateError error) noexcept
{
    switch (error) {
    case ContactEstimateError::None:
        return "none";
    case ContactEstimateError::ModelNotLoaded:
        return "contact model is not loaded";
    case ContactEstimateError::NonFiniteWrench:
        return "wrench contains a non-finite value";
    case ContactEstimateError::ForceTooSmall:
        return "force magnitude is below the estimation threshold";
    case ContactEstimateError::IllConditionedSystem:
        return "contact equation is rank deficient or ill-conditioned";
    case ContactEstimateError::ResidualTooLarge:
        return "contact wrench residual exceeds the point-error threshold";
    case ContactEstimateError::NoValidSurface:
        return "line of force does not yield a valid inward-facing surface point";
    }
    return "unknown contact estimation error";
}

bool ContactLocation::LoadSTL(const std::string &filename)
{
    normal.clear();
    vertex.clear();
    last_error_.clear();

    std::ifstream input(filename);
    if (!input.is_open()) {
        last_error_ = "cannot open ASCII STL file: " + filename;
        return false;
    }

    std::vector<Eigen::Vector3d> loaded_normals;
    std::vector<Eigen::Matrix3d> loaded_vertices;
    Eigen::Vector3d pending_normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d pending_vertices = Eigen::Matrix3d::Zero();
    bool in_facet = false;
    int vertex_count = 0;
    std::string line;
    std::size_t line_number = 0;

    auto fail = [&](const std::string &reason) {
        last_error_ = "invalid ASCII STL at line " + std::to_string(line_number) +
                      ": " + reason;
        return false;
    };

    while (std::getline(input, line)) {
        ++line_number;
        std::istringstream stream(line);
        std::string keyword;
        if (!(stream >> keyword)) {
            continue;
        }
        keyword = lowerCopy(keyword);

        if (keyword == "facet") {
            std::string normal_keyword;
            if (in_facet) {
                return fail("nested facet");
            }
            if (!(stream >> normal_keyword >> pending_normal.x() >> pending_normal.y() >>
                  pending_normal.z()) ||
                lowerCopy(normal_keyword) != "normal" || !finiteVector(pending_normal)) {
                return fail("malformed facet normal");
            }
            in_facet = true;
            vertex_count = 0;
            continue;
        }

        if (keyword == "vertex") {
            if (!in_facet || vertex_count >= 3) {
                return fail("vertex outside a facet or too many facet vertices");
            }
            Eigen::Vector3d value;
            if (!(stream >> value.x() >> value.y() >> value.z()) || !finiteVector(value)) {
                return fail("malformed vertex");
            }
            pending_vertices.row(vertex_count) = value.transpose();
            ++vertex_count;
            continue;
        }

        if (keyword == "endfacet") {
            if (!in_facet || vertex_count != 3) {
                return fail("facet does not contain exactly three vertices");
            }

            Eigen::Vector3d unit_normal;
            if (!orientedTriangleNormal(pending_vertices, &pending_normal, unit_normal)) {
                return fail("degenerate triangle");
            }
            loaded_normals.push_back(unit_normal);
            loaded_vertices.push_back(pending_vertices);
            in_facet = false;
            vertex_count = 0;
            continue;
        }

        // "solid", "endsolid", "outer loop" and "endloop" are structural
        // ASCII STL tokens and do not carry data needed by the estimator.
        if (keyword != "solid" && keyword != "endsolid" && keyword != "outer" &&
            keyword != "endloop") {
            return fail("unexpected token '" + keyword + "'");
        }
    }

    if (input.bad()) {
        last_error_ = "failed while reading ASCII STL file: " + filename;
        return false;
    }
    if (in_facet) {
        return fail("unterminated facet");
    }
    if (loaded_vertices.empty()) {
        last_error_ = "ASCII STL contains no facets: " + filename;
        return false;
    }

    normal = std::move(loaded_normals);
    vertex = std::move(loaded_vertices);
    return true;
}

ContactEstimate ContactLocation::estimateContactPoint(
    const Eigen::Matrix<double, 6, 1> &wrench,
    double min_force_norm,
    double max_point_error_m) const
{
    ContactEstimate estimate;
    if (vertex.empty()) {
        estimate.error = ContactEstimateError::ModelNotLoaded;
        return estimate;
    }
    if (!wrench.array().isFinite().all() || !std::isfinite(min_force_norm) ||
        min_force_norm < 0.0 || !std::isfinite(max_point_error_m) ||
        max_point_error_m <= 0.0) {
        estimate.error = ContactEstimateError::NonFiniteWrench;
        return estimate;
    }

    const Eigen::Vector3d force = wrench.head<3>();
    const Eigen::Vector3d moment = wrench.tail<3>();
    const double force_norm = force.norm();
    if (!std::isfinite(force_norm) || force_norm < min_force_norm) {
        estimate.error = ContactEstimateError::ForceTooSmall;
        return estimate;
    }

    bool found_full_rank_system = false;
    bool found_excessive_residual = false;
    for (std::size_t i = 0; i < vertex.size(); ++i) {
        const Eigen::Vector3d &unit_normal = normal[i];

        const Eigen::Vector3d first_vertex = vertex[i].row(0).transpose();
        const double plane_offset = -unit_normal.dot(first_vertex);
        const PlaneSolution solution =
            solveContactOnPlane(force, moment, unit_normal, plane_offset);
        if (!solution.valid) {
            continue;
        }
        found_full_rank_system = true;

        const double point_error_m = solution.residual / force_norm;
        if (!std::isfinite(point_error_m)
            || point_error_m > max_point_error_m) {
            found_excessive_residual = true;
            continue;
        }

        const double inward_projection = force.dot(unit_normal);
        const double direction_tolerance = force_norm * kSvdRelativeTolerance;
        if (inward_projection >= -direction_tolerance ||
            !pointInTriangle(solution.point, vertex[i], unit_normal)) {
            continue;
        }

        if (!estimate.valid || solution.residual < estimate.residual) {
            estimate.point = solution.point;
            estimate.residual = solution.residual;
            estimate.point_error_m = point_error_m;
            estimate.valid = true;
            estimate.error = ContactEstimateError::None;
        }
    }

    if (estimate.valid) {
        return estimate;
    }

    if (!found_full_rank_system) {
        estimate.error = ContactEstimateError::IllConditionedSystem;
    } else if (found_excessive_residual) {
        estimate.error = ContactEstimateError::ResidualTooLarge;
    } else {
        estimate.error = ContactEstimateError::NoValidSurface;
    }
    return estimate;
}
