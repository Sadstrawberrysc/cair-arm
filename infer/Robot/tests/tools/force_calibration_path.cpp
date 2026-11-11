#include <force_calibration_path.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <force_calibration.hpp>
#include <realman_kinematics.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

bool Fail(std::string* error, const std::string& message) {
    if (error != nullptr) *error = message;
    return false;
}

double Halton(std::size_t index, unsigned base) {
    double value = 0.0, scale = 1.0;
    while (index) {
        scale /= base;
        value += scale * (index % base);
        index /= base;
    }
    return value;
}

bool GenerateWorkspace(const Eigen::Matrix<double, 7, 1>& center,
                       const Eigen::Matrix3d& sensor_rotation, RMKinematics& kinematics,
                       const ForceCalibrationPathOptions& options,
                       ForceCalibrationPath& path, std::string* error) {
    if (options.pose_count < 6 || options.pose_count > 8000 || !options.tcp_armtip_m.allFinite())
        return Fail(error, "workspace requires 6..8000 poses and a finite TCP offset");
    auto valid_box = [](const ForceCalibrationWorkspaceBox& box) {
        return box.minimum_m.allFinite() && box.maximum_m.allFinite()
            && (box.minimum_m.array() < box.maximum_m.array()).all();
    };
    for (const auto& box : options.allowed_regions)
        if (!valid_box(box)) return Fail(error, "invalid allowed workspace region");
    for (const auto& box : options.forbidden_boxes)
        if (!valid_box(box)) return Fail(error, "invalid forbidden workspace box");
    for (const auto& sphere : options.forbidden_spheres)
        if (!sphere.center_m.allFinite() || !std::isfinite(sphere.radius_m) || sphere.radius_m <= 0)
            return Fail(error, "invalid forbidden workspace sphere");
    auto inside_box = [](const Eigen::Vector3d& point, const ForceCalibrationWorkspaceBox& box) {
        return (point.array() >= box.minimum_m.array()).all()
            && (point.array() <= box.maximum_m.array()).all();
    };
    auto allowed = [&](const Eigen::Vector3d& point) {
        bool in_region = options.allowed_regions.empty();
        for (const auto& box : options.allowed_regions) in_region = in_region || inside_box(point, box);
        if (!in_region) return false;
        for (const auto& box : options.forbidden_boxes) if (inside_box(point, box)) return false;
        for (const auto& sphere : options.forbidden_spheres)
            if ((point - sphere.center_m).norm() <= sphere.radius_m) return false;
        return true;
    };
    struct Candidate {
        Eigen::Matrix<double, 7, 1> joints;
        Eigen::Vector3d position, gravity;
    };
    if (options.preserved_prefix_joints_rad.size() >= options.pose_count)
        return Fail(error, "workspace prefix must leave at least one new point");
    const auto& minimum = kinematics.JointMinimums();
    const auto& maximum = kinematics.JointMaximums();
    const unsigned primes[] = {2, 3, 5, 7, 11, 13, 17};
    const double margin = options.joint_limit_margin_deg * kDegToRad;
    for (int axis = 0; axis < 7; ++axis) {
        if (center[axis] < minimum[axis] || center[axis] > maximum[axis]
            || minimum[axis] + margin >= maximum[axis] - margin)
            return Fail(error, "invalid workspace center or joint margin");
    }
    const auto desired_count = std::min<std::size_t>(100000, std::max<std::size_t>(8192, options.pose_count * 64));
    std::vector<Candidate> candidates;
    candidates.reserve(desired_count);
    Eigen::Vector3d lower = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d upper = -lower;
    const bool constrained = !options.allowed_regions.empty() || !options.forbidden_boxes.empty()
        || !options.forbidden_spheres.empty() || std::isfinite(options.minimum_body_z_m);
    if (!ForceCalibrationAbovePlane(center, center, kinematics, options))
        return Fail(error, "start component envelope violates minimum body height or envelope is invalid");
    const auto attempts = desired_count * (constrained ? 20 : 1);
    for (std::size_t i = 1; i <= attempts && candidates.size() < desired_count; ++i) {
        Candidate candidate;
        for (int axis = 0; axis < 7; ++axis)
            candidate.joints[axis] = minimum[axis] + margin
                + Halton(i, primes[axis]) * (maximum[axis] - minimum[axis] - 2 * margin);
        Eigen::Matrix4d fk;
        kinematics.GetKinematics(fk, candidate.joints);
        candidate.position = fk.topRightCorner<3, 1>() + fk.topLeftCorner<3, 3>() * options.tcp_armtip_m;
        if (!allowed(candidate.position)) continue;
        if (!ForceCalibrationAbovePlane(center, candidate.joints, kinematics, options)) continue;
        candidate.gravity = (fk.topLeftCorner<3, 3>() * sensor_rotation).transpose()
            * Eigen::Vector3d(0, 0, -1);
        lower = lower.cwiseMin(candidate.position);
        upper = upper.cwiseMax(candidate.position);
        candidates.push_back(candidate);
    }
    const auto count = candidates.size();
    if (count < options.pose_count - options.preserved_prefix_joints_rad.size())
        return Fail(error, "too few reachable candidates in configured allowed/forbidden regions");
    const double scale = std::max(1e-9, (upper - lower).maxCoeff());
    std::vector<double> nearest(count, std::numeric_limits<double>::infinity());
    std::vector<double> position_distance(count, std::numeric_limits<double>::infinity());
    std::vector<bool> used(count, false);
    std::vector<Candidate> prefix;
    for (const auto& original : options.preserved_prefix_joints_rad) {
        Candidate point;
        point.joints = original;
        for (int axis = 0; axis < 7; ++axis)
            if (!std::isfinite(original[axis]) || original[axis] < minimum[axis]
                || original[axis] > maximum[axis]) return Fail(error, "invalid preserved joint target");
        auto q = original;
        Eigen::Matrix4d fk;
        kinematics.GetKinematics(fk, q);
        point.position = fk.topRightCorner<3, 1>() + fk.topLeftCorner<3, 3>() * options.tcp_armtip_m;
        point.gravity = (fk.topLeftCorner<3, 3>() * sensor_rotation).transpose() * Eigen::Vector3d(0,0,-1);
        prefix.push_back(point);
    }
    auto update_distances = [&](const Candidate& point) {
        std::size_t farthest = 0;
        double farthest_distance = -1;
        for (std::size_t i = 0; i < count; ++i) {
            const double distance = (candidates[i].position - point.position).squaredNorm();
            position_distance[i] = std::min(position_distance[i], distance);
            const double score = distance / (scale * scale)
                + 0.01 * (candidates[i].gravity - point.gravity).squaredNorm();
            nearest[i] = std::min(nearest[i], score);
            if (!used[i] && nearest[i] > farthest_distance) {
                farthest = i;
                farthest_distance = nearest[i];
            }
        }
        return farthest;
    };
    std::size_t next = 0;
    for (const auto& point : prefix) next = update_distances(point);
    std::vector<std::size_t> selected;
    selected.reserve(options.pose_count - prefix.size());
    for (std::size_t pick = prefix.size(); pick < options.pose_count; ++pick) {
        selected.push_back(next);
        used[next] = true;
        next = update_distances(candidates[next]);
    }
    path.workspace_min_m = lower;
    path.workspace_max_m = upper;
    path.workspace_candidate_count = count;
    path.candidate_cover_radius_m = std::sqrt(*std::max_element(position_distance.begin(), position_distance.end()));
    // Reorder the selected spatial samples to shorten joint travel from the actual start.
    auto previous = center;
    auto nearest_branch = [&](Eigen::Matrix<double, 7, 1> joints) {
        // J7 spans two revolutions. Equivalent branches have identical FK;
        // choose the nearest valid turn instead of adding a cable-winding turn.
        const double original = joints[6];
        for (int turn = -2; turn <= 2; ++turn) {
            const double value = original + turn * 2.0 * kPi;
            if (value >= minimum[6] + margin && value <= maximum[6] - margin
                && std::abs(value - previous[6]) < std::abs(joints[6] - previous[6]))
                joints[6] = value;
        }
        return joints;
    };
    std::vector<ForceCalibrationSample> synthetic;
    auto append = [&](const Candidate& point, const Eigen::Matrix<double, 7, 1>& joints) {
        path.via_center.push_back(!ForceCalibrationAbovePlane(previous, joints, kinematics, options));
        path.maximum_step_deg = std::max(path.maximum_step_deg,
            (joints - previous).cwiseAbs().maxCoeff() / kDegToRad);
        previous = joints;
        path.joints_rad.push_back(joints);
        path.tcp_positions_base_m.push_back(point.position);
        path.gravity_directions_sensor.push_back(point.gravity);
        auto q = joints;
        Eigen::Matrix4d fk;
        kinematics.GetKinematics(fk, q);
        ForceCalibrationSample sample;
        sample.rotation_base_from_sensor = fk.topLeftCorner<3, 3>() * sensor_rotation;
        sample.raw_wrench_sensor.head<3>() = 5.0 * point.gravity;
        sample.raw_wrench_sensor.tail<3>() = Eigen::Vector3d(.01, -.02, .08).cross(5.0 * point.gravity);
        synthetic.push_back(sample);
    };
    path.preserved_prefix_count = prefix.size();
    for (const auto& point : prefix) append(point, point.joints);
    for (std::size_t position = 0; position < selected.size(); ++position) {
        std::size_t best = position;
        double best_cost = std::numeric_limits<double>::infinity();
        for (std::size_t j = position; j < selected.size(); ++j) {
            const auto q = nearest_branch(candidates[selected[j]].joints);
            const double cost = (q - previous).squaredNorm();
            if (cost < best_cost) { best_cost = cost; best = j; }
        }
        std::swap(selected[position], selected[best]);
        const auto& point = candidates[selected[position]];
        const auto joints = nearest_branch(point.joints);
        append(point, joints);
    }
    ForceCalibration fit;
    std::string detail;
    if (!ForceCalibration::Fit(synthetic, fit, nullptr, nullptr, &detail)) {
        path = ForceCalibrationPath{};
        return Fail(error, "workspace orientation excitation failed: " + detail);
    }
    double minimum_cosine = 1;
    for (const auto& a : path.gravity_directions_sensor)
        for (const auto& b : path.gravity_directions_sensor)
            minimum_cosine = std::min(minimum_cosine, a.dot(b));
    path.maximum_gravity_separation_deg = std::acos(std::clamp(minimum_cosine, -1.0, 1.0)) / kDegToRad;
    return true;
}

}  // namespace

bool ForceCalibrationAbovePlane(const Eigen::Matrix<double, 7, 1>& from,
    const Eigen::Matrix<double, 7, 1>& to, RMKinematics& kinematics,
    const ForceCalibrationPathOptions& options) {
    if (options.minimum_body_z_m == -std::numeric_limits<double>::infinity()) return true;
    if (!from.allFinite() || !to.allFinite() || !std::isfinite(options.minimum_body_z_m)
        || options.body_spheres.empty()) return false;
    auto clearance = [&](const Eigen::Matrix<double, 7, 1>& q) {
        const auto links = kinematics.GetLinkKinematics(q);
        double result = std::numeric_limits<double>::infinity();
        for (const auto& body : options.body_spheres) {
            if (body.joint_index < 0 || body.joint_index >= 7 || !body.center_m.allFinite()
                || !std::isfinite(body.radius_m) || body.radius_m < 0) return -1.0;
            const auto& t = links[body.joint_index];
            const Eigen::Vector3d p = t.topRightCorner<3,1>() + t.topLeftCorner<3,3>() * body.center_m;
            result = std::min(result, p.z() - body.radius_m - options.minimum_body_z_m);
        }
        return result;
    };
    double radius = 0;
    for (const auto& body : options.body_spheres)
        // Sum of all MDH translations bounds distance from any joint axis.
        radius = std::max(radius, .8505 + body.center_m.norm() + body.radius_m);
    auto check = [&](auto&& self, const Eigen::Matrix<double,7,1>& a,
                     const Eigen::Matrix<double,7,1>& b, double ca, double cb, int depth) -> bool {
        if (ca <= .005 || cb <= .005) return false;
        // Every interior state is at most half the joint arc from one endpoint.
        if (std::min(ca, cb) > .005 + radius * (b-a).cwiseAbs().sum() * .5) return true;
        if (depth == 16) return false;
        const Eigen::Matrix<double,7,1> middle = (a+b)*.5;
        const double cm = clearance(middle);
        return self(self,a,middle,ca,cm,depth+1) && self(self,middle,b,cm,cb,depth+1);
    };
    return check(check, from, to, clearance(from), clearance(to), 0);
}

bool GenerateForceCalibrationPath(
    const Eigen::Matrix<double, 7, 1>& center_joints_rad,
    const Eigen::Matrix3d& rotation_armtip_from_sensor,
    RMKinematics& kinematics,
    const ForceCalibrationPathOptions& options,
    ForceCalibrationPath& path,
    std::string* error) {
    path = ForceCalibrationPath{};
    if (error != nullptr) error->clear();
    if (!center_joints_rad.allFinite()
        || !rotation_armtip_from_sensor.allFinite()
        || (rotation_armtip_from_sensor.transpose() * rotation_armtip_from_sensor
            - Eigen::Matrix3d::Identity()).norm() > 1e-6
        || std::abs(rotation_armtip_from_sensor.determinant() - 1.0) > 1e-6) {
        return Fail(error, "path center and sensor rotation must be finite and rigid");
    }
    if (!options.arm_half_ranges_deg.allFinite()
        || (options.arm_half_ranges_deg.array() < 0.0).any()
        || !options.wrist_half_ranges_deg.allFinite()
        || (options.wrist_half_ranges_deg.array() <= 0.0).any()
        || !std::isfinite(options.joint_limit_margin_deg)
        || options.joint_limit_margin_deg < 0.0) {
        return Fail(error, "arm ranges must be non-negative, wrist ranges positive, and margin non-negative");
    }
    if (options.workspace_coverage)
        return GenerateWorkspace(center_joints_rad, rotation_armtip_from_sensor,
                                 kinematics, options, path, error);
    if (options.minimum_body_z_m != -std::numeric_limits<double>::infinity())
        return Fail(error, "component height constraints require workspace mode");
    // Bound the offline allocation and integer arithmetic before cube checks.
    if (options.pose_count < 27 || options.pose_count > 8000) {
        return Fail(error, "pose count must be a complete cube in 27..8000");
    }
    const auto levels = static_cast<std::size_t>(
        std::llround(std::cbrt(static_cast<double>(options.pose_count))));
    if (levels * levels * levels != options.pose_count) {
        return Fail(error, "pose count must be a complete cube (default 10*10*10=1000)");
    }
    const auto& joint_min = kinematics.JointMinimums();
    const auto& joint_max = kinematics.JointMaximums();
    const double margin = options.joint_limit_margin_deg * kDegToRad;
    for (int joint = 0; joint < 7; ++joint) {
        if (center_joints_rad[joint] < joint_min[joint]
            || center_joints_rad[joint] > joint_max[joint]) {
            return Fail(error, "path center exceeds a joint hard limit");
        }
    }
    Eigen::Matrix<double, 4, 1> arm_lower, arm_upper;
    for (int joint = 0; joint < 4; ++joint) {
        const double amplitude = options.arm_half_ranges_deg[joint] * kDegToRad;
        arm_lower[joint] = std::max(center_joints_rad[joint] - amplitude, joint_min[joint] + margin);
        arm_upper[joint] = std::min(center_joints_rad[joint] + amplitude, joint_max[joint] - margin);
        if (arm_lower[joint] > arm_upper[joint]) {
            return Fail(error, "clipped arm joint interval has no usable amplitude");
        }
    }
    ForceCalibrationPath candidate;
    Eigen::Vector3d lower, upper;
    for (int axis = 0; axis < 3; ++axis) {
        const int joint = axis + 4;
        const double half_range = options.wrist_half_ranges_deg[axis] * kDegToRad;
        lower[axis] = std::max(center_joints_rad[joint] - half_range,
                               joint_min[joint] + margin);
        upper[axis] = std::min(center_joints_rad[joint] + half_range,
                               joint_max[joint] - margin);
        if (!(upper[axis] - lower[axis] > 1e-8)) {
            return Fail(error, "clipped wrist range has no usable amplitude");
        }
    }
    candidate.wrist_min_deg = lower / kDegToRad;
    candidate.wrist_max_deg = upper / kDegToRad;
    // Choose the nearer corner as approach target, keeping the same grid.
    Eigen::Vector3d first, last;
    for (int axis = 0; axis < 3; ++axis) {
        const bool lower_first = center_joints_rad[axis + 4] - lower[axis]
            <= upper[axis] - center_joints_rad[axis + 4];
        first[axis] = lower_first ? lower[axis] : upper[axis];
        last[axis] = lower_first ? upper[axis] : lower[axis];
    }
    candidate.joints_rad.reserve(options.pose_count);
    candidate.gravity_directions_sensor.reserve(options.pose_count);
    std::vector<ForceCalibrationSample> synthetic;
    synthetic.reserve(options.pose_count);
    const Eigen::Vector3d gravity_base(0.0, 0.0, -5.0);
    const Eigen::Vector3d force_bias(0.2, -0.3, 0.1);
    const Eigen::Vector3d torque_bias(0.01, 0.02, -0.015);
    const Eigen::Vector3d center_of_mass(0.01, -0.02, 0.08);
    for (std::size_t i = 0; i < levels; ++i) {
        for (std::size_t row = 0; row < levels; ++row) {
            const std::size_t j = i % 2 == 0 ? row : levels - 1 - row;
            const bool forward = (i * levels + row) % 2 == 0;
            for (std::size_t column = 0; column < levels; ++column) {
                const std::size_t k = forward ? column : levels - 1 - column;
                const Eigen::Vector3d fraction(
                    static_cast<double>(i) / (levels - 1),
                    static_cast<double>(j) / (levels - 1),
                    static_cast<double>(k) / (levels - 1));
                Eigen::Matrix<double, 7, 1> joints = center_joints_rad;
                const double phase = 2.0 * kPi * candidate.joints_rad.size() / (options.pose_count - 1);
                for (int axis = 0; axis < 4; ++axis) {
                    const double offset = options.arm_half_ranges_deg[axis] * kDegToRad
                        * std::sin((axis + 1) * phase);
                    joints[axis] = std::clamp(center_joints_rad[axis] + offset,
                                             arm_lower[axis], arm_upper[axis]);
                }
                joints.tail<3>() = first + fraction.cwiseProduct(last - first);
                if (!candidate.joints_rad.empty()) {
                    candidate.maximum_step_deg = std::max(candidate.maximum_step_deg,
                        (joints - candidate.joints_rad.back()).cwiseAbs().maxCoeff() / kDegToRad);
                }
                Eigen::Matrix4d fk;
                kinematics.GetKinematics(fk, joints);
                ForceCalibrationSample sample;
                sample.rotation_base_from_sensor = fk.topLeftCorner<3, 3>()
                    * rotation_armtip_from_sensor;
                const Eigen::Vector3d gravity_sensor =
                    sample.rotation_base_from_sensor.transpose() * gravity_base;
                sample.raw_wrench_sensor.head<3>() = force_bias + gravity_sensor;
                sample.raw_wrench_sensor.tail<3>() = torque_bias + center_of_mass.cross(gravity_sensor);
                synthetic.push_back(sample);
                candidate.joints_rad.push_back(joints);
                candidate.gravity_directions_sensor.push_back(gravity_sensor / gravity_base.norm());
            }
        }
    }
    ForceCalibration fit;
    std::string fit_error;
    if (!ForceCalibration::Fit(synthetic, fit, nullptr, nullptr, &fit_error)) {
        return Fail(error, "planned path lacks gravity excitation: " + fit_error);
    }
    double minimum_cosine = 1.0;
    for (std::size_t i = 0; i < candidate.gravity_directions_sensor.size(); ++i) {
        for (std::size_t j = i + 1; j < candidate.gravity_directions_sensor.size(); ++j) {
            minimum_cosine = std::min(minimum_cosine,
                candidate.gravity_directions_sensor[i].dot(candidate.gravity_directions_sensor[j]));
        }
    }
    candidate.maximum_gravity_separation_deg = std::acos(
        std::clamp(minimum_cosine, -1.0, 1.0)) / kDegToRad;
    path = std::move(candidate);
    return true;
}
