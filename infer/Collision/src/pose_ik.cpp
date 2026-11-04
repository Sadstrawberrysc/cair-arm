#include "collision/pose_ik.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <numeric>
#include <stdexcept>

namespace uspilot::collision {
namespace {
using Clock = std::chrono::steady_clock;
using Error = Eigen::Matrix<double, 6, 1>;
void Rigid(const Eigen::Isometry3d& t) {
    if (!t.matrix().allFinite() ||
        (t.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-8 ||
        (t.linear().transpose() * t.linear() - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        std::abs(t.linear().determinant() - 1) > 1e-6)
        throw std::invalid_argument("IK requires rigid TCP transforms");
}
Error Difference(const Eigen::Isometry3d& to, const Eigen::Isometry3d& from) {
    Error e;
    e.head<3>() = to.translation() - from.translation();
    const Eigen::AngleAxisd r(to.linear() * from.linear().transpose());
    e.tail<3>() = r.axis() * r.angle();
    return e;
}
double Cost(Error e) { e.tail<3>() *= 0.2; return e.squaredNorm(); }
double JointDistance(const JointPositions& a, const JointPositions& b) {
    double d = 0;
    for (const auto& [name, value] : a) d += std::pow(value - b.at(name), 2);
    return d;
}
} // namespace

PoseIkResult SolvePoseIk(const RobotModel& model, const JointPositions& seed,
    const std::string& tip_link, const Eigen::Isometry3d& tip_from_tcp,
    const Eigen::Isometry3d& target, const PoseIkOptions& options, const std::function<bool()>& cancelled) {
    Rigid(target); Rigid(tip_from_tcp);
    if (!options.attempts || options.attempts > 32 || !options.iterations || options.iterations > 500 ||
        !std::isfinite(options.timeout_s) || options.timeout_s <= 0 || options.timeout_s > 30 ||
        !std::isfinite(options.position_tolerance_m) || options.position_tolerance_m <= 0 || options.position_tolerance_m > .01 ||
        !std::isfinite(options.rotation_tolerance_rad) || options.rotation_tolerance_rad <= 0 || options.rotation_tolerance_rad > .1)
        throw std::invalid_argument("Invalid IK budget or convergence tolerance");
    const auto& names = model.ActiveJointNames();
    if (options.yaw_offsets_rad.empty() || options.yaw_offsets_rad.size() > 12)
        throw std::invalid_argument("IK requires 1..12 yaw samples");
    for (const auto angle : options.yaw_offsets_rad)
        if (!std::isfinite(angle) || std::abs(angle) > std::acos(-1.) + 1e-9)
            throw std::invalid_argument("IK yaw sample must be within +/- pi");
    if (options.tilt_offsets_rad.empty() || options.tilt_offsets_rad.size() > 5 ||
        !std::isfinite(options.max_normal_deviation_rad) || options.max_normal_deviation_rad <= 0 ||
        options.max_normal_deviation_rad > 10 * std::acos(-1.) / 180 + 1e-12 ||
        !options.max_candidates_per_pose || options.max_candidates_per_pose > 4)
        throw std::invalid_argument("Invalid IK normal cone or per-pose candidate count");
    for (const auto& tilt : options.tilt_offsets_rad)
        if (!tilt.allFinite() || tilt.norm() > options.max_normal_deviation_rad + 1e-12)
            throw std::invalid_argument("IK tilt sample exceeds normal cone");
    if (names.empty() || names.size() > 32) throw std::invalid_argument("IK needs bounded revolute joints");
    const auto seed_fk = model.Forward(seed);
    if (!seed_fk.count(tip_link)) throw std::invalid_argument("Unknown ArmTip link: " + tip_link);
    std::vector<double> lower, upper;
    for (const auto& name : names) {
        const auto joint = std::find_if(model.Joints().begin(), model.Joints().end(), [&](const Joint& j) { return j.name == name; });
        if (joint == model.Joints().end() || joint->type != JointType::kRevolute || joint->lower >= joint->upper)
            throw std::invalid_argument("IK supports bounded revolute joints only");
        lower.push_back(joint->lower); upper.push_back(joint->upper);
    }
    auto tcp = [&](const JointPositions& q) { return model.Forward(q).at(tip_link) * tip_from_tcp; };
    const auto begin = Clock::now();
    auto stop = [&] { return (cancelled && cancelled()) || std::chrono::duration<double>(Clock::now() - begin).count() >= options.timeout_s; };
    std::mt19937 random(options.seed);
    std::normal_distribution<double> normal(0, 1);
    PoseIkResult result;
    double best_cost = std::numeric_limits<double>::infinity();
    const auto pose_count = options.yaw_offsets_rad.size() * options.tilt_offsets_rad.size();
    for (std::size_t pose_index = 0; pose_index < pose_count && !stop(); ++pose_index) {
      const double yaw = options.yaw_offsets_rad[pose_index % options.yaw_offsets_rad.size()];
      const auto& tilt = options.tilt_offsets_rad[pose_index / options.yaw_offsets_rad.size()];
      Eigen::Matrix3d tilt_rotation = Eigen::Matrix3d::Identity();
      if (tilt.norm() > 1e-12) tilt_rotation = Eigen::AngleAxisd(tilt.norm(),
          Eigen::Vector3d(tilt.x(), tilt.y(), 0).normalized()).toRotationMatrix();
      Eigen::Isometry3d pose_target = target;
      pose_target.linear() = target.linear() * Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * tilt_rotation;
      const auto pose_begin = Clock::now();
      const double pose_budget = std::max(0., options.timeout_s -
          std::chrono::duration<double>(pose_begin - begin).count()) / (pose_count-pose_index);
      auto pose_stop = [&] { return stop() || std::chrono::duration<double>(Clock::now()-pose_begin).count() >= pose_budget; };
      std::size_t pose_candidates = 0;
      for (std::size_t attempt = 0; attempt < options.attempts && !pose_stop(); ++attempt) {
        ++result.attempts;
        auto q = seed;
        if (attempt) for (std::size_t j = 0; j < names.size(); ++j)
            q[names[j]] = std::clamp(seed.at(names[j]) + (.3 + .15 * attempt) * normal(random), lower[j], upper[j]);
        for (std::size_t iteration = 0; iteration < options.iterations && !pose_stop(); ++iteration) {
            ++result.iterations;
            const auto actual = tcp(q);
            const auto error = Difference(pose_target, actual);
            const double cost = Cost(error);
            if (cost < best_cost) {
                best_cost = cost; result.best_position_error_m = error.head<3>().norm();
                result.best_rotation_error_rad = error.tail<3>().norm();
            }
            if (error.head<3>().norm() <= options.position_tolerance_m && error.tail<3>().norm() <= options.rotation_tolerance_rad) {
                const double deviation = std::acos(std::clamp(actual.linear().col(2).dot(target.linear().col(2)), -1., 1.));
                if (deviation > options.max_normal_deviation_rad) break;
                bool duplicate = false;
                for (const auto& previous : result.candidates) if (JointDistance(q, previous) < 1e-4) duplicate = true;
                if (!duplicate) {
                    result.candidates.push_back(q);
                    result.candidate_yaw_offsets_rad.push_back(yaw);
                    result.candidate_normal_deviation_rad.push_back(deviation);
                    result.candidate_targets.push_back(pose_target);
                    ++pose_candidates;
                }
                break;
            }
            Eigen::MatrixXd jacobian(6, names.size());
            for (std::size_t j = 0; j < names.size(); ++j) {
                auto plus = q, minus = q;
                plus[names[j]] = std::min(upper[j], q.at(names[j]) + 1e-5);
                minus[names[j]] = std::max(lower[j], q.at(names[j]) - 1e-5);
                const double span = plus.at(names[j]) - minus.at(names[j]);
                if (span > 0) jacobian.col(j) = Difference(tcp(plus), tcp(minus)) / span;
                else jacobian.col(j).setZero();
            }
            Error weighted = error; weighted.tail<3>() *= .2;
            jacobian.bottomRows(3) *= .2;
            const Eigen::Matrix<double, 6, 6> lhs = jacobian * jacobian.transpose() + 0.000025 * Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::VectorXd delta = jacobian.transpose() * lhs.ldlt().solve(weighted);
            if (!delta.allFinite()) break;
            const double scale = std::max(1.0, delta.cwiseAbs().maxCoeff() / .2);
            delta /= scale;
            bool improved = false;
            for (int backtrack = 0; backtrack < 8 && !pose_stop(); ++backtrack) {
                auto trial = q;
                for (std::size_t j = 0; j < names.size(); ++j)
                    trial[names[j]] = std::clamp(q.at(names[j]) + delta[j], lower[j], upper[j]);
                if (Cost(Difference(pose_target, tcp(trial))) < cost) { q = std::move(trial); improved = true; break; }
                delta *= .5;
            }
            if (!improved) break;
        }
        if (pose_candidates >= options.max_candidates_per_pose) break;
      }
    }
    std::vector<std::size_t> order(result.candidates.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](const auto a, const auto b) {
        return JointDistance(result.candidates[a], seed) < JointDistance(result.candidates[b], seed);
    });
    auto candidates = std::move(result.candidates);
    auto yaws = std::move(result.candidate_yaw_offsets_rad);
    auto deviations = std::move(result.candidate_normal_deviation_rad);
    auto targets = std::move(result.candidate_targets);
    result.candidates.clear(); result.candidate_yaw_offsets_rad.clear();
    result.candidate_normal_deviation_rad.clear(); result.candidate_targets.clear();
    for (const auto i : order) {
        result.candidates.push_back(std::move(candidates[i]));
        result.candidate_yaw_offsets_rad.push_back(yaws[i]);
        result.candidate_normal_deviation_rad.push_back(deviations[i]);
        result.candidate_targets.push_back(std::move(targets[i]));
    }
    return result;
}

PosePlanResult PlanPosePath(const RobotModel& model, const CollisionChecker& checker,
    const JointPositions& start, const std::string& tip_link, const Eigen::Isometry3d& tip_from_tcp,
    const Eigen::Isometry3d& target, const CollisionScene& scene, std::int64_t now,
    const PlannerOptions& planning, const PoseIkOptions& ik, const std::function<bool()>& cancelled) {
    const auto begin = Clock::now();
    PosePlanResult out;
    auto& result = out.plan;
    result.unknown_space_allowed = planning.collision.allow_unknown_space;
    result.model_id = checker.ModelId(); result.scene_id = scene.id; result.calibration_id = scene.calibration_id;
    result.captured_at_ns = scene.captured_at_ns; result.checked_at_ns = now; result.seed = planning.seed;
    auto elapsed = [&] { return std::chrono::duration<double>(Clock::now() - begin).count(); };
    auto stop = [&] { return (cancelled && cancelled()) || elapsed() >= planning.timeout_s; };
    try {
        if (model.Sha256() != checker.UrdfSha256() || !std::isfinite(planning.timeout_s) ||
            planning.timeout_s <= 0 || planning.timeout_s > 300 || !planning.max_queries || planning.max_queries > 1000000)
            throw std::invalid_argument("Invalid pose planning context or budget");
        if (!planning.skip_start_precheck) {
            result.start_check = checker.CheckState(start, scene, now, planning.collision);
            result.state_queries = 1;
        } else result.start_check.detail = "skipped_start_precheck";
        if (!planning.skip_start_precheck && !result.start_check.Valid()) { result.status = "invalid_start"; result.detail = result.start_check.detail; }
        else {
            out.ik = SolvePoseIk(model, start, tip_link, tip_from_tcp, target, ik, stop);
            result.status = "ik_no_solution"; result.detail = "No IK candidate within the configured budget; not proof of unreachability";
            out.candidate_checks.resize(out.ik.candidates.size());
            out.candidate_searches.resize(out.ik.candidates.size());
            std::vector<std::size_t> valid_candidates;
            for (std::size_t i = 0; i < out.ik.candidates.size(); ++i) {
                out.candidate_searches[i].candidate_index = i;
                out.candidate_searches[i].yaw_offset_rad = out.ik.candidate_yaw_offsets_rad.at(i);
                out.candidate_searches[i].normal_deviation_rad = out.ik.candidate_normal_deviation_rad.at(i);
                out.candidate_searches[i].detail = "Not searched: shared budget or cancellation";
                out.candidate_checks[i].detail = "candidate_not_checked";
            }
            // Check every endpoint first so invalid goals consume no search slot.
            for (std::size_t i = 0; i < out.ik.candidates.size(); ++i) {
                if (stop()) break;
                if (result.state_queries + result.segment_queries >= planning.max_queries) { result.status = "budget_exceeded"; break; }
                ++result.state_queries;
                result.goal_check = checker.CheckState(out.ik.candidates[i], scene, now, planning.collision);
                out.candidate_checks[i] = result.goal_check;
                out.preview_candidate_index = i;
                if (!result.goal_check.Valid()) {
                    result.status = "invalid_goal"; result.detail = result.goal_check.detail;
                    out.candidate_searches[i].status = "invalid_goal";
                    out.candidate_searches[i].detail = result.goal_check.detail;
                } else valid_candidates.push_back(i);
            }
            std::size_t state_queries = result.state_queries, segment_queries = result.segment_queries;
            std::size_t segment_state_checks = result.segment_state_checks;
            std::optional<PlanResult> best;
            std::optional<std::size_t> best_index;
            double best_length = std::numeric_limits<double>::infinity();
            for (std::size_t slot = 0; slot < valid_candidates.size(); ++slot) {
                const auto index = valid_candidates[slot];
                const auto used = state_queries + segment_queries;
                if (stop() || used >= planning.max_queries) break;
                const auto slots_left = valid_candidates.size() - slot;
                auto remaining = planning;
                remaining.timeout_s = (planning.timeout_s - elapsed()) / slots_left;
                remaining.max_queries = (planning.max_queries - used) / slots_left;
                if (remaining.timeout_s <= 0 || !remaining.max_queries) { result.status = "budget_exceeded"; break; }
                auto& summary = out.candidate_searches[index];
                summary.budget_s = remaining.timeout_s;
                auto attempt = PlanJointPath(model, checker, start, out.ik.candidates[index], scene, now, remaining, stop);
                summary.status = attempt.status; summary.detail = attempt.detail;
                summary.elapsed_ms = attempt.elapsed_ms; summary.waypoint_count = attempt.waypoints.size();
                state_queries += attempt.state_queries;
                segment_queries += attempt.segment_queries;
                segment_state_checks += attempt.segment_state_checks;
                if (attempt.Exact()) {
                    double length = 0;
                    for (std::size_t j = 1; j < attempt.waypoints.size(); ++j)
                        length += std::sqrt(JointDistance(attempt.waypoints[j-1], attempt.waypoints[j]));
                    summary.joint_path_length_rad = length;
                    if (length < best_length) { best_length = length; best = attempt; best_index = index; }
                }
                result = std::move(attempt);
                out.preview_candidate_index = index;
            }
            if (best) { result = std::move(*best); out.preview_candidate_index = best_index; }
            else if (state_queries + segment_queries >= planning.max_queries) {
                result.status = "budget_exceeded"; result.detail = "Shared candidate query budget exhausted";
            }
            result.state_queries = state_queries; result.segment_queries = segment_queries;
            result.segment_state_checks = segment_state_checks;
        }
        if (cancelled && cancelled()) { result.status = "cancelled"; result.detail = "Pose planning cancelled"; result.waypoints.clear(); }
        else if (elapsed() >= planning.timeout_s && !result.Exact()) { result.status = "timeout"; result.detail = "Shared IK/planning budget exhausted"; result.waypoints.clear(); }
    } catch (const std::exception& e) { result.status = "invalid_input"; result.detail = e.what(); result.waypoints.clear(); }
    result.elapsed_ms = elapsed() * 1000;
    return out;
}
} // namespace uspilot::collision
