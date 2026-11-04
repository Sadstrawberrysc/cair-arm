#include "collision/joint_planner.hpp"
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace uspilot::collision {
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace {
using Clock = std::chrono::steady_clock;
class SeededSampler : public ob::RealVectorStateSampler {
public:
    SeededSampler(const ob::StateSpace* space, std::uint32_t seed) : RealVectorStateSampler(space) {
        rng_.setLocalSeed(seed);
    }
};
class SeededConnect : public og::RRTConnect {
public:
    SeededConnect(const ob::SpaceInformationPtr& si, std::uint32_t seed) : RRTConnect(si) {
        rng_.setLocalSeed(seed);
    }
};
struct Context {
    const CollisionChecker& checker;
    const CollisionScene& scene;
    const PlannerOptions& options;
    const std::function<bool()>& cancel;
    std::int64_t now;
    Clock::time_point begin;
    PlanResult& result;
    std::vector<std::string> names;
    std::string stop;
    // Exact keys, scoped to one immutable scene/options/clock context. Never
    // reuse certificates across snapshots or after cancellation/deadline.
    std::map<JointPositions, StateCheck> state_cache;
    std::map<std::pair<JointPositions, JointPositions>, SegmentCheck> segment_cache;
    static bool Cacheable(const JointPositions& q) {
        return std::all_of(q.begin(), q.end(), [](const auto& entry) { return std::isfinite(entry.second); });
    }
    bool Stopped() {
        if (!stop.empty()) return true;
        if (cancel && cancel()) stop = "cancelled";
        else if (std::chrono::duration<double>(Clock::now() - begin).count() >= options.timeout_s)
            stop = "timeout";
        return !stop.empty();
    }
    bool Query() {
        if (Stopped()) return false;
        if (result.state_queries + result.segment_queries >= options.max_queries) {
            stop = "budget_exceeded";
            return false;
        }
        return true;
    }
    JointPositions Decode(const ob::State* state) const {
        const auto* values = state->as<ob::RealVectorStateSpace::StateType>();
        JointPositions q;
        for (std::size_t i = 0; i < names.size(); ++i) q[names[i]] = values->values[i];
        return q;
    }
    bool State(const JointPositions& q, StateCheck* diagnostic = nullptr) {
        if (!Query()) return false;
        const bool cacheable = Cacheable(q);
        const auto found = cacheable ? state_cache.find(q) : state_cache.end();
        if (found != state_cache.end()) {
            if (diagnostic) *diagnostic = found->second;
            return found->second.Valid();
        }
        ++result.state_queries;
        auto check = checker.CheckState(q, scene, now, options.collision);
        const bool valid = check.Valid();
        if (cacheable && valid && state_cache.size() < 4096) state_cache.emplace(q, check);
        if (diagnostic) *diagnostic = std::move(check);
        return valid;
    }
    bool Segment(const JointPositions& a, const JointPositions& b) {
        if (!Query()) return false;
        const bool cacheable = Cacheable(a) && Cacheable(b);
        const auto key = cacheable && b < a ? std::make_pair(b, a) : std::make_pair(a, b);
        if (cacheable && segment_cache.count(key)) return true;
        ++result.segment_queries;
        auto check = checker.CheckSegment(a, b, scene, now, options.collision, [this] { return Stopped(); });
        result.segment_state_checks += check.state_checks;
        if (!check.Valid()) result.last_segment_failure = check;
        else if (cacheable && !Stopped() && segment_cache.size() < 4096) segment_cache.emplace(key, check);
        return check.Valid() && !Stopped();
    }
};
class CertifiedMotion : public ob::MotionValidator {
public:
    CertifiedMotion(const ob::SpaceInformationPtr& si, Context& c) : MotionValidator(si), c_(c) {}
    bool checkMotion(const ob::State* a, const ob::State* b) const override {
        const bool ok = c_.Segment(c_.Decode(a), c_.Decode(b));
        if (ok) ++valid_; else ++invalid_;
        return ok;
    }
    bool checkMotion(const ob::State* a, const ob::State* b,
                     std::pair<ob::State*, double>& last) const override {
        if (checkMotion(a, b)) return true;
        // Returning the known-valid start is conservative. Never report an
        // unverified prefix merely from a failed interval's collision sample.
        last.second = 0;
        if (last.first) si_->copyState(last.first, a);
        return false;
    }
private:
    Context& c_;
};
} // namespace

PlanResult PlanJointPath(const RobotModel& model, const CollisionChecker& checker,
    const JointPositions& start, const JointPositions& goal, const CollisionScene& scene,
    std::int64_t now, const PlannerOptions& options, const std::function<bool()>& cancelled) {
    PlanResult result;
    result.unknown_space_allowed = options.collision.allow_unknown_space;
    result.model_id = checker.ModelId(); result.scene_id = scene.id;
    result.calibration_id = scene.calibration_id; result.captured_at_ns = scene.captured_at_ns;
    result.checked_at_ns = now; result.seed = options.seed;
    const auto begin = Clock::now();
    Context c{checker, scene, options, cancelled, now, begin, result, model.ActiveJointNames(), {}, {}, {}};
    try {
        if (!std::isfinite(options.timeout_s) || options.timeout_s <= 0 || options.timeout_s > 300 ||
            !std::isfinite(options.range) || options.range <= 0 || !options.max_queries ||
            options.max_queries > 1000000 || options.shortcut_attempts > 10000 || !options.seed || c.names.empty() || c.names.size() > 32)
            throw std::invalid_argument("Invalid planner options or active joint count");
        if (model.Sha256() != checker.UrdfSha256())
            throw std::invalid_argument("Robot model and collision checker identity mismatch");
        auto space = std::make_shared<ob::RealVectorStateSpace>(c.names.size());
        ob::RealVectorBounds bounds(c.names.size());
        for (std::size_t i = 0; i < c.names.size(); ++i) {
            auto joint = std::find_if(model.Joints().begin(), model.Joints().end(),
                [&](const Joint& j) { return j.name == c.names[i]; });
            if (joint == model.Joints().end() || joint->type != JointType::kRevolute ||
                !std::isfinite(joint->lower) || !std::isfinite(joint->upper) || joint->lower >= joint->upper)
                throw std::invalid_argument("Planner requires finite bounded revolute joints (rad)");
            bounds.setLow(i, joint->lower); bounds.setHigh(i, joint->upper);
        }
        space->setBounds(bounds);
        space->setStateSamplerAllocator([&](const ob::StateSpace* s) {
            return std::make_shared<SeededSampler>(s, options.seed);
        });
        auto fail = [&](const std::string& status, const std::string& detail) {
            result.status = c.stop.empty() ? status : c.stop;
            result.detail = detail;
        };
        if (options.skip_start_precheck) result.start_check.detail = "skipped_start_precheck";
        if (!options.skip_start_precheck && !c.State(start, &result.start_check)) fail("invalid_start", result.start_check.detail);
        else if (!c.State(goal, &result.goal_check)) fail("invalid_goal", result.goal_check.detail);
        else {
            og::SimpleSetup setup(space);
            auto si = setup.getSpaceInformation();
            auto* space_info = si.get();
            si->setStateValidityChecker([&, space_info](const ob::State* state) {
                return space_info->satisfiesBounds(state) && c.State(c.Decode(state));
            });
            si->setMotionValidator(std::make_shared<CertifiedMotion>(si, c));
            ob::ScopedState<> a(space), b(space);
            for (std::size_t i = 0; i < c.names.size(); ++i) { a[i] = start.at(c.names[i]); b[i] = goal.at(c.names[i]); }
            setup.setStartAndGoalStates(a, b, 1e-12);
            auto planner = std::make_shared<SeededConnect>(si, options.seed);
            planner->setRange(options.range); setup.setPlanner(planner);
            const auto status = setup.solve(ob::PlannerTerminationCondition([&] { return !c.Query(); }));
            if (status != ob::PlannerStatus::EXACT_SOLUTION || !setup.haveExactSolutionPath())
                fail("no_exact_solution", "No exact path within the planning budget; this is not proof of infeasibility");
            else if (c.Stopped()) fail(c.stop, "Planning interrupted");
            else {
                std::vector<JointPositions> path;
                for (auto* state : setup.getSolutionPath().getStates()) path.push_back(c.Decode(state));
                if (path.empty()) throw std::runtime_error("OMPL returned an empty exact path");
                if (path.size() == 1 && start != goal) path.push_back(goal);
                // RRTConnect's goal tolerance must never silently alter the requested endpoints.
                path.front() = start; path.back() = goal;
                // Deterministic longest-span shortcut attempts, each using the same segment checker.
                std::size_t attempts = 0;
                for (std::size_t i = 0; i + 2 < path.size() && attempts < options.shortcut_attempts && !c.Stopped(); ++i) {
                    for (std::size_t j = path.size() - 1; j > i + 1 && attempts < options.shortcut_attempts; --j) {
                        ++attempts;
                        if (c.Segment(path[i], path[j])) {
                            path.erase(path.begin() + i + 1, path.begin() + j);
                            break;
                        }
                        if (!c.stop.empty()) break;
                    }
                }
                bool valid = !path.empty();
                for (std::size_t i = 0; valid && i < path.size(); ++i) {
                    StateCheck state;
                    valid = c.State(path[i], &state);
                    if (!valid) result.validation_failure = state;
                    if (valid && i) valid = c.Segment(path[i - 1], path[i]);
                }
                if (!valid || c.Stopped()) fail("path_validation_failed", "Final full-path verification did not finish successfully");
                else { result.status = "exact_solution"; result.detail = options.collision.allow_unknown_space ?
                        "All waypoints and segments rechecked against observed obstacles; unknown space allowed" :
                        "All waypoints and joint-linear segments rechecked";
                    result.waypoints = std::move(path); }
            }
        }
    } catch (const std::exception& e) { result.status = "invalid_input"; result.detail = e.what(); }
    result.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    return result;
}
} // namespace uspilot::collision
