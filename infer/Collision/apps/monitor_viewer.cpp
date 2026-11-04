#include "monitor_viewer.hpp"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <octomap/OcTree.h>
#include <set>
#include <sstream>
#include <stdexcept>

namespace c = uspilot::collision;
struct MonitorViewer::Impl {
    struct Body {
        std::string link, id;
        std::size_t index;
        c::Geometry geometry;
        c::TriangleMesh mesh;
    };
    c::RobotModel model;
    std::vector<Body> bodies;
    std::vector<Eigen::Vector3d> path_points;
    bool path_stale = false;
    Eigen::Vector3d view_target = Eigen::Vector3d(0, 0, .55);
    std::vector<c::CollisionBodyPreview> goal_bodies;
    c::StateCheck goal_check;
    c::CollisionScene goal_scene;
    c::JointPositions captured_joints;
    Eigen::Vector3d goal_tcp = Eigen::Vector3d::Zero();
    bool goal_visible = false;
    std::string planning_status;
    bool plan_requested = false;
    GLFWwindow *window = nullptr;
    double yaw = -.8, pitch = .3, distance = 2.6, last_x = 0, last_y = 0;
    bool dragging = false;
    ~Impl() {
        if (window)
            glfwDestroyWindow(window);
        glfwTerminate();
    }
};
MonitorViewer::MonitorViewer(c::RobotModel model, const c::MeshDirectories &directories, bool drop)
    : impl_(std::make_unique<Impl>()) {
    auto &p = *impl_;
    p.model = std::move(model);
    for (const auto &l : p.model.Links())
        for (std::size_t i = 0; i < l.collisions.size(); ++i) {
            Impl::Body b{l.name, l.name + "/collision/" + std::to_string(i), i, l.collisions[i], {}};
            if (b.geometry.shape == c::Shape::kMesh)
                b.mesh = c::LoadStl(
                    c::ResolveMesh(b.geometry.mesh_uri, p.model.SourceDirectory(), {}, directories),
                    b.geometry.mesh_scale, {drop});
            p.bodies.push_back(std::move(b));
        }
    if (!glfwInit())
        throw std::runtime_error("GLFW init failed; use --headless without a desktop");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    p.window = glfwCreateWindow(1100, 800, "Collision monitor - READ ONLY", nullptr, nullptr);
    if (!p.window)
        throw std::runtime_error("Cannot create OpenGL window; use --headless");
    glfwMakeContextCurrent(p.window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(p.window, &p);
    glfwSetKeyCallback(p.window, [](GLFWwindow* w, int key, int, int action, int) {
        if (key == GLFW_KEY_P && action == GLFW_PRESS)
            static_cast<Impl*>(glfwGetWindowUserPointer(w))->plan_requested = true;
        if (key == GLFW_KEY_G && action == GLFW_PRESS) {
            auto* p = static_cast<Impl*>(glfwGetWindowUserPointer(w));
            p->goal_visible = !p->goal_visible;
        }
    });
    glfwSetScrollCallback(p.window, [](GLFWwindow *w, double, double y) {
        auto *v = static_cast<Impl *>(glfwGetWindowUserPointer(w));
        v->distance = std::clamp(v->distance * std::exp(-y * .1), .25, 15.);
    });
    glfwSetCursorPosCallback(p.window, [](GLFWwindow *w, double x, double y) {
        auto *v = static_cast<Impl *>(glfwGetWindowUserPointer(w));
        if (v->dragging) {
            v->yaw += (x - v->last_x) * .008;
            v->pitch = std::clamp(v->pitch + (y - v->last_y) * .008, -1.4, 1.4);
        }
        v->last_x = x;
        v->last_y = y;
    });
    glfwSetMouseButtonCallback(p.window, [](GLFWwindow *w, int button, int action, int) {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            static_cast<Impl *>(glfwGetWindowUserPointer(w))->dragging = action == GLFW_PRESS;
    });
}
MonitorViewer::~MonitorViewer() = default;
void MonitorViewer::SetGoalPreview(std::vector<c::CollisionBodyPreview> bodies,
        const c::StateCheck& check, const c::CollisionScene& scene,
        const c::JointPositions& captured_joints, const Eigen::Vector3d& tcp) {
    auto& p = *impl_;
    p.goal_bodies = std::move(bodies);
    p.goal_check = check;
    p.goal_scene = scene;
    p.captured_joints = captured_joints;
    p.goal_tcp = tcp;
    p.goal_visible = true;
}
void MonitorViewer::ClearGoalPreview() {
    impl_->goal_bodies.clear();
    impl_->goal_scene = {};
    impl_->goal_check = {};
    impl_->captured_joints.clear();
    impl_->goal_visible = false;
}
void MonitorViewer::SetPlanningStatus(const std::string& status) { impl_->planning_status = status; }
bool MonitorViewer::TakePlanningRequest() {
    const bool requested = impl_->plan_requested;
    impl_->plan_requested = false;
    return requested;
}
void MonitorViewer::MarkPlannedPathStale() { impl_->path_stale = true; }
void MonitorViewer::SetPlannedPath(const std::vector<c::JointPositions>& path,
        const std::string& tip_link, const Eigen::Isometry3d& tip_from_tcp) {
    auto& p = *impl_;
    p.path_points.clear();
    p.path_stale = false;
    if (path.empty()) return;
    auto tip = tip_link;
    if (tip.empty()) {
        const bool rm75 = std::any_of(p.model.Links().begin(), p.model.Links().end(),
            [](const c::Link& link) { return link.name == "Link7"; });
        tip = rm75 ? "Link7" : (p.model.Joints().empty() ? p.model.Root() : p.model.Joints().back().child);
    }
    auto append = [&](const c::JointPositions& q) {
        if (p.path_points.size() >= 100000) throw std::runtime_error("Path preview exceeds sample budget");
        p.path_points.push_back((p.model.Forward(q).at(tip) * tip_from_tcp).translation());
    };
    append(path.front());
    for (std::size_t i = 1; i < path.size(); ++i) {
        double maximum = 0;
        for (const auto& [name, value] : path[i-1]) maximum = std::max(maximum, std::abs(path[i].at(name)-value));
        const auto steps = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(maximum / .02)));
        for (std::size_t j = 1; j <= steps; ++j) {
            const double fraction = double(j) / steps;
            c::JointPositions q;
            for (const auto& [name, value] : path[i-1]) q[name] = value + fraction * (path[i].at(name)-value);
            append(q);
        }
    }
    Eigen::Vector3d minimum = Eigen::Vector3d(0, 0, .55), maximum = minimum;
    for (const auto& point : p.path_points) {
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
    }
    p.view_target = (minimum + maximum) / 2;
    p.distance = std::max(p.distance, (maximum-minimum).norm() / .7 + .3);
}
namespace {
void Cube(const Eigen::Vector3d &size) {
    const std::array<std::array<int, 4>, 6> faces = {
        {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}}};
    glBegin(GL_QUADS);
    for (const auto &f : faces)
        for (int i : f)
            glVertex3d((i & 1 ? .5 : -.5) * size.x(), (i & 2 ? .5 : -.5) * size.y(),
                       (i & 4 ? .5 : -.5) * size.z());
    glEnd();
}
void SolidCylinder(double radius, double length) {
    constexpr int segments = 48;
    const double half = length / 2;
    const double step = 2 * std::acos(-1.) / segments;
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        const double x = radius * std::cos(i * step), y = radius * std::sin(i * step);
        glVertex3d(x, y, half);
        glVertex3d(x, y, -half);
    }
    glEnd();
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(0, 0, half);
    for (int i = 0; i <= segments; ++i)
        glVertex3d(radius * std::cos(i * step), radius * std::sin(i * step), half);
    glEnd();
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(0, 0, -half);
    for (int i = segments; i >= 0; --i)
        glVertex3d(radius * std::cos(i * step), radius * std::sin(i * step), -half);
    glEnd();
}
std::string Distance(double d) {
    if (!std::isfinite(d))
        return "n/a";
    std::ostringstream o;
    o.precision(3);
    o << d << "m";
    return o.str();
}
} // namespace
bool MonitorViewer::Draw(const c::MonitorFrame &frame, const c::MonitorResult &result) {
    auto &p = *impl_;
    glfwPollEvents();
    if (glfwWindowShouldClose(p.window) || glfwGetKey(p.window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        return false;
    const double unavailable = std::numeric_limits<double>::infinity();
    std::string title = "READ ONLY | env=" + Distance(result.input_valid ? result.check.minimum_environment_distance_m : unavailable) +
                        " self=" + Distance(result.input_valid ? result.check.minimum_self_distance_m : unavailable) + " | " +
                        c::CheckStatusName(result.check.status) + " " + result.check.detail +
                        " | left drag: orbit, wheel: zoom, Esc: exit";
    if (!result.check.violations.empty()) {
        const auto& pair = result.check.violations.front();
        title += " | " + pair.first_body + " <> " + pair.second_body;
    }
    if (!p.planning_status.empty()) title += " | plan=" + p.planning_status;
    const bool preview = p.goal_visible && !p.goal_bodies.empty();
    if (preview) {
        title = (p.path_points.empty() ? "NO PATH | " : "PATH READY | ") + p.planning_status +
            " | GOAL SNAPSHOT (not live) | " + p.goal_scene.id + " | " + p.goal_check.detail +
            " | orange/red=goal hull, green=goal STL, white=TCP, yellow=nearest pair, G=live, P=replan";
        if (!p.goal_check.violations.empty()) title += " | " +
            p.goal_check.violations.front().first_body + " distance=" + Distance(p.goal_check.violations.front().distance_m);
    } else title += " | G: goal snapshot";
    if (p.path_points.empty() && !p.planning_status.empty() && !preview)
        title = "NO PATH | " + p.planning_status + " | " + title;
    if (!p.path_points.empty()) title += p.path_stale ? " | TCP curve: STALE snapshot" : " | TCP curve: planned snapshot";
    glfwSetWindowTitle(p.window, title.c_str());
    int width, height;
    glfwGetFramebufferSize(p.window, &width, &height);
    if (!width || !height)
        return true;
    glViewport(0, 0, width, height);
    glEnable(GL_DEPTH_TEST);
    glClearColor(.08f, .09f, .12f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const double near = .02, aspect = double(width) / height, half = near * std::tan(.5);
    glFrustum(-half * aspect, half * aspect, -half, half, near, 30);
    const Eigen::Vector3d target = p.view_target,
        eye = target + p.distance * Eigen::Vector3d(std::cos(p.pitch) * std::cos(p.yaw),
                                                    std::cos(p.pitch) * std::sin(p.yaw), std::sin(p.pitch));
    const Eigen::Vector3d forward = (target - eye).normalized(),
                          right = forward.cross(Eigen::Vector3d::UnitZ()).normalized(),
                          up = right.cross(forward);
    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();
    view.block<1, 3>(0, 0) = right.transpose();
    view.block<1, 3>(1, 0) = up.transpose();
    view.block<1, 3>(2, 0) = -forward.transpose();
    view.block<3, 1>(0, 3) = -view.block<3, 3>(0, 0) * eye;
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(view.data());
    glLineWidth(2);
    glBegin(GL_LINES);
    for (int i = 0; i < 3; ++i) {
        glColor3d(i == 0, i == 1, i == 2);
        glVertex3d(0, 0, 0);
        Eigen::Vector3d a = Eigen::Vector3d::Zero();
        a[i] = .3;
        glVertex3dv(a.data());
    }
    glEnd();
    // The planner's surveyed primitive scene is drawn as well as camera voxels.
    glColor3f(.9f, .3f, .65f);
    const auto& display_scene = preview ? p.goal_scene : result.scene;
    for (const auto& obstacle : display_scene.obstacles) {
        const auto& g = obstacle.geometry;
        glPushMatrix(); glMultMatrixd(g.link_from_geometry.matrix().data());
        if (g.shape == c::Shape::kBox) Cube(g.box_size);
        else {
            for (int j = 0; j < 12; ++j) {
                const double z = g.shape == c::Shape::kSphere
                    ? g.radius * std::sin(j * std::acos(-1.) / 11 - std::acos(-1.) / 2)
                    : g.length * (j / 11. - .5);
                const double r = g.shape == c::Shape::kSphere
                    ? std::sqrt(std::max(0., g.radius * g.radius - z * z)) : g.radius;
                glBegin(GL_LINE_LOOP);
                for (int i = 0; i < 40; ++i) {
                    const double a = i * 2 * std::acos(-1.) / 40;
                    glVertex3d(r * std::cos(a), r * std::sin(a), z);
                }
                glEnd();
            }
        }
        glPopMatrix();
    }
    glPointSize(2);
    glBegin(GL_POINTS);
    for (std::size_t i = 0; i < result.points_base_m.size(); ++i) {
        if (preview) break; // Never mix the frozen planning scene with new raw depth.
        if (i < result.occluded_mask.size() && result.occluded_mask[i]) continue;
        if (i < result.outlier_mask.size() && result.outlier_mask[i])
            glColor3f(1.f, .45f, .1f);
        else if (i >= result.robot_mask.size() || result.robot_mask[i])
            glColor3f(.35f, .35f, .35f);
        else
            glColor3f(.15f, .7f, .9f);
        glVertex3dv(result.points_base_m[i].data());
    }
    glEnd();
    if (display_scene.observation && display_scene.observation->tree) {
        glPointSize(4);
        glColor3f(.9f, .3f, .65f);
        glBegin(GL_POINTS);
        const auto &tree = *display_scene.observation->tree;
        for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
            if (tree.isNodeOccupied(*it))
                glVertex3f(it.getX(), it.getY(), it.getZ());
        glEnd();
    }
    std::set<std::string> environment, self;
    if (!preview) for (const auto &v : result.check.violations) {
        if (v.self_collision) {
            self.insert(v.first_body);
            self.insert(v.second_body);
        } else
            environment.insert(v.first_body);
    }
    if (preview || frame.robot_valid) {
        try {
            const auto poses = p.model.Forward(preview ? p.captured_joints : frame.joints);
            for (const auto &b : p.bodies) {
                if (environment.count(b.id))
                    glColor3f(1, .1f, .1f);
                else if (self.count(b.id))
                    glColor3f(.95f, .65f, .15f);
                else
                    glColor3f(.75f, .8f, .85f);
                glPushMatrix();
                const auto transform = p.model.CollisionPose(poses, b.link, b.index);
                glMultMatrixd(transform.matrix().data());
                if (b.geometry.shape == c::Shape::kMesh) {
                    glBegin(GL_TRIANGLES);
                    for (const auto &face : b.mesh.triangles)
                        for (auto index : face)
                            glVertex3dv(b.mesh.vertices[index].data());
                    glEnd();
                } else if (b.geometry.shape == c::Shape::kBox)
                    Cube(b.geometry.box_size);
                else if (b.geometry.shape == c::Shape::kCylinder)
                    SolidCylinder(b.geometry.radius, b.geometry.length);
                else {
                    const double r = b.geometry.radius;
                    for (int j = 0; j < 12; ++j) {
                        glBegin(GL_LINE_LOOP);
                        for (int i = 0; i < 40; ++i) {
                            const double a = i * 2 * std::acos(-1.) / 40,
                                         z = b.geometry.shape == c::Shape::kSphere
                                                 ? r * std::sin(j * std::acos(-1.) / 12 - std::acos(-1.) / 2)
                                                 : b.geometry.length * (j / 11. - .5);
                            const double rr = b.geometry.shape == c::Shape::kSphere
                                                  ? std::sqrt(std::max(0., r * r - z * z))
                                                  : r;
                            glVertex3d(rr * std::cos(a), rr * std::sin(a), z);
                        }
                        glEnd();
                    }
                }
                glPopMatrix();
            }
        } catch (const std::exception &) {
        }
    }
    if (preview) {
        // Render actual FCL hulls as wireframes; the grey model underneath is
        // the original STL at capture time, not a newly observed obstacle.
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.5f);
        for (const auto& b : p.goal_bodies) {
            const bool violated = std::any_of(p.goal_check.violations.begin(), p.goal_check.violations.end(),
                [&](const c::PairViolation& pair) { return pair.first_body == b.id || pair.second_body == b.id; });
            glColor3f(1.f, violated ? .15f : .65f, .05f);
            glPushMatrix();
            glMultMatrixd(b.base_from_body.matrix().data());
            if (violated && b.geometry.shape == c::Shape::kMesh) {
                const auto source = std::find_if(p.bodies.begin(), p.bodies.end(),
                    [&](const Impl::Body& original) { return original.id == b.id; });
                if (source != p.bodies.end()) {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                    glColor3f(.2f, .85f, .35f);
                    glBegin(GL_TRIANGLES);
                    for (const auto& face : source->mesh.triangles)
                        for (auto index : face) glVertex3dv(source->mesh.vertices[index].data());
                    glEnd();
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    glColor3f(1.f, .15f, .05f);
                }
            }
            if (b.geometry.shape == c::Shape::kMesh) {
                glBegin(GL_TRIANGLES);
                for (const auto& face : b.mesh.triangles)
                    for (auto index : face) glVertex3dv(b.mesh.vertices[index].data());
                glEnd();
            } else if (b.geometry.shape == c::Shape::kBox) Cube(b.geometry.box_size);
            else if (b.geometry.shape == c::Shape::kCylinder) SolidCylinder(b.geometry.radius, b.geometry.length);
            else {
                for (int j = 0; j < 12; ++j) {
                    const double z = b.geometry.radius * std::sin(j * std::acos(-1.) / 11 - std::acos(-1.) / 2);
                    const double r = std::sqrt(std::max(0., b.geometry.radius*b.geometry.radius-z*z));
                    glBegin(GL_LINE_LOOP);
                    for (int i = 0; i < 40; ++i) {
                        const double angle = i * 2 * std::acos(-1.) / 40;
                        glVertex3d(r*std::cos(angle), r*std::sin(angle), z);
                    }
                    glEnd();
                }
            }
            glPopMatrix();
        }
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDisable(GL_DEPTH_TEST);
        glPointSize(10);
        glColor3f(1, 1, 1);
        glBegin(GL_POINTS); glVertex3dv(p.goal_tcp.data()); glEnd();
        for (const auto& pair : p.goal_check.violations) {
            if (pair.self_collision || !pair.first_point_base_m || !pair.second_point_base_m) continue;
            glColor3f(1, 1, 0);
            glLineWidth(4);
            glBegin(GL_LINES);
            glVertex3dv(pair.first_point_base_m->data());
            glVertex3dv(pair.second_point_base_m->data());
            glEnd();
            glPointSize(9);
            glBegin(GL_POINTS);
            glVertex3dv(pair.first_point_base_m->data());
            glVertex3dv(pair.second_point_base_m->data());
            glEnd();
            if (p.goal_scene.observation && p.goal_scene.observation->tree) {
                const auto& tree = *p.goal_scene.observation->tree;
                glColor3f(1, .6f, .1f);
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
                    if (tree.isNodeOccupied(*it) &&
                        (Eigen::Vector3d(it.getX(), it.getY(), it.getZ()) - *pair.second_point_base_m).norm() <= .05) {
                        glPushMatrix();
                        glTranslated(it.getX(), it.getY(), it.getZ());
                        Cube(Eigen::Vector3d::Constant(it.getSize()));
                        glPopMatrix();
                    }
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }
        }
        glEnable(GL_DEPTH_TEST);
    }
    if (!p.path_points.empty()) {
        // Sample the checked joint-linear path through FK. No Cartesian spline
        // smoothing: that would display a different, unchecked motion.
        glDisable(GL_DEPTH_TEST);
        if (p.path_stale) glColor3f(1.f, .65f, .15f);
        else glColor3f(.1f, 1.f, .85f);
        glLineWidth(4);
        glBegin(GL_LINE_STRIP);
        for (const auto& point : p.path_points) glVertex3dv(point.data());
        glEnd();
        glPointSize(10);
        glBegin(GL_POINTS);
        glColor3f(.2f, 1.f, .2f); glVertex3dv(p.path_points.front().data());
        glColor3f(1.f, .2f, .8f); glVertex3dv(p.path_points.back().data());
        glEnd();
        glEnable(GL_DEPTH_TEST);
    }
    glfwSwapBuffers(p.window);
    return true;
}
