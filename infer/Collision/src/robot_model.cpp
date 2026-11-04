#include "collision/robot_model.hpp"

#include <expat.h>
#include <openssl/sha.h>
#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <locale>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

namespace uspilot::collision {
namespace {

struct Node {
    std::string tag;
    std::map<std::string, std::string> attributes;
    std::vector<std::unique_ptr<Node>> children;
};

struct XmlContext {
    XML_Parser parser = nullptr;
    std::unique_ptr<Node> root;
    std::vector<Node*> stack;
    std::exception_ptr failure;
    std::size_t count = 0;
};

void Start(void* data, const XML_Char* tag, const XML_Char** attributes) noexcept {
    auto& c = *static_cast<XmlContext*>(data);
    try {
        if (++c.count > 100000 || c.stack.size() >= 64)
            throw std::runtime_error("URDF XML exceeds node/depth budget");
        const std::string name(tag);
        if (name.find(':') != std::string::npos)
            throw std::runtime_error("Xacro/namespaced elements are not supported; load expanded .urdf");
        auto node = std::make_unique<Node>();
        node->tag = name;
        for (std::size_t i = 0; attributes[i]; i += 2) {
            const std::string value(attributes[i + 1]);
            if (value.find("${") != std::string::npos || value.find("$(") != std::string::npos)
                throw std::runtime_error("Unexpanded Xacro expression");
            node->attributes.emplace(attributes[i], value);
        }
        Node* raw = node.get();
        if (c.stack.empty()) {
            if (c.root) throw std::runtime_error("Multiple XML roots");
            c.root = std::move(node);
        } else {
            c.stack.back()->children.push_back(std::move(node));
        }
        c.stack.push_back(raw);
    } catch (...) {
        c.failure = std::current_exception();
        XML_StopParser(c.parser, XML_FALSE);
    }
}

void End(void* data, const XML_Char*) noexcept {
    auto& c = *static_cast<XmlContext*>(data);
    if (!c.stack.empty()) c.stack.pop_back();
}

void Doctype(void* data, const XML_Char*, const XML_Char*, const XML_Char*, int) noexcept {
    auto& c = *static_cast<XmlContext*>(data);
    try { throw std::runtime_error("DTD/entities are not supported in URDF"); }
    catch (...) { c.failure = std::current_exception(); }
    XML_StopParser(c.parser, XML_FALSE);
}

std::string Attribute(const Node& n, const std::string& key,
                      const std::string& fallback = "", bool required = false) {
    const auto it = n.attributes.find(key);
    if (it == n.attributes.end() || it->second.empty()) {
        if (required) throw std::runtime_error(n.tag + " requires attribute " + key);
        return fallback;
    }
    return it->second;
}

const Node* Child(const Node& n, const std::string& tag, bool required = false) {
    const Node* result = nullptr;
    for (const auto& child : n.children) if (child->tag == tag) {
        if (result) throw std::runtime_error("Duplicate " + tag + " in " + n.tag);
        result = child.get();
    }
    if (required && !result) throw std::runtime_error(n.tag + " requires " + tag);
    return result;
}

std::vector<double> Numbers(const std::string& text, std::size_t count) {
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    std::vector<double> result;
    double x;
    while (stream >> x) {
        if (!std::isfinite(x)) throw std::runtime_error("Non-finite URDF number");
        result.push_back(x);
    }
    if (!stream.eof() || result.size() != count)
        throw std::runtime_error("Expected " + std::to_string(count) + " finite numbers: " + text);
    return result;
}

double Number(const Node& n, const std::string& key) {
    return Numbers(Attribute(n, key, "", true), 1)[0];
}

Eigen::Vector3d Vector(const std::string& text) {
    const auto v = Numbers(text, 3);
    return {v[0], v[1], v[2]};
}

Eigen::Isometry3d Origin(const Node& parent) {
    const Node* n = Child(parent, "origin");
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    if (!n) return result;
    result.translation() = Vector(Attribute(*n, "xyz", "0 0 0"));
    const auto rpy = Vector(Attribute(*n, "rpy", "0 0 0"));
    result.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    return result;
}

Geometry ParseGeometry(const Node& n) {
    Geometry g;
    g.name = Attribute(n, "name");
    g.link_from_geometry = Origin(n);
    const Node& geometry = *Child(n, "geometry", true);
    if (geometry.children.size() != 1) throw std::runtime_error("geometry requires exactly one shape");
    const Node& s = *geometry.children[0];
    if (s.tag == "box") {
        g.shape = Shape::kBox;
        g.box_size = Vector(Attribute(s, "size", "", true));
        if ((g.box_size.array() <= 0).any()) throw std::runtime_error("Box dimensions must be positive");
    } else if (s.tag == "sphere") {
        g.shape = Shape::kSphere;
        g.radius = Number(s, "radius");
        if (g.radius <= 0) throw std::runtime_error("Sphere radius must be positive");
    } else if (s.tag == "cylinder") {
        g.shape = Shape::kCylinder;
        g.radius = Number(s, "radius");
        g.length = Number(s, "length");
        if (g.radius <= 0 || g.length <= 0) throw std::runtime_error("Cylinder dimensions must be positive");
    } else if (s.tag == "mesh") {
        g.shape = Shape::kMesh;
        g.mesh_uri = Attribute(s, "filename", "", true);
        g.mesh_scale = Vector(Attribute(s, "scale", "1 1 1"));
        if ((g.mesh_scale.array() <= 0).any()) throw std::runtime_error("Only positive mesh scales are supported");
    } else throw std::runtime_error("Unsupported collision shape: " + s.tag);
    return g;
}

Joint ParseJoint(const Node& n) {
    Joint j;
    j.name = Attribute(n, "name", "", true);
    const auto type = Attribute(n, "type", "", true);
    if (type == "fixed") j.type = JointType::kFixed;
    else if (type == "revolute") j.type = JointType::kRevolute;
    else if (type == "continuous") j.type = JointType::kContinuous;
    else if (type == "prismatic") j.type = JointType::kPrismatic;
    else throw std::runtime_error("Unsupported joint type: " + type);
    if (Child(n, "mimic")) throw std::runtime_error("Mimic joints are not supported");
    j.parent = Attribute(*Child(n, "parent", true), "link", "", true);
    j.child = Attribute(*Child(n, "child", true), "link", "", true);
    j.parent_from_joint = Origin(n);
    if (j.type != JointType::kFixed) {
        if (const auto* axis = Child(n, "axis"))
            j.axis = Vector(Attribute(*axis, "xyz", "1 0 0"));
        if (j.axis.norm() < 1e-12 || !std::isfinite(j.axis.norm()))
            throw std::runtime_error("Invalid joint axis: " + j.name);
        j.axis.normalize();
        const Node& limit = *Child(n, "limit", true);
        j.velocity = Number(limit, "velocity");
        if (j.velocity <= 0) throw std::runtime_error("Joint velocity must be positive");
        if (j.type != JointType::kContinuous) {
            j.lower = Number(limit, "lower");
            j.upper = Number(limit, "upper");
            if (j.lower >= j.upper) throw std::runtime_error("Invalid joint bounds: " + j.name);
        }
    }
    return j;
}

}  // namespace

RobotModel RobotModel::Parse(const std::string& xml) {
    if (xml.empty() || xml.size() > 16 * 1024 * 1024)
        throw std::runtime_error("Empty or oversized URDF");
    std::unique_ptr<XML_ParserStruct, decltype(&XML_ParserFree)> parser(XML_ParserCreate(nullptr), XML_ParserFree);
    if (!parser) throw std::runtime_error("Cannot create XML parser");
    XmlContext c;
    c.parser = parser.get();
    XML_SetUserData(c.parser, &c);
    XML_SetElementHandler(c.parser, Start, End);
    XML_SetStartDoctypeDeclHandler(c.parser, Doctype);
    XML_SetParamEntityParsing(c.parser, XML_PARAM_ENTITY_PARSING_NEVER);
    const auto status = XML_Parse(c.parser, xml.data(), static_cast<int>(xml.size()), XML_TRUE);
    if (c.failure) std::rethrow_exception(c.failure);
    if (status != XML_STATUS_OK) throw std::runtime_error("Invalid URDF XML at line "
        + std::to_string(XML_GetCurrentLineNumber(c.parser)) + ": " + XML_ErrorString(XML_GetErrorCode(c.parser)));
    if (!c.root || c.root->tag != "robot") throw std::runtime_error("URDF root must be robot");
    RobotModel model;
    model.name_ = Attribute(*c.root, "name", "", true);
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(xml.data()), xml.size(), digest);
    std::ostringstream hash;
    for (unsigned char b : digest) hash << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
    model.sha256_ = hash.str();
    std::set<std::string> link_names, joint_names, children;
    std::vector<Joint> unsorted;
    for (const auto& n : c.root->children) {
        if (n->tag == "link") {
            Link link;
            link.name = Attribute(*n, "name", "", true);
            if (!link_names.insert(link.name).second) throw std::runtime_error("Duplicate link: " + link.name);
            for (const auto& child : n->children) if (child->tag == "collision")
                link.collisions.push_back(ParseGeometry(*child));
            model.links_.push_back(std::move(link));
        } else if (n->tag == "joint") {
            auto j = ParseJoint(*n);
            if (!joint_names.insert(j.name).second) throw std::runtime_error("Duplicate joint: " + j.name);
            if (!children.insert(j.child).second) throw std::runtime_error("Multiple parents for link: " + j.child);
            unsorted.push_back(std::move(j));
        }
    }
    if (link_names.empty()) throw std::runtime_error("URDF has no links");
    for (const auto& j : unsorted) if (!link_names.count(j.parent) || !link_names.count(j.child))
        throw std::runtime_error("Joint references unknown link: " + j.name);
    for (const auto& name : link_names) if (!children.count(name)) {
        if (!model.root_.empty()) throw std::runtime_error("URDF must have one connected root");
        model.root_ = name;
    }
    if (model.root_.empty()) throw std::runtime_error("URDF has no root (cycle)");
    std::map<std::string, std::vector<const Joint*>> adjacency;
    for (const auto& j : unsorted) adjacency[j.parent].push_back(&j);
    std::set<std::string> visited{model.root_};
    std::vector<std::string> pending{model.root_};
    for (std::size_t i = 0; i < pending.size(); ++i) {
        const auto found = adjacency.find(pending[i]);
        if (found == adjacency.end()) continue;
        for (const Joint* j : found->second) {
            if (!visited.insert(j->child).second) throw std::runtime_error("URDF cycle at " + j->child);
            model.joints_.push_back(*j);
            if (j->type != JointType::kFixed) model.active_names_.push_back(j->name);
            pending.push_back(j->child);
        }
    }
    if (visited.size() != link_names.size()) throw std::runtime_error("URDF contains disconnected links/cycle");
    return model;
}

RobotModel RobotModel::Load(const std::filesystem::path& path) {
    if (path.extension() == ".xacro") throw std::runtime_error("Load expanded .urdf; Xacro execution is not supported");
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream || stream.tellg() < 0 || stream.tellg() > 16 * 1024 * 1024)
        throw std::runtime_error("Cannot read URDF or file exceeds 16 MiB: " + path.string());
    std::string text(static_cast<std::size_t>(stream.tellg()), '\0');
    stream.seekg(0);
    if (!stream.read(text.data(), static_cast<std::streamsize>(text.size()))) throw std::runtime_error("Cannot read URDF");
    auto model = Parse(text);
    model.source_directory_ = std::filesystem::absolute(path).parent_path();
    return model;
}

LinkPoses RobotModel::Forward(const JointPositions& positions) const {
    if (positions.size() != active_names_.size()) throw std::runtime_error("Joint set must exactly match active URDF joints");
    LinkPoses poses;
    poses.emplace(root_, Eigen::Isometry3d::Identity());
    for (const auto& j : joints_) {
        Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
        if (j.type != JointType::kFixed) {
            const auto it = positions.find(j.name);
            if (it == positions.end() || !std::isfinite(it->second)) throw std::runtime_error("Missing/non-finite joint: " + j.name);
            const double q = it->second;
            if (j.type != JointType::kContinuous && (q < j.lower || q > j.upper))
                throw std::runtime_error("Joint outside URDF bounds: " + j.name);
            if (j.type == JointType::kPrismatic) motion.translation() = j.axis * q;
            else motion.linear() = Eigen::AngleAxisd(q, j.axis).toRotationMatrix();
        }
        const Eigen::Isometry3d pose = poses.at(j.parent) * j.parent_from_joint * motion;
        if (!pose.matrix().allFinite()) throw std::runtime_error("Non-finite FK result");
        poses.emplace(j.child, pose);
    }
    return poses;
}

Eigen::Isometry3d RobotModel::CollisionPose(const LinkPoses& poses, const std::string& name,
                                           std::size_t index) const {
    const auto it = std::find_if(links_.begin(), links_.end(), [&](const Link& l) { return l.name == name; });
    if (it == links_.end()) throw std::runtime_error("Unknown collision link: " + name);
    return poses.at(name) * it->collisions.at(index).link_from_geometry;
}

}  // namespace uspilot::collision
