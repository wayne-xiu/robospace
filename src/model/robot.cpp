#include <robospace/model/robot.hpp>
#include <stdexcept>

namespace robospace {
namespace model {

Robot::Robot(const std::string& name)
    : Entity(name, Entity::Type::ROBOT) {}

Robot::Robot(const std::string& name, Entity* parent)
    : Entity(name, Entity::Type::ROBOT) {
    set_parent(parent);
}

// Pose overrides (TCP semantics)
math::SE3 Robot::pose() const {
    // Compute TCP pose via FK: FK_origin * base_frame * FK(q) * flange * active_tool
    if (num_links() == 0) {
        return Entity::pose();  // No links, return FK origin
    }

    // Compute FK to get flange pose
    const_cast<KinematicTree&>(tree_).compute_forward_kinematics();
    math::SE3 flange_pose = base_frame_ * tree_.link_pose(flange_link_id());

    // Add active tool TCP if present
    if (has_active_tool()) {
        flange_pose = flange_pose * active_tool().tcp_pose();
    }

    // Transform to world coordinates
    return Entity::pose() * flange_pose;
}

void Robot::set_pose(const math::SE3& tcp_target) {
    (void)tcp_target;  // Suppress unused parameter warning
    // TODO: Implement IK to compute joint configuration for tcp_target
    // For now, throw not implemented
    throw std::runtime_error("IK not yet implemented. Use kinematic_tree().set_configuration() to set joint angles directly.");
}

void Robot::move_base_by(const math::SE3& delta) {
    // Move FK origin by relative amount
    Entity::set_pose(Entity::pose() * delta);
}

// Building
void Robot::add_link(const Link& link) {
    int id = tree_.num_links();
    link_name_to_id_[link.name()] = id;
    tree_.add_link(link);
}

void Robot::add_joint(const Joint& joint) {
    int id = tree_.num_joints();
    joint_name_to_id_[joint.name()] = id;
    tree_.add_joint(joint);
}

// Tool management
int Robot::add_tool(const Tool& tool) {
    if (tool_name_to_id_.count(tool.name()) > 0) {
        throw std::runtime_error("Tool with name '" + tool.name() + "' already exists");
    }

    int tool_id = static_cast<int>(tools_.size());
    Tool tool_copy = tool;
    tool_copy.set_parent(&flange_link());
    tools_.push_back(tool_copy);
    tool_name_to_id_[tool.name()] = tool_id;

    return tool_id;
}

const Tool& Robot::tool(int id) const {
    if (id < 0 || id >= num_tools()) {
        throw std::out_of_range("Tool ID out of range: " + std::to_string(id));
    }
    return tools_[id];
}

Tool& Robot::tool(int id) {
    if (id < 0 || id >= num_tools()) {
        throw std::out_of_range("Tool ID out of range: " + std::to_string(id));
    }
    return tools_[id];
}

const Tool& Robot::tool(const std::string& name) const {
    return tool(tool_id(name));
}

Tool& Robot::tool(const std::string& name) {
    return tool(tool_id(name));
}

int Robot::tool_id(const std::string& name) const {
    auto it = tool_name_to_id_.find(name);
    if (it == tool_name_to_id_.end()) {
        throw std::runtime_error("Tool not found: " + name);
    }
    return it->second;
}

bool Robot::has_tool(const std::string& name) const {
    return tool_name_to_id_.count(name) > 0;
}

// Link/Joint accessors by ID (const)
const Link& Robot::link(int id) const {
    if (id < 0 || id >= num_links()) {
        throw std::out_of_range("Link ID out of range: " + std::to_string(id));
    }
    return tree_.link(id);
}

const Joint& Robot::joint(int id) const {
    if (id < 0 || id >= num_joints()) {
        throw std::out_of_range("Joint ID out of range: " + std::to_string(id));
    }
    return tree_.joint(id);
}

// Link/Joint accessors by ID (non-const)
Link& Robot::link(int id) {
    if (id < 0 || id >= num_links()) {
        throw std::out_of_range("Link ID out of range: " + std::to_string(id));
    }
    return tree_.link(id);
}

Joint& Robot::joint(int id) {
    if (id < 0 || id >= num_joints()) {
        throw std::out_of_range("Joint ID out of range: " + std::to_string(id));
    }
    return tree_.joint(id);
}

// Link/Joint accessors by name
const Link& Robot::link(const std::string& name) const {
    return link(link_id(name));
}

const Joint& Robot::joint(const std::string& name) const {
    return joint(joint_id(name));
}

Link& Robot::link(const std::string& name) {
    return link(link_id(name));
}

Joint& Robot::joint(const std::string& name) {
    return joint(joint_id(name));
}

// Link/Joint lookups
int Robot::link_id(const std::string& name) const {
    auto it = link_name_to_id_.find(name);
    if (it == link_name_to_id_.end()) {
        throw std::runtime_error("Link not found: " + name);
    }
    return it->second;
}

int Robot::joint_id(const std::string& name) const {
    auto it = joint_name_to_id_.find(name);
    if (it == joint_name_to_id_.end()) {
        throw std::runtime_error("Joint not found: " + name);
    }
    return it->second;
}

bool Robot::has_link(const std::string& name) const {
    return link_name_to_id_.count(name) > 0;
}

bool Robot::has_joint(const std::string& name) const {
    return joint_name_to_id_.count(name) > 0;
}

// Counts
int Robot::num_positions() const {
    int count = 0;
    for (int i = 0; i < num_joints(); ++i) {
        if (!joint(i).is_fixed()) {
            count++;
        }
    }
    return count;
}

// Base frame
void Robot::set_base_frame(const math::SE3& frame) {
    base_frame_ = frame;
}

// Active tool management
const Tool& Robot::active_tool() const {
    if (active_tool_id_ < 0) {
        throw std::runtime_error("Active tool not set");
    }
    return tool(active_tool_id_);
}

Tool& Robot::active_tool() {
    if (active_tool_id_ < 0) {
        throw std::runtime_error("Active tool not set");
    }
    return tool(active_tool_id_);
}

void Robot::set_active_tool(int tool_id) {
    if (tool_id < 0 || tool_id >= num_tools()) {
        throw std::out_of_range("Invalid tool ID: " + std::to_string(tool_id));
    }
    active_tool_id_ = tool_id;
}

void Robot::set_active_tool(const std::string& tool_name) {
    active_tool_id_ = tool_id(tool_name);
}

} // namespace model
} // namespace robospace
