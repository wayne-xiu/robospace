#include <robospace/model/robot.hpp>
#include <robospace/model/urdf_parser.hpp>
#include <stdexcept>

namespace robospace {
namespace model {

Robot::Robot(const std::string& name, Entity* parent)
    : Entity(name, Entity::Type::ROBOT, parent) {}

int Robot::dof() const {
    int count = 0;
    for (int i = 0; i < num_joints(); ++i) {
        if (!joint(i).is_fixed()) {
            count++;
        }
    }
    return count;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Robot::joint_limits() const {
    Eigen::VectorXd qmin(num_joints());
    Eigen::VectorXd qmax(num_joints());

    for (int i = 0; i < num_joints(); ++i) {
        qmin(i) = joint(i).lower_limit();
        qmax(i) = joint(i).upper_limit();
    }

    return {qmin, qmax};
}

void Robot::set_home(const Eigen::VectorXd& q) {
    if (q.size() != dof()) {
        throw std::invalid_argument("Home configuration size must match dof");
    }
    home_position_ = q;
}

// Forward kinematics (stateless)
math::SE3 Robot::fk(const Eigen::VectorXd& q) const {
    if (num_links() == 0) {
        return base_frame_;
    }

    int flange_id = num_links() - 1;
    math::SE3 flange_pose = tree_.compute_link_pose(q, flange_id);
    math::SE3 tcp_pose = base_frame_ * flange_pose;

    if (has_active_tool()) {
        tcp_pose = tcp_pose * active_tool().tcp_pose();
    }

    return tcp_pose;
}

math::SE3 Robot::fk(const Eigen::VectorXd& q, const std::string& link_name) const {
    auto it = link_name_to_id_.find(link_name);
    if (it == link_name_to_id_.end()) {
        throw std::runtime_error("Link not found: " + link_name);
    }
    return fk(q, it->second);
}

math::SE3 Robot::fk(const Eigen::VectorXd& q, int link_id) const {
    if (link_id < 0 || link_id >= num_links()) {
        throw std::out_of_range("Link ID out of range");
    }
    math::SE3 link_pose = tree_.compute_link_pose(q, link_id);
    return base_frame_ * link_pose;
}

std::vector<math::SE3> Robot::fk_all(const Eigen::VectorXd& q) const {
    std::vector<math::SE3> poses = tree_.compute_forward_kinematics(q);
    for (auto& pose : poses) {
        pose = base_frame_ * pose;
    }
    return poses;
}

// Forward kinematics (stateful)
math::SE3 Robot::current_pose() const {
    if (tree_.configuration().size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return fk(tree_.configuration());
}

math::SE3 Robot::current_pose(const std::string& link_name) const {
    if (tree_.configuration().size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return fk(tree_.configuration(), link_name);
}

math::SE3 Robot::current_pose(int link_id) const {
    if (tree_.configuration().size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return fk(tree_.configuration(), link_id);
}

std::vector<math::SE3> Robot::current_pose_all() const {
    if (tree_.configuration().size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return fk_all(tree_.configuration());
}

// Differential kinematics
Eigen::MatrixXd Robot::jacob0(const Eigen::VectorXd& q) const {
    Eigen::MatrixXd J_tree = tree_.compute_jacobian_base(q);
    return base_frame_.adjoint() * J_tree;
}

Eigen::MatrixXd Robot::jacobe(const Eigen::VectorXd& q) const {
    Eigen::MatrixXd J_flange = tree_.compute_jacobian_ee(q);

    if (!has_active_tool()) {
        return J_flange;
    }

    math::SE3 T_tool_inv = active_tool().tcp_pose().inverse();
    return T_tool_inv.adjoint() * J_flange;
}

Eigen::MatrixXd Robot::jacob0() const {
    if (tree_.configuration().size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return jacob0(tree_.configuration());
}

Eigen::MatrixXd Robot::jacobe() const {
    if (tree_.configuration().size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return jacobe(tree_.configuration());
}

void Robot::set_base(const math::SE3& base) {
    base_frame_ = base;
}

int Robot::add_tool(const Tool& tool) {
    auto it = tool_name_to_id_.find(tool.name());
    if (it != tool_name_to_id_.end()) {
        throw std::runtime_error("Tool with name '" + tool.name() + "' already exists");
    }

    int tool_id = static_cast<int>(tools_.size());
    Tool tool_copy = tool;

    if (num_links() > 0) {
        int flange_id = num_links() - 1;
        tool_copy.set_parent(&tree_.link(flange_id));
    }

    tools_.push_back(tool_copy);
    tool_name_to_id_[tool.name()] = tool_id;

    return tool_id;
}

const Tool& Robot::tool(int id) const {
    if (id < 0 || id >= num_tools()) {
        throw std::out_of_range("Tool ID out of range");
    }
    return tools_[id];
}

const Tool& Robot::tool(const std::string& name) const {
    auto it = tool_name_to_id_.find(name);
    if (it == tool_name_to_id_.end()) {
        throw std::runtime_error("Tool not found: " + name);
    }
    return tool(it->second);
}

void Robot::set_active_tool(int id) {
    if (id < 0 || id >= num_tools()) {
        throw std::out_of_range("Invalid tool ID");
    }
    active_tool_id_ = id;
}

void Robot::set_active_tool(const std::string& name) {
    auto it = tool_name_to_id_.find(name);
    if (it == tool_name_to_id_.end()) {
        throw std::runtime_error("Tool not found: " + name);
    }
    active_tool_id_ = it->second;
}

const Tool& Robot::active_tool() const {
    if (active_tool_id_ < 0) {
        throw std::runtime_error("Active tool not set");
    }
    return tool(active_tool_id_);
}

const Link& Robot::link(int id) const {
    if (id < 0 || id >= num_links()) {
        throw std::out_of_range("Link ID out of range");
    }
    return tree_.link(id);
}

const Link& Robot::link(const std::string& name) const {
    auto it = link_name_to_id_.find(name);
    if (it == link_name_to_id_.end()) {
        throw std::runtime_error("Link not found: " + name);
    }
    return link(it->second);
}

bool Robot::has_link(const std::string& name) const {
    return link_name_to_id_.find(name) != link_name_to_id_.end();
}

const Joint& Robot::joint(int id) const {
    if (id < 0 || id >= num_joints()) {
        throw std::out_of_range("Joint ID out of range");
    }
    return tree_.joint(id);
}

const Joint& Robot::joint(const std::string& name) const {
    auto it = joint_name_to_id_.find(name);
    if (it == joint_name_to_id_.end()) {
        throw std::runtime_error("Joint not found: " + name);
    }
    return joint(it->second);
}

bool Robot::has_joint(const std::string& name) const {
    return joint_name_to_id_.find(name) != joint_name_to_id_.end();
}

math::SE3 Robot::pose() const {
    if (num_links() == 0) {
        return Entity::pose();
    }
    if (tree_.configuration().size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return Entity::pose() * current_pose();
}

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

Robot Robot::from_urdf(const std::string& urdf_path) {
    return URDFParser::parse_file(urdf_path);
}

Robot Robot::from_urdf_string(const std::string& urdf_string) {
    return URDFParser::parse_string(urdf_string);
}

double Robot::manipulability(const Eigen::VectorXd& q) const {
    Eigen::MatrixXd J = jacob0(q);
    Eigen::MatrixXd JJT = J * J.transpose();
    double det = JJT.determinant();
    return std::sqrt(std::abs(det));
}

} // namespace model
} // namespace robospace

