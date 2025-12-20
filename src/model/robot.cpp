#include <robospace/model/robot.hpp>
#include <robospace/model/urdf_parser.hpp>
#include <robospace/math/SO3.hpp>
#include <robospace/units.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <utility>
#include <stdexcept>
#include <filesystem>

namespace robospace {
namespace model {

Robot::Robot(const std::string& name, Entity* parent)
    : Entity(name, Entity::Type::ROBOT, parent),
      base_frame_(name + "_base"),
      flange_frame_(name + "_flange") {
    rebuild_entity_graph();
}

Robot::Robot(const Robot& other)
    : Entity(other.name_, Entity::Type::ROBOT, nullptr),
      tree_(other.tree_),
      base_frame_(other.base_frame_),
      flange_frame_(other.flange_frame_),
      tools_(other.tools_),
      active_tool_id_(other.active_tool_id_),
      q_(other.q_),
      home_position_(other.home_position_),
      link_name_to_id_(other.link_name_to_id_),
      joint_name_to_id_(other.joint_name_to_id_),
      tool_name_to_id_(other.tool_name_to_id_) {
    pose_ = other.pose_;
    rebuild_entity_graph();
    if (q_.size() > 0) {
        update_link_poses(expand_config(q_));
    }
}

Robot& Robot::operator=(const Robot& other) {
    if (this == &other) {
        return *this;
    }

    detach_entity_graph();

    name_ = other.name_;
    type_ = Entity::Type::ROBOT;
    pose_ = other.pose_;
    set_parent(nullptr);
    std::vector<Entity*> detached = children_;
    for (auto* child : detached) {
        child->set_parent(nullptr);
    }
    children_.clear();

    tree_ = other.tree_;
    base_frame_ = other.base_frame_;
    flange_frame_ = other.flange_frame_;
    tools_ = other.tools_;
    active_tool_id_ = other.active_tool_id_;
    q_ = other.q_;
    home_position_ = other.home_position_;
    link_name_to_id_ = other.link_name_to_id_;
    joint_name_to_id_ = other.joint_name_to_id_;
    tool_name_to_id_ = other.tool_name_to_id_;

    rebuild_entity_graph();
    if (q_.size() > 0) {
        update_link_poses(expand_config(q_));
    }

    return *this;
}

Robot::Robot(Robot&& other) noexcept
    : Entity(other.name_, Entity::Type::ROBOT, nullptr),
      base_frame_(other.base_frame_.name()),
      flange_frame_(other.flange_frame_.name()),
      active_tool_id_(other.active_tool_id_),
      q_(std::move(other.q_)),
      home_position_(std::move(other.home_position_)),
      link_name_to_id_(std::move(other.link_name_to_id_)),
      joint_name_to_id_(std::move(other.joint_name_to_id_)),
      tool_name_to_id_(std::move(other.tool_name_to_id_)) {
    pose_ = other.pose_;
    other.detach_entity_graph();
    tree_ = std::move(other.tree_);
    base_frame_ = std::move(other.base_frame_);
    flange_frame_ = std::move(other.flange_frame_);
    tools_ = std::move(other.tools_);
    rebuild_entity_graph();
    if (q_.size() > 0) {
        update_link_poses(expand_config(q_));
    }
}

Robot& Robot::operator=(Robot&& other) noexcept {
    if (this == &other) {
        return *this;
    }

    detach_entity_graph();
    other.detach_entity_graph();

    name_ = std::move(other.name_);
    type_ = Entity::Type::ROBOT;
    pose_ = other.pose_;
    set_parent(nullptr);
    std::vector<Entity*> detached = children_;
    for (auto* child : detached) {
        child->set_parent(nullptr);
    }
    children_.clear();

    tree_ = std::move(other.tree_);
    base_frame_ = std::move(other.base_frame_);
    flange_frame_ = std::move(other.flange_frame_);
    tools_ = std::move(other.tools_);
    active_tool_id_ = other.active_tool_id_;
    q_ = std::move(other.q_);
    home_position_ = std::move(other.home_position_);
    link_name_to_id_ = std::move(other.link_name_to_id_);
    joint_name_to_id_ = std::move(other.joint_name_to_id_);
    tool_name_to_id_ = std::move(other.tool_name_to_id_);

    rebuild_entity_graph();
    if (q_.size() > 0) {
        update_link_poses(expand_config(q_));
    }

    return *this;
}

Robot::~Robot() {
    detach_entity_graph();
}

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
    Eigen::VectorXd qmin(dof());
    Eigen::VectorXd qmax(dof());

    int dof_idx = 0;
    for (int i = 0; i < num_joints(); ++i) {
        if (!joint(i).is_fixed()) {
            qmin(dof_idx) = joint(i).lower_limit();
            qmax(dof_idx) = joint(i).upper_limit();
            dof_idx++;
        }
    }

    return {qmin, qmax};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Robot::joint_limits_full() const {
    Eigen::VectorXd qmin(num_joints());
    Eigen::VectorXd qmax(num_joints());

    for (int i = 0; i < num_joints(); ++i) {
        qmin(i) = joint(i).lower_limit();
        qmax(i) = joint(i).upper_limit();
    }

    return {qmin, qmax};
}

void Robot::set_config(const Eigen::VectorXd& q) {
    bool updated = false;
    if (q.size() == dof()) {
        q_ = q;
        updated = true;
    }
    if (q.size() == num_joints()) {
        q_ = compress_config(q);
        updated = true;
    }
    if (!updated) {
        throw std::invalid_argument(
            "Configuration size mismatch: expected " +
            std::to_string(dof()) + " (dof) or " +
            std::to_string(num_joints()) + " (full), got " +
            std::to_string(q.size()));
    }

    if (num_joints() > 0) {
        update_link_poses(expand_config(q_));
    }
}

void Robot::set_home(const Eigen::VectorXd& q) {
    if (q.size() == dof()) {
        home_position_ = q;
        return;
    }
    if (q.size() == num_joints()) {
        home_position_ = compress_config(q);
        return;
    }
    throw std::invalid_argument("Home configuration size must match dof or full joint size");
}

// Forward kinematics (stateless)
// FK = base_frame_.pose() * chain_fk * flange_frame_.pose() [* tool_tcp if active]
math::SE3 Robot::fk(const Eigen::VectorXd& q) const {
    if (num_links() == 0) {
        return base_frame_.pose();
    }

    Eigen::VectorXd q_full = normalize_config(q);
    int flange_id = num_links() - 1;
    math::SE3 chain_fk = tree_.compute_link_pose(q_full, flange_id);
    math::SE3 tcp_pose = base_frame_.pose() * chain_fk * flange_frame_.pose();

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
    Eigen::VectorXd q_full = normalize_config(q);
    math::SE3 link_pose = tree_.compute_link_pose(q_full, link_id);
    return base_frame_.pose() * link_pose;
}

std::vector<math::SE3> Robot::fk_all(const Eigen::VectorXd& q) const {
    Eigen::VectorXd q_full = normalize_config(q);
    std::vector<math::SE3> poses = tree_.compute_forward_kinematics(q_full);
    for (auto& pose : poses) {
        pose = base_frame_.pose() * pose;
    }
    return poses;
}

// Forward kinematics (stateful)
math::SE3 Robot::current_pose() const {
    if (q_.size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return fk(q_);
}

math::SE3 Robot::current_pose(const std::string& link_name) const {
    if (q_.size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return fk(q_, link_name);
}

math::SE3 Robot::current_pose(int link_id) const {
    if (q_.size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return fk(q_, link_id);
}

std::vector<math::SE3> Robot::current_pose_all() const {
    if (q_.size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return fk_all(q_);
}

// Differential kinematics
Eigen::MatrixXd Robot::jacob0(const Eigen::VectorXd& q) const {
    Eigen::VectorXd q_full = normalize_config(q);
    Eigen::MatrixXd J_tree = tree_.compute_jacobian_base(q_full);
    return base_frame_.pose().adjoint() * J_tree;
}

Eigen::MatrixXd Robot::jacobe(const Eigen::VectorXd& q) const {
    Eigen::VectorXd q_full = normalize_config(q);
    Eigen::MatrixXd J_flange = tree_.compute_jacobian_ee(q_full);

    if (!has_active_tool()) {
        return J_flange;
    }

    math::SE3 T_tool_inv = active_tool().tcp_pose().inverse();
    return T_tool_inv.adjoint() * J_flange;
}

Eigen::MatrixXd Robot::jacob0() const {
    if (q_.size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return jacob0(q_);
}

Eigen::MatrixXd Robot::jacobe() const {
    if (q_.size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return jacobe(q_);
}

void Robot::set_base_pose(const math::SE3& pose) {
    base_frame_.set_pose(pose);
}

void Robot::set_flange_pose(const math::SE3& pose) {
    flange_frame_.set_pose(pose);
}

int Robot::add_tool(const Tool& tool) {
    auto it = tool_name_to_id_.find(tool.name());
    if (it != tool_name_to_id_.end()) {
        throw std::runtime_error("Tool with name '" + tool.name() + "' already exists");
    }

    int tool_id = static_cast<int>(tools_.size());
    tools_.push_back(tool);
    tools_.back().set_parent(&flange_frame_);
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
    if (q_.size() == 0) {
        throw std::runtime_error("Configuration not set");
    }
    return Entity::pose() * current_pose();
}

Eigen::VectorXd Robot::expand_config(const Eigen::VectorXd& q_dof) const {
    if (q_dof.size() == num_joints()) {
        return q_dof;
    }
    if (q_dof.size() != dof()) {
        throw std::invalid_argument(
            "Configuration size mismatch: expected " +
            std::to_string(dof()) + " (dof), got " +
            std::to_string(q_dof.size()));
    }

    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(num_joints());
    int dof_idx = 0;
    for (int i = 0; i < num_joints(); ++i) {
        if (!joint(i).is_fixed()) {
            q_full(i) = q_dof(dof_idx++);
        }
    }
    return q_full;
}

Eigen::VectorXd Robot::compress_config(const Eigen::VectorXd& q_full) const {
    if (q_full.size() != num_joints()) {
        throw std::invalid_argument(
            "Full configuration size mismatch: expected " +
            std::to_string(num_joints()) + ", got " +
            std::to_string(q_full.size()));
    }

    Eigen::VectorXd q_dof(dof());
    int dof_idx = 0;
    for (int i = 0; i < num_joints(); ++i) {
        if (!joint(i).is_fixed()) {
            q_dof(dof_idx++) = q_full(i);
        }
    }
    return q_dof;
}

void Robot::add_link(const Link& link) {
    int id = tree_.num_links();
    link_name_to_id_[link.name()] = id;
    tree_.add_link(link);
    rebuild_entity_graph();
}

void Robot::add_joint(const Joint& joint) {
    int id = tree_.num_joints();
    joint_name_to_id_[joint.name()] = id;
    tree_.add_joint(joint);
    rebuild_entity_graph();
}

Robot Robot::from_urdf(const std::string& urdf_path) {
    return URDFParser::parse_file(urdf_path);
}

Robot Robot::from_urdf_string(const std::string& urdf_string) {
    return URDFParser::parse_string(urdf_string);
}

Robot Robot::from_config(const std::string& config_path) {
    using json = nlohmann::json;
    namespace fs = std::filesystem;

    std::ifstream file(config_path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open config file: " + config_path);
    }
    json config = json::parse(file);

    fs::path config_dir = fs::path(config_path).parent_path();

    if (!config.contains("model")) {
        throw std::runtime_error("Invalid config: missing 'model' field");
    }
    fs::path model_path = config_dir / config["model"].get<std::string>();
    Robot robot = Robot::from_urdf(model_path.string());

    if (config.contains("name")) {
        robot.set_name(config["name"].get<std::string>());
    }

    auto parse_frame_pose = [&](const json& frame) -> math::SE3 {
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

        if (frame.contains("xyz")) {
            auto xyz_arr = frame["xyz"].get<std::vector<double>>();
            if (xyz_arr.size() == 3) {
                xyz = units::mm_to_m(Eigen::Vector3d(xyz_arr[0], xyz_arr[1], xyz_arr[2]));
            }
        }

        if (frame.contains("rpy")) {
            auto rpy_arr = frame["rpy"].get<std::vector<double>>();
            if (rpy_arr.size() == 3) {
                // RPY convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
                double roll = units::deg_to_rad(rpy_arr[0]);
                double pitch = units::deg_to_rad(rpy_arr[1]);
                double yaw = units::deg_to_rad(rpy_arr[2]);
                rot = math::SO3::RotZ(yaw).matrix() *
                      math::SO3::RotY(pitch).matrix() *
                      math::SO3::RotX(roll).matrix();
            }
        } else if (frame.contains("quaternion")) {
            auto q = frame["quaternion"].get<std::vector<double>>();
            if (q.size() == 4) {
                Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
                rot = quat.normalized().toRotationMatrix();
            }
        }

        return math::SE3(rot, xyz);
    };

    if (config.contains("base_frame")) {
        robot.set_base_pose(parse_frame_pose(config["base_frame"]));
    }

    if (config.contains("flange_frame")) {
        robot.set_flange_pose(parse_frame_pose(config["flange_frame"]));
    }

    if (config.contains("home")) {
        auto home_arr = config["home"].get<std::vector<double>>();
        Eigen::VectorXd home(home_arr.size());
        for (size_t i = 0; i < home_arr.size(); ++i) {
            home(i) = units::deg_to_rad(home_arr[i]);
        }
        robot.set_home(home);
    }

    return robot;
}

double Robot::manipulability(const Eigen::VectorXd& q) const {
    Eigen::MatrixXd J = jacob0(q);
    Eigen::MatrixXd JJT = J * J.transpose();
    double det = JJT.determinant();
    return std::sqrt(std::abs(det));
}

Eigen::VectorXd Robot::normalize_config(const Eigen::VectorXd& q) const {
    if (q.size() == num_joints()) {
        return q;
    }
    if (q.size() == dof()) {
        return expand_config(q);
    }
    throw std::invalid_argument(
        "Configuration size mismatch: expected " +
        std::to_string(dof()) + " (dof) or " +
        std::to_string(num_joints()) + " (full), got " +
        std::to_string(q.size()));
}

void Robot::update_link_poses(const Eigen::VectorXd& q_full) {
    if (num_links() == 0) {
        return;
    }
    if (q_full.size() != num_joints()) {
        throw std::invalid_argument(
            "Full configuration size mismatch: expected " +
            std::to_string(num_joints()) + ", got " +
            std::to_string(q_full.size()));
    }

    tree_.link(0).set_pose(math::SE3::Identity());
    for (int i = 1; i < num_links(); ++i) {
        int joint_idx = i - 1;
        const Joint& joint = tree_.joint(joint_idx);
        double q_eff = joint.get_effective_angle(q_full(joint_idx), q_full);
        tree_.link(i).set_pose(joint.transform(q_eff));
    }
}

void Robot::rebuild_entity_graph() {
    detach_entity_graph();

    base_frame_.set_parent(this);

    if (num_links() > 0) {
        tree_.link(0).set_parent(&base_frame_);
        for (int i = 0; i < num_joints(); ++i) {
            const Joint& joint = tree_.joint(i);
            int parent_id = joint.parent_link_id();
            int child_id = joint.child_link_id();
            if (parent_id >= 0 && parent_id < num_links() &&
                child_id >= 0 && child_id < num_links()) {
                tree_.link(child_id).set_parent(&tree_.link(parent_id));
            }
        }
        flange_frame_.set_parent(&tree_.link(num_links() - 1));
    } else {
        flange_frame_.set_parent(&base_frame_);
    }

    for (auto& tool : tools_) {
        tool.set_parent(&flange_frame_);
    }
}

void Robot::detach_entity_graph() {
    base_frame_.set_parent(nullptr);
    flange_frame_.set_parent(nullptr);

    for (int i = 0; i < num_links(); ++i) {
        tree_.link(i).set_parent(nullptr);
    }

    for (auto& tool : tools_) {
        tool.set_parent(nullptr);
    }
}

} // namespace model
} // namespace robospace
