#include <robospace/model/robot.hpp>
#include <robospace/model/urdf_parser.hpp>
#include <robospace/math/SO3.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include <filesystem>

namespace robospace {
namespace model {

Robot::Robot(const std::string& name, Entity* parent)
    : Entity(name, Entity::Type::ROBOT, parent),
      base_frame_(name + "_base"),
      flange_frame_(name + "_flange") {}

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

void Robot::set_config(const Eigen::VectorXd& q) {
    if (q.size() != num_joints()) {
        throw std::invalid_argument(
            "Configuration size mismatch: expected " +
            std::to_string(num_joints()) + " values, got " +
            std::to_string(q.size()));
    }
    q_ = q;
}

void Robot::set_home(const Eigen::VectorXd& q) {
    if (q.size() != dof()) {
        throw std::invalid_argument("Home configuration size must match dof");
    }
    home_position_ = q;
}

// Forward kinematics (stateless)
// FK = base_frame_.pose() * chain_fk * flange_frame_.pose() [* tool_tcp if active]
math::SE3 Robot::fk(const Eigen::VectorXd& q) const {
    if (num_links() == 0) {
        return base_frame_.pose();
    }

    int flange_id = num_links() - 1;
    math::SE3 chain_fk = tree_.compute_link_pose(q, flange_id);
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
    math::SE3 link_pose = tree_.compute_link_pose(q, link_id);
    return base_frame_.pose() * link_pose;
}

std::vector<math::SE3> Robot::fk_all(const Eigen::VectorXd& q) const {
    std::vector<math::SE3> poses = tree_.compute_forward_kinematics(q);
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
    Eigen::MatrixXd J_tree = tree_.compute_jacobian_base(q);
    return base_frame_.pose().adjoint() * J_tree;
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
    if (q_.size() == 0) {
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

Robot Robot::from_config(const std::string& config_path) {
    using json = nlohmann::json;
    namespace fs = std::filesystem;

    constexpr double deg2rad = M_PI / 180.0;
    constexpr double mm2m = 0.001;

    // Read and parse JSON config
    std::ifstream file(config_path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open config file: " + config_path);
    }
    json config = json::parse(file);

    // Get config directory for resolving relative paths
    fs::path config_dir = fs::path(config_path).parent_path();

    // Load model (URDF)
    if (!config.contains("model")) {
        throw std::runtime_error("Invalid config: missing 'model' field");
    }
    fs::path model_path = config_dir / config["model"].get<std::string>();
    Robot robot = Robot::from_urdf(model_path.string());

    // Override name if specified
    if (config.contains("name")) {
        robot.set_name(config["name"].get<std::string>());
    }

    // Helper to parse frame pose (xyz in mm, rpy in degrees)
    auto parse_frame_pose = [&](const json& frame) -> math::SE3 {
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

        if (frame.contains("xyz")) {
            auto xyz_arr = frame["xyz"].get<std::vector<double>>();
            if (xyz_arr.size() == 3) {
                // Convert mm to meters
                xyz = Eigen::Vector3d(xyz_arr[0], xyz_arr[1], xyz_arr[2]) * mm2m;
            }
        }

        if (frame.contains("rpy")) {
            auto rpy_arr = frame["rpy"].get<std::vector<double>>();
            if (rpy_arr.size() == 3) {
                // Convert degrees to radians
                // RPY convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
                double roll = rpy_arr[0] * deg2rad;
                double pitch = rpy_arr[1] * deg2rad;
                double yaw = rpy_arr[2] * deg2rad;
                rot = math::SO3::RotZ(yaw).matrix() *
                      math::SO3::RotY(pitch).matrix() *
                      math::SO3::RotX(roll).matrix();
            }
        } else if (frame.contains("quaternion")) {
            auto q = frame["quaternion"].get<std::vector<double>>();
            if (q.size() == 4) {
                // Quaternion [w, x, y, z]
                Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
                rot = quat.normalized().toRotationMatrix();
            }
        }

        return math::SE3(rot, xyz);
    };

    // Apply base_frame pose (REP-103 correction)
    if (config.contains("base_frame")) {
        robot.set_base_pose(parse_frame_pose(config["base_frame"]));
    }

    // Apply flange_frame pose
    if (config.contains("flange_frame")) {
        robot.set_flange_pose(parse_frame_pose(config["flange_frame"]));
    }

    // Set home configuration (degrees to radians)
    if (config.contains("home")) {
        auto home_arr = config["home"].get<std::vector<double>>();
        Eigen::VectorXd home(home_arr.size());
        for (size_t i = 0; i < home_arr.size(); ++i) {
            home(i) = home_arr[i] * deg2rad;
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

} // namespace model
} // namespace robospace

