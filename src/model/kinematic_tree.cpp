#include <robospace/model/kinematic_tree.hpp>
#include <stdexcept>

namespace robospace {
namespace model {

void KinematicTree::add_link(const Link& link) {
    links_.push_back(link);
}

void KinematicTree::add_joint(const Joint& joint) {
    joints_.push_back(joint);
}

void KinematicTree::set_configuration(const Eigen::VectorXd& q) {
    if (q.size() != num_joints()) {
        throw std::invalid_argument(
            "Configuration size mismatch: expected " +
            std::to_string(num_joints()) + " values, got " +
            std::to_string(q.size()));
    }
    q_ = q;
}

void KinematicTree::compute_forward_kinematics() {
    if (!is_valid()) {
        throw std::runtime_error(
            "Invalid kinematic tree: " + std::to_string(num_links()) +
            " links but " + std::to_string(num_joints()) + " joints");
    }

    if (q_.size() != num_joints()) {
        throw std::runtime_error("Configuration not set");
    }

    link_poses_.resize(num_links());
    link_poses_[0] = math::SE3::Identity();

    for (int i = 1; i < num_links(); ++i) {
        int joint_idx = i - 1;
        double q_eff = joints_[joint_idx].get_effective_angle(q_(joint_idx), q_);
        math::SE3 T_joint = joints_[joint_idx].transform(q_eff);
        link_poses_[i] = link_poses_[i - 1] * T_joint;
    }
}

math::SE3 KinematicTree::link_pose(int link_id) const {
    if (link_id < 0 || link_id >= num_links()) {
        throw std::out_of_range("Link ID " + std::to_string(link_id) + " out of range");
    }

    if (link_poses_.empty()) {
        throw std::runtime_error("Forward kinematics not computed");
    }

    return link_poses_[link_id];
}

std::vector<math::SE3> KinematicTree::compute_forward_kinematics(const Eigen::VectorXd& q) const {
    if (q.size() != num_joints()) {
        throw std::invalid_argument("Configuration size mismatch");
    }

    const int n_links = num_links();
    std::vector<math::SE3> poses;
    poses.reserve(n_links);
    poses.emplace_back(math::SE3::Identity());

    for (int i = 1; i < n_links; ++i) {
        const int joint_idx = i - 1;
        const double q_eff = joints_[joint_idx].get_effective_angle(q(joint_idx), q);
        poses.emplace_back(poses[i - 1] * joints_[joint_idx].transform(q_eff));
    }

    return poses;
}

math::SE3 KinematicTree::compute_link_pose(const Eigen::VectorXd& q, int link_id) const {
    if (q.size() != num_joints()) {
        throw std::invalid_argument("Configuration size mismatch");
    }
    if (link_id < 0 || link_id >= num_links()) {
        throw std::out_of_range("Link ID out of range");
    }
    if (link_id == 0) {
        return math::SE3::Identity();
    }

    math::SE3 pose = math::SE3::Identity();
    for (int i = 1; i <= link_id; ++i) {
        const int joint_idx = i - 1;
        const double q_eff = joints_[joint_idx].get_effective_angle(q(joint_idx), q);
        pose = pose * joints_[joint_idx].transform(q_eff);
    }

    return pose;
}

} // namespace model
} // namespace robospace
