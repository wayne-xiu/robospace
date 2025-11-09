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
            " links but " + std::to_string(num_joints()) + " joints. " +
            "Expected N+1 links for N joints.");
    }

    if (q_.size() != num_joints()) {
        throw std::runtime_error(
            "Configuration not set. Call set_configuration() first.");
    }

    // Resize pose cache
    link_poses_.resize(num_links());

    // Link 0 (base) is at identity
    link_poses_[0] = math::SE3::Identity();

    // Forward kinematics: propagate through chain
    // Link i pose = Link(i-1) pose * Joint(i) transform
    for (int i = 1; i < num_links(); ++i) {
        int joint_idx = i - 1;  // Joint connecting Link(i-1) to Link(i)

        // Get joint transform at current configuration
        math::SE3 joint_transform = joints_[joint_idx].transform(q_(joint_idx));

        // Compose with previous link pose
        link_poses_[i] = link_poses_[i - 1] * joint_transform;
    }
}

math::SE3 KinematicTree::link_pose(int link_id) const {
    if (link_id < 0 || link_id >= num_links()) {
        throw std::out_of_range(
            "Link ID " + std::to_string(link_id) + " out of range [0, " +
            std::to_string(num_links() - 1) + "]");
    }

    if (link_poses_.empty()) {
        throw std::runtime_error(
            "Forward kinematics not computed. Call compute_forward_kinematics() first.");
    }

    return link_poses_[link_id];
}

} // namespace model
} // namespace robospace
