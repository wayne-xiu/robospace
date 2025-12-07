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

Eigen::MatrixXd KinematicTree::compute_jacobian_base_impl(const Eigen::VectorXd& q, const std::vector<math::SE3>& poses) const {
    if (num_joints() == 0) {
        return Eigen::MatrixXd::Zero(6, 0);
    }

    std::vector<int> active_indices;
    for (int i = 0; i < num_joints(); ++i) {
        if (!joints_[i].is_fixed()) {
            active_indices.push_back(i);
        }
    }

    const int n_dof = static_cast<int>(active_indices.size());
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, n_dof);

    math::SE3 T_ee = poses.back();
    Eigen::Vector3d p_ee = T_ee.translation();

    for (int col = 0; col < n_dof; ++col) {
        int i = active_indices[col];
        math::SE3 T_joint = poses[i] * joints_[i].origin();
        Eigen::Vector3d p_joint = T_joint.translation();
        Eigen::Vector3d z_i = T_joint.rotation() * joints_[i].axis();

        if (joints_[i].is_revolute()) {
            J.block<3, 1>(0, col) = z_i;
            J.block<3, 1>(3, col) = z_i.cross(p_ee - p_joint);
        } else if (joints_[i].is_prismatic()) {
            J.block<3, 1>(0, col) = Eigen::Vector3d::Zero();
            J.block<3, 1>(3, col) = z_i;
        }
    }

    // Save geometric Jacobian for chain rule
    const Eigen::MatrixXd J_geo = J;

    std::unordered_map<int, int> joint_to_col;
    for (int col = 0; col < n_dof; ++col) {
        joint_to_col[active_indices[col]] = col;
    }

    for (int col = 0; col < n_dof; ++col) {
        int i = active_indices[col];
        J.col(col) = J_geo.col(col) * joints_[i].axis_direction();
    }

    for (int col_b = 0; col_b < n_dof; ++col_b) {
        int joint_b = active_indices[col_b];
        const auto& coupling_terms = joints_[joint_b].coupling_terms();

        for (const auto& term : coupling_terms) {
            auto it = joint_to_col.find(term.from_joint_id);
            if (it != joint_to_col.end()) {
                int col_a = it->second;
                J.col(col_a) += J_geo.col(col_b) * term.coefficient;
            }
        }
    }

    return J;
}

Eigen::MatrixXd KinematicTree::compute_jacobian_base(const Eigen::VectorXd& q) const {
    if (q.size() != num_joints()) {
        throw std::invalid_argument("Configuration size mismatch");
    }

    std::vector<math::SE3> poses = compute_forward_kinematics(q);
    return compute_jacobian_base_impl(q, poses);
}

Eigen::MatrixXd KinematicTree::compute_jacobian_ee(const Eigen::VectorXd& q) const {
    if (q.size() != num_joints()) {
        throw std::invalid_argument("Configuration size mismatch");
    }

    if (num_joints() == 0) {
        return Eigen::MatrixXd::Zero(6, 0);
    }

    std::vector<math::SE3> poses = compute_forward_kinematics(q);
    Eigen::MatrixXd J_base = compute_jacobian_base_impl(q, poses);

    math::SE3 T_ee = poses.back();
    return T_ee.inverse().adjoint() * J_base;
}

} // namespace model
} // namespace robospace
