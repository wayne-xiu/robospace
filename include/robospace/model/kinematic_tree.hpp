#pragma once

#include <robospace/model/joint.hpp>
#include <robospace/model/link.hpp>
#include <robospace/math/SE3.hpp>
#include <vector>
#include <Eigen/Dense>

namespace robospace {
namespace model {

/**
 * @brief Stateless kinematic computation engine for serial manipulators
 *
 * Manages joint-link chains for industrial robots:
 *   Link0 (base) -- J1 -- Link1 -- J2 -- Link2 -- ... -- LinkN
 *
 * Supports:
 * - Serial chains (N joints → N+1 links)
 * - Industrial robot features (axis direction, J2-J3 coupling)
 * - Pure stateless forward kinematics and Jacobian computation
 *
 * Limitation: Serial chains only (no branching). For dual-arm or humanoid
 * robots, use multiple KinematicTree instances or future TreeKinematics class.
 *
 * Note: This class is stateless and manages only structure and computation.
 * Robot class manages state (configuration) and provides high-level API.
 */
class KinematicTree {
public:
    KinematicTree() = default;

    // Construction
    void add_link(const Link& link);
    void add_joint(const Joint& joint);

    // Queries
    int num_joints() const { return static_cast<int>(joints_.size()); }
    int num_links() const { return static_cast<int>(links_.size()); }
    const Joint& joint(int i) const { return joints_[i]; }
    const Link& link(int i) const { return links_[i]; }
    Joint& joint(int i) { return joints_[i]; }
    Link& link(int i) { return links_[i]; }
    bool is_valid() const { return links_.size() == joints_.size() + 1; }

    // Forward kinematics (stateless - pure computation)
    std::vector<math::SE3> compute_forward_kinematics(const Eigen::VectorXd& q) const;
    math::SE3 compute_link_pose(const Eigen::VectorXd& q, int link_id) const;

    // Jacobian (stateless - pure computation)
    // Returns 6×n Jacobian: rows 0-2 = angular (ω), rows 3-5 = linear (v)
    // NOTE: Returns FLANGE Jacobian (last link). Does NOT include tool/TCP offset.
    // For TCP Jacobian, use Robot::jacobe() which may apply tool transformation.
    //
    // compute_jacobian_base: Jacobian in base frame (spatial)
    // compute_jacobian_ee:   Jacobian in end-effector frame (body)
    Eigen::MatrixXd compute_jacobian_base(const Eigen::VectorXd& q) const;
    Eigen::MatrixXd compute_jacobian_ee(const Eigen::VectorXd& q) const;

private:
    Eigen::MatrixXd compute_jacobian_base_impl(const std::vector<math::SE3>& poses) const;

    std::vector<Link> links_;
    std::vector<Joint> joints_;
};

} // namespace model
} // namespace robospace
