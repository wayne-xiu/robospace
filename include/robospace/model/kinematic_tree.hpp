#pragma once

#include <robospace/model/joint.hpp>
#include <robospace/model/link.hpp>
#include <robospace/math/SE3.hpp>
#include <vector>
#include <Eigen/Dense>

namespace robospace {
namespace model {

/**
 * @brief KinematicTree manages the joint-link chain of a robot
 *
 * A kinematic tree represents the structural connectivity and motion of
 * a robot's joints and links. For Phase 1, we support serial chains:
 *
 * Structure:
 *   Link0 (base) -- Joint1 -- Link1 -- Joint2 -- Link2 -- ... -- LinkN
 *
 * Conventions:
 * - Link 0 is the base (fixed to world or parent frame)
 * - Joint i connects Link(i-1) to Link(i)
 * - N joints → N+1 links
 * - Configuration q is N-dimensional (one value per joint)
 *
 * Usage:
 *   KinematicTree tree;
 *   tree.add_link(base);           // Link 0
 *   tree.add_joint(joint1);        // Joint 1
 *   tree.add_link(link1);          // Link 1
 *   tree.add_joint(joint2);        // Joint 2
 *   tree.add_link(link2);          // Link 2
 *
 *   tree.set_configuration(q);     // Update joint angles
 *   SE3 T = tree.link_pose(2);     // Get Link 2 pose in base frame
 */
class KinematicTree {
public:
    /**
     * @brief Construct empty kinematic tree
     */
    KinematicTree() = default;

    // ========================================================================
    // Construction (build the tree)
    // ========================================================================

    /**
     * @brief Add a link to the chain
     * @param link Link to add (copied)
     *
     * Links must be added in order: Link0, Link1, Link2, ...
     * First link added is the base.
     */
    void add_link(const Link& link);

    /**
     * @brief Add a joint to the chain
     * @param joint Joint to add (copied)
     *
     * Joints must be added in order: Joint1, Joint2, ...
     * Joint i connects Link(i-1) to Link(i).
     */
    void add_joint(const Joint& joint);

    // ========================================================================
    // Queries
    // ========================================================================

    /**
     * @brief Get number of joints
     */
    int num_joints() const { return static_cast<int>(joints_.size()); }

    /**
     * @brief Get number of links
     */
    int num_links() const { return static_cast<int>(links_.size()); }

    /**
     * @brief Get joint by index
     * @param i Joint index (0-based, but Joint 0 doesn't exist - use 1..N)
     */
    const Joint& joint(int i) const { return joints_[i]; }

    /**
     * @brief Get link by index
     * @param i Link index (0 = base, 1..N)
     */
    const Link& link(int i) const { return links_[i]; }

    /**
     * @brief Check if tree is valid (N joints → N+1 links)
     */
    bool is_valid() const { return links_.size() == joints_.size() + 1; }

    // ========================================================================
    // Configuration (joint angles)
    // ========================================================================

    /**
     * @brief Set joint configuration
     * @param q Joint values (size must equal num_joints)
     *
     * Updates internal configuration. Call compute_forward_kinematics()
     * afterwards to update link poses.
     */
    void set_configuration(const Eigen::VectorXd& q);

    /**
     * @brief Get current joint configuration
     */
    const Eigen::VectorXd& configuration() const { return q_; }

    // ========================================================================
    // Forward Kinematics
    // ========================================================================

    /**
     * @brief Compute forward kinematics for all links
     *
     * Updates all link poses based on current joint configuration.
     * Must call set_configuration() first.
     */
    void compute_forward_kinematics();

    /**
     * @brief Get link pose in base frame
     * @param link_id Link index
     * @return SE3 transformation from base to link
     *
     * Must call compute_forward_kinematics() first.
     * Link 0 (base) returns identity.
     */
    math::SE3 link_pose(int link_id) const;

private:
    std::vector<Link> links_;           ///< Links (Link 0 is base)
    std::vector<Joint> joints_;         ///< Joints (Joint i: Link(i-1) → Link(i))
    Eigen::VectorXd q_;                 ///< Current joint configuration

    std::vector<math::SE3> link_poses_; ///< Cached link poses (base frame)
};

} // namespace model
} // namespace robospace
