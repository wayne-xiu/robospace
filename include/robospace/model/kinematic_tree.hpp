#pragma once

#include <robospace/model/joint.hpp>
#include <robospace/model/link.hpp>
#include <robospace/math/SE3.hpp>
#include <vector>
#include <Eigen/Dense>

namespace robospace {
namespace model {

/**
 * @brief Kinematic chain for serial manipulators
 *
 * Manages joint-link chains for industrial robots:
 *   Link0 (base) -- J1 -- Link1 -- J2 -- Link2 -- ... -- LinkN
 *
 * Supports:
 * - Serial chains (N joints → N+1 links)
 * - Industrial robot features (axis direction, J2-J3 coupling)
 * - Forward kinematics with cached poses
 *
 * Limitation: Serial chains only (no branching). For dual-arm or humanoid
 * robots, use multiple KinematicTree instances or future TreeKinematics class.
 *
 * Note: This class is typically internal to Robot. Users interact with
 * Robot class which provides high-level FK API.
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

    // Configuration
    void set_configuration(const Eigen::VectorXd& q);
    const Eigen::VectorXd& configuration() const { return q_; }

    // Forward kinematics (stateful - uses stored configuration)
    void compute_forward_kinematics();
    math::SE3 link_pose(int link_id) const;

    // Forward kinematics (stateless - takes configuration as parameter)
    std::vector<math::SE3> compute_forward_kinematics(const Eigen::VectorXd& q) const;
    math::SE3 compute_link_pose(const Eigen::VectorXd& q, int link_id) const;

private:
    std::vector<Link> links_;
    std::vector<Joint> joints_;
    Eigen::VectorXd q_;
    std::vector<math::SE3> link_poses_;
};

} // namespace model
} // namespace robospace
