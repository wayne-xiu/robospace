#pragma once

#include <robospace/model/dh_params.hpp>
#include <robospace/math/SE3.hpp>
#include <string>
#include <vector>
#include <limits>

namespace robospace {
namespace model {

/**
 * @brief Type of robot joint
 */
enum class JointType {
    REVOLUTE,    ///< Rotational joint (1-DOF rotation about axis)
    PRISMATIC,   ///< Linear joint (1-DOF translation along axis)
    CONTINUOUS,  ///< Continuous rotation (no limits)
    FIXED        ///< Fixed joint (0-DOF, rigid connection)
};

/**
 * @brief Robot joint connecting two links
 *
 * A joint defines the kinematic relationship between a parent link and a child
 * link. It supports multiple kinematic representations:
 * - DH parameters (Standard or Modified convention)
 * - URDF-style origin + axis
 * - Screw axis (Modern Robotics)
 *
 * The joint also stores limits (position, velocity, effort) and can compute
 * transformations based on joint variable q.
 */
class Joint {
public:
    /**
     * @brief Construct a joint
     * @param name Joint name
     * @param type Joint type
     * @param parent_link_id Index of parent link
     * @param child_link_id Index of child link
     */
    Joint(const std::string& name, JointType type,
          int parent_link_id, int child_link_id);

    // ========================================================================
    // Identification
    // ========================================================================

    const std::string& name() const { return name_; }
    JointType type() const { return type_; }
    bool is_fixed() const { return type_ == JointType::FIXED; }
    bool is_revolute() const {
        return type_ == JointType::REVOLUTE || type_ == JointType::CONTINUOUS;
    }
    bool is_prismatic() const { return type_ == JointType::PRISMATIC; }

    // ========================================================================
    // Topology
    // ========================================================================

    int parent_link_id() const { return parent_link_id_; }
    int child_link_id() const { return child_link_id_; }

    // ========================================================================
    // Kinematic Representations (Triple representation)
    // ========================================================================

    // DH parameters (for DH-based kinematics)
    void set_dh_params(const DHParams& dh) { dh_params_ = dh; has_dh_ = true; }
    const DHParams& dh_params() const { return dh_params_; }
    bool has_dh_params() const { return has_dh_; }

    // URDF-style origin + axis (for URDF compatibility)
    void set_origin(const math::SE3& origin) { origin_ = origin; }
    void set_axis(const Eigen::Vector3d& axis) { axis_ = axis.normalized(); }
    const math::SE3& origin() const { return origin_; }
    const Eigen::Vector3d& axis() const { return axis_; }

    // Screw axis (for Modern Robotics / PoE kinematics)
    void set_screw_axis(const Eigen::Vector<double, 6>& screw) {
        screw_axis_ = screw;
        has_screw_ = true;
    }
    const Eigen::Vector<double, 6>& screw_axis() const { return screw_axis_; }
    bool has_screw_axis() const { return has_screw_; }

    // ========================================================================
    // Axis Direction and Coupling (RoboDK "Joint Senses")
    // ========================================================================

    /**
     * @brief Coupling term for joint coupling
     *
     * Represents contribution from another joint to this joint's effective angle.
     * Example: Fanuc J2-J3 coupling where J3_effective = J3 + (-1) * J2
     */
    struct CouplingTerm {
        int from_joint_id;      ///< Index of joint that affects this joint
        double coefficient;     ///< Coupling coefficient (e.g., -1, 0.5, 1)
    };

    /**
     * @brief Set axis direction (inversion)
     * @param dir Direction multiplier: +1 (normal) or -1 (inverted)
     *
     * Some manufacturers (KUKA, Comau) use inverted joint axes.
     * This corresponds to RoboDK "Joint Senses" values 1-6.
     */
    void set_axis_direction(int dir) {
        axis_direction_ = (dir >= 0) ? 1 : -1;
    }
    int axis_direction() const { return axis_direction_; }

    /**
     * @brief Add joint coupling relationship
     * @param from_joint_id Index of joint that affects this joint
     * @param coefficient Coupling coefficient
     *
     * Example: Fanuc J2-J3 coupling: joint[2].add_coupling(1, -1.0)
     * This corresponds to RoboDK "Joint Senses" 7th value.
     */
    void add_coupling(int from_joint_id, double coefficient) {
        if (std::abs(coefficient) > 1e-10) {  // Ignore near-zero coupling
            coupling_terms_.push_back({from_joint_id, coefficient});
        }
    }

    void clear_coupling() { coupling_terms_.clear(); }
    bool has_coupling() const { return !coupling_terms_.empty(); }
    const std::vector<CouplingTerm>& coupling_terms() const {
        return coupling_terms_;
    }

    /**
     * @brief Get effective joint angle considering axis direction and coupling
     * @param q_input Joint angle input (from controller or user)
     * @param all_q Vector of all joint angles (for coupling computation)
     * @return Effective joint angle after applying direction and coupling
     *
     * Formula: q_eff = axis_direction * (q_input + Σ(coef_i * q_i))
     *
     * Note: This does NOT include DH offset - that's handled separately.
     */
    double get_effective_angle(double q_input, const Eigen::VectorXd& all_q) const {
        double q = axis_direction_ * q_input;

        // Add coupling contributions
        for (const auto& term : coupling_terms_) {
            if (term.from_joint_id >= 0 && term.from_joint_id < all_q.size()) {
                q += term.coefficient * all_q[term.from_joint_id];
            }
        }

        return q;
    }

    // ========================================================================
    // Limits
    // ========================================================================

    void set_limits(double lower, double upper) {
        lower_limit_ = lower;
        upper_limit_ = upper;
    }
    void set_velocity_limit(double limit) { velocity_limit_ = limit; }
    void set_effort_limit(double limit) { effort_limit_ = limit; }

    double lower_limit() const { return lower_limit_; }
    double upper_limit() const { return upper_limit_; }
    double velocity_limit() const { return velocity_limit_; }
    double effort_limit() const { return effort_limit_; }

    bool has_position_limits() const {
        return lower_limit_ > -std::numeric_limits<double>::infinity() ||
               upper_limit_ < std::numeric_limits<double>::infinity();
    }

    /**
     * @brief Check if joint position is within limits
     * @param q Joint position
     * @return True if within limits or no limits defined
     */
    bool is_within_limits(double q) const {
        if (type_ == JointType::CONTINUOUS) return true;
        return q >= lower_limit_ && q <= upper_limit_;
    }

    // ========================================================================
    // Transformation computation
    // ========================================================================

    /**
     * @brief Compute transformation from parent link to child link
     * @param q Joint variable (angle for revolute, distance for prismatic)
     * @return SE3 transformation
     *
     * Uses DH parameters if available, otherwise uses origin + axis.
     * For fixed joints, returns origin transformation.
     */
    math::SE3 transform(double q) const;

private:
    // Identification
    std::string name_;
    JointType type_;

    // Topology
    int parent_link_id_;
    int child_link_id_;

    // Kinematic representations
    DHParams dh_params_;
    bool has_dh_ = false;

    math::SE3 origin_ = math::SE3::Identity();  // URDF origin
    Eigen::Vector3d axis_ = Eigen::Vector3d::UnitZ();  // URDF axis (default Z)

    Eigen::Vector<double, 6> screw_axis_;  // Modern Robotics screw axis
    bool has_screw_ = false;

    // Axis direction and coupling (RoboDK "Joint Senses")
    int axis_direction_ = 1;  // ±1 for normal/inverted axis
    std::vector<CouplingTerm> coupling_terms_;  // Joint coupling relationships

    // Limits
    double lower_limit_ = -std::numeric_limits<double>::infinity();
    double upper_limit_ = std::numeric_limits<double>::infinity();
    double velocity_limit_ = std::numeric_limits<double>::infinity();
    double effort_limit_ = std::numeric_limits<double>::infinity();
};

} // namespace model
} // namespace robospace
