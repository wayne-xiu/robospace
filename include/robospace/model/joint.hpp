#pragma once

#include <robospace/model/dh_params.hpp>
#include <robospace/math/SE3.hpp>
#include <string>
#include <vector>
#include <limits>

namespace robospace {
namespace model {

enum class JointType {
    REVOLUTE,
    PRISMATIC,
    CONTINUOUS,
    FIXED
};

/**
 * @brief Robot joint connecting two links
 *
 * Supports multiple kinematic representations (DH, URDF, screw axis),
 * industrial robot features (axis direction, coupling), and limits.
 */
class Joint {
public:
    Joint(const std::string& name, JointType type,
          int parent_link_id, int child_link_id);

    // Identification
    const std::string& name() const { return name_; }
    JointType type() const { return type_; }
    bool is_fixed() const { return type_ == JointType::FIXED; }
    bool is_revolute() const {
        return type_ == JointType::REVOLUTE || type_ == JointType::CONTINUOUS;
    }
    bool is_prismatic() const { return type_ == JointType::PRISMATIC; }

    // Topology
    int parent_link_id() const { return parent_link_id_; }
    int child_link_id() const { return child_link_id_; }

    // Kinematic representations
    void set_dh_params(const DHParams& dh) { dh_params_ = dh; has_dh_ = true; }
    const DHParams& dh_params() const { return dh_params_; }
    bool has_dh_params() const { return has_dh_; }

    void set_origin(const math::SE3& origin) { origin_ = origin; }
    void set_axis(const Eigen::Vector3d& axis) { axis_ = axis.normalized(); }
    const math::SE3& origin() const { return origin_; }
    const Eigen::Vector3d& axis() const { return axis_; }

    void set_screw_axis(const Eigen::Vector<double, 6>& screw) {
        screw_axis_ = screw;
        has_screw_ = true;
    }
    const Eigen::Vector<double, 6>& screw_axis() const { return screw_axis_; }
    bool has_screw_axis() const { return has_screw_; }

    // Industrial robot features (axis direction and coupling)
    struct CouplingTerm {
        int from_joint_id;
        double coefficient;
    };

    void set_axis_direction(int dir) {
        axis_direction_ = (dir >= 0) ? 1 : -1;
    }
    int axis_direction() const { return axis_direction_; }

    void add_coupling(int from_joint_id, double coefficient) {
        if (std::abs(coefficient) > 1e-10) {
            coupling_terms_.push_back({from_joint_id, coefficient});
        }
    }

    void clear_coupling() { coupling_terms_.clear(); }
    bool has_coupling() const { return !coupling_terms_.empty(); }
    const std::vector<CouplingTerm>& coupling_terms() const {
        return coupling_terms_;
    }

    // Effective angle: q_eff = axis_direction * (q_input + Σ(coef_i * q_i))
    double get_effective_angle(double q_input, const Eigen::VectorXd& all_q) const {
        double q = axis_direction_ * q_input;

        for (const auto& term : coupling_terms_) {
            if (term.from_joint_id >= 0 && term.from_joint_id < all_q.size()) {
                q += term.coefficient * all_q[term.from_joint_id];
            }
        }

        return q;
    }

    // Limits
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

    bool is_within_limits(double q) const {
        if (type_ == JointType::CONTINUOUS) return true;
        return q >= lower_limit_ && q <= upper_limit_;
    }

    // Transform computation (uses DH if available, otherwise origin + axis)
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

    // Axis direction and coupling ("Joint Senses")
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
