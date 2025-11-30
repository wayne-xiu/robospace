#include <robospace/model/joint.hpp>
#include <robospace/model/dh_adapter.hpp>
#include <robospace/math/SO3.hpp>

namespace robospace {
namespace model {

Joint::Joint(const std::string& name, JointType type,
             int parent_link_id, int child_link_id)
    : name_(name), type_(type),
      parent_link_id_(parent_link_id), child_link_id_(child_link_id) {

    // Set default limits based on joint type
    if (type == JointType::CONTINUOUS) {
        lower_limit_ = -std::numeric_limits<double>::infinity();
        upper_limit_ = std::numeric_limits<double>::infinity();
    } else if (type == JointType::REVOLUTE) {
        lower_limit_ = -M_PI;
        upper_limit_ = M_PI;
    } else if (type == JointType::PRISMATIC) {
        lower_limit_ = -1.0;  // Default: -1m to 1m
        upper_limit_ = 1.0;
    }
}

void Joint::precompute_dh_se3() {
    if (!has_dh_) {
        return;
    }

    dh_se3_base_ = DHAdapter::convert(dh_params_, 0.0, is_prismatic());
    has_dh_se3_ = true;
}

math::SE3 Joint::transform(double q) const {
    // Fixed joint: just return origin
    if (type_ == JointType::FIXED) {
        return origin_;
    }

    // If DH parameters are available, use them
    if (has_dh_) {
        return dh_params_.transform(q, is_prismatic());
    }

    // Otherwise, use URDF-style origin + axis
    // T = origin * exp([axis] * q)

    if (is_revolute()) {
        // Rotation about axis by angle q
        math::SO3 R = math::SO3::FromAxisAngle(axis_, q);
        return origin_ * math::SE3(R.matrix(), Eigen::Vector3d::Zero());
    } else if (is_prismatic()) {
        // Translation along axis by distance q
        Eigen::Vector3d translation = axis_ * q;
        return origin_ * math::SE3::Translation(translation);
    }

    // Should not reach here
    return origin_;
}

} // namespace model
} // namespace robospace
