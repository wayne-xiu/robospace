#include "SO3.hpp"
#include "se3.hpp"  // for skew()
#include <cmath>

namespace robospace {
namespace math {

SO3::SO3() {
    R_ = Eigen::Matrix3d::Identity();
}

SO3::SO3(const Eigen::Matrix3d& R) : R_(R) {}

SO3::SO3(const Eigen::Vector3d& axis, double angle) {
    Eigen::Vector3d omega = axis * angle;
    *this = exp_so3(so3(omega));
}

SO3 SO3::Identity() {
    return SO3();
}

SO3 SO3::FromAxisAngle(const Eigen::Vector3d& axis, double angle) {
    return SO3(axis, angle);
}

SO3 SO3::RotX(double angle) {
    return FromAxisAngle(Eigen::Vector3d::UnitX(), angle);
}

SO3 SO3::RotY(double angle) {
    return FromAxisAngle(Eigen::Vector3d::UnitY(), angle);
}

SO3 SO3::RotZ(double angle) {
    return FromAxisAngle(Eigen::Vector3d::UnitZ(), angle);
}

void SO3::axisAngle(Eigen::Vector3d& axis, double& angle) const {
    // Extract angle from trace
    double trace = R_.trace();
    angle = std::acos((trace - 1.0) / 2.0);

    if (angle < 1e-10) {
        // Near identity
        axis = Eigen::Vector3d::UnitX();
        angle = 0.0;
        return;
    }

    // Extract axis from skew-symmetric part
    Eigen::Matrix3d R_minus_RT = R_ - R_.transpose();
    axis << R_minus_RT(2, 1), R_minus_RT(0, 2), R_minus_RT(1, 0);
    axis /= (2.0 * std::sin(angle));
}

SO3 SO3::operator*(const SO3& other) const {
    return SO3(R_ * other.R_);
}

Eigen::Vector3d SO3::operator*(const Eigen::Vector3d& v) const {
    return R_ * v;
}

SO3 SO3::inverse() const {
    return SO3(R_.transpose());
}

bool SO3::isApprox(const SO3& other, double tol) const {
    return R_.isApprox(other.R_, tol);
}

std::ostream& operator<<(std::ostream& os, const SO3& R) {
    os << "SO3:\n" << R.matrix();
    return os;
}

// Exponential map: so(3) → SO(3)

SO3 exp_so3(const so3& omega) {
    Eigen::Vector3d omega_vec = omega.vector();
    double theta = omega_vec.norm();

    if (theta < 1e-10) {
        // Near zero, return identity
        return SO3::Identity();
    }

    // Rodrigues' formula:
    // R = I + sin(θ)[ω̂] + (1-cos(θ))[ω̂]²
    Eigen::Vector3d omega_normalized = omega_vec / theta;
    Eigen::Matrix3d omega_skew = skew(omega_normalized);
    Eigen::Matrix3d omega_skew_sq = omega_skew * omega_skew;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
                       + std::sin(theta) * omega_skew
                       + (1.0 - std::cos(theta)) * omega_skew_sq;

    return SO3(R);
}

// Logarithm map: SO(3) → so(3)

so3 log_SO3(const SO3& R) {
    Eigen::Matrix3d R_mat = R.matrix();
    double trace = R_mat.trace();
    double theta = std::acos((trace - 1.0) / 2.0);

    if (theta < 1e-10) {
        // Near identity
        return so3::Zero();
    }

    // Extract axis from skew-symmetric part
    Eigen::Matrix3d R_minus_RT = R_mat - R_mat.transpose();
    Eigen::Vector3d omega_normalized;
    omega_normalized << R_minus_RT(2, 1), R_minus_RT(0, 2), R_minus_RT(1, 0);
    omega_normalized /= (2.0 * std::sin(theta));

    Eigen::Vector3d omega = omega_normalized * theta;
    return so3(omega);
}

} // namespace math
} // namespace robospace
