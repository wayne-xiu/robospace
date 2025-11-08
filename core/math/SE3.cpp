#include "SE3.hpp"
#include <cmath>

namespace robospace {
namespace math {

// SE3 implementation

SE3::SE3() {
    g_ = Eigen::Matrix4d::Identity();
}

SE3::SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
    g_ = Eigen::Matrix4d::Identity();
    g_.block<3, 3>(0, 0) = R;
    g_.block<3, 1>(0, 3) = p;
}

SE3::SE3(const Eigen::Matrix4d& g) : g_(g) {}

SE3 SE3::Identity() {
    return SE3();
}

SE3 SE3::Translation(const Eigen::Vector3d& p) {
    return SE3(Eigen::Matrix3d::Identity(), p);
}

void SE3::decompose(Eigen::Matrix3d& R, Eigen::Vector3d& p) const {
    R = g_.block<3, 3>(0, 0);
    p = g_.block<3, 1>(0, 3);
}

SE3 SE3::operator*(const SE3& other) const {
    return SE3(g_ * other.g_);
}

Eigen::Vector3d SE3::operator*(const Eigen::Vector3d& p) const {
    Eigen::Vector4d p_h;
    p_h << p, 1.0;
    Eigen::Vector4d result = g_ * p_h;
    return result.head<3>();
}

SE3 SE3::inverse() const {
    Eigen::Matrix3d R = g_.block<3, 3>(0, 0);
    Eigen::Vector3d p = g_.block<3, 1>(0, 3);
    Eigen::Matrix3d R_T = R.transpose();
    Eigen::Vector3d p_inv = -R_T * p;
    return SE3(R_T, p_inv);
}

Eigen::Matrix6d SE3::adjoint() const {
    Eigen::Matrix3d R = g_.block<3, 3>(0, 0);
    Eigen::Vector3d p = g_.block<3, 1>(0, 3);
    Eigen::Matrix3d p_skew = skew(p);

    Eigen::Matrix6d Ad;
    Ad.block<3, 3>(0, 0) = R;
    Ad.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    Ad.block<3, 3>(3, 0) = p_skew * R;
    Ad.block<3, 3>(3, 3) = R;

    return Ad;
}

bool SE3::isApprox(const SE3& other, double tol) const {
    return g_.isApprox(other.g_, tol);
}

std::ostream& operator<<(std::ostream& os, const SE3& g) {
    os << "SE3:\n" << g.matrix();
    return os;
}

// Exponential map: se(3) → SE(3)

SE3 exp_se3(const se3& xi) {
    Eigen::Vector3d omega = xi.omega();
    Eigen::Vector3d v = xi.v();

    double theta = omega.norm();

    if (theta < 1e-10) {
        // Pure translation (||ω|| ≈ 0)
        return SE3(Eigen::Matrix3d::Identity(), v);
    }

    // Screw motion
    Eigen::Vector3d omega_normalized = omega / theta;
    Eigen::Matrix3d omega_skew = skew(omega_normalized);
    Eigen::Matrix3d omega_skew_sq = omega_skew * omega_skew;

    // Rodrigues' formula for SO(3):
    // R = I + sin(θ)[ω] + (1-cos(θ))[ω]²
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
                       + std::sin(theta) * omega_skew
                       + (1.0 - std::cos(theta)) * omega_skew_sq;

    // Matrix M for translation:
    // M = I*θ + (1-cos(θ))[ω] + (θ-sin(θ))[ω]²
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity() * theta
                      + (1.0 - std::cos(theta)) * omega_skew
                      + (theta - std::sin(theta)) * omega_skew_sq;

    // p = M * v / θ
    Eigen::Vector3d p = M * v / theta;

    return SE3(R, p);
}

// Logarithm map: SE(3) → se(3)

se3 log_SE3(const SE3& g) {
    Eigen::Matrix3d R = g.rotation();
    Eigen::Vector3d p = g.translation();

    // Compute angle of rotation
    double trace = R.trace();
    double theta = std::acos((trace - 1.0) / 2.0);

    if (theta < 1e-10) {
        // Near identity, pure translation
        return se3(Eigen::Vector3d::Zero(), p);
    }

    // Extract rotation axis
    Eigen::Matrix3d R_minus_RT = R - R.transpose();
    Eigen::Vector3d omega_normalized;
    omega_normalized << R_minus_RT(2, 1), R_minus_RT(0, 2), R_minus_RT(1, 0);
    omega_normalized /= (2.0 * std::sin(theta));

    Eigen::Vector3d omega = omega_normalized * theta;
    Eigen::Matrix3d omega_skew = skew(omega_normalized);
    Eigen::Matrix3d omega_skew_sq = omega_skew * omega_skew;

    // Inverse of M matrix:
    // M_inv = (1/θ)*I - (1/2)[ω] + (1/θ - (1/2)cot(θ/2))[ω]²
    double cot_half_theta = 1.0 / std::tan(theta / 2.0);
    Eigen::Matrix3d M_inv = (Eigen::Matrix3d::Identity() / theta)
                          - 0.5 * omega_skew
                          + (1.0 / theta - 0.5 * cot_half_theta) * omega_skew_sq;

    Eigen::Vector3d v = M_inv * p;

    return se3(omega, v);
}

// Adjoint transformation

se3 adjoint_SE3(const SE3& g, const se3& xi) {
    Eigen::Matrix6d Ad = g.adjoint();
    Eigen::Vector6d xi_vec = xi.vector();
    Eigen::Vector6d result = Ad * xi_vec;
    return se3(result);
}

} // namespace math
} // namespace robospace
