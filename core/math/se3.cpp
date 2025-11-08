#include "se3.hpp"
#include <stdexcept>

namespace robospace {
namespace math {

// Utility functions

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return S;
}

Eigen::Vector3d unskew(const Eigen::Matrix3d& S) {
    return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
}

// se3 implementation

se3::se3() : omega_(Eigen::Vector3d::Zero()), v_(Eigen::Vector3d::Zero()) {}

se3::se3(const Eigen::Vector3d& omega, const Eigen::Vector3d& v)
    : omega_(omega), v_(v) {}

se3::se3(const Vector6d& xi) {
    omega_ = xi.head<3>();
    v_ = xi.tail<3>();
}

se3::se3(const Eigen::Matrix4d& bracket) {
    omega_ = unskew(bracket.block<3, 3>(0, 0));
    v_ = bracket.block<3, 1>(0, 3);
}

se3 se3::Zero() {
    return se3();
}

Vector6d se3::vector() const {
    Vector6d xi;
    xi << omega_, v_;
    return xi;
}

Eigen::Matrix4d se3::bracket() const {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
    mat.block<3, 3>(0, 0) = skew(omega_);
    mat.block<3, 1>(0, 3) = v_;
    return mat;
}

se3 se3::lieBracket(const se3& other) const {
    // [ξ₁, ξ₂] = ω₁ × ω₂, ω₁ × v₂ - ω₂ × v₁
    Eigen::Vector3d omega_result = omega_.cross(other.omega_);
    Eigen::Vector3d v_result = omega_.cross(other.v_) - other.omega_.cross(v_);
    return se3(omega_result, v_result);
}

se3 se3::operator+(const se3& other) const {
    return se3(omega_ + other.omega_, v_ + other.v_);
}

se3 se3::operator-(const se3& other) const {
    return se3(omega_ - other.omega_, v_ - other.v_);
}

se3 se3::operator*(double scalar) const {
    return se3(omega_ * scalar, v_ * scalar);
}

se3 se3::operator/(double scalar) const {
    if (std::abs(scalar) < 1e-10) {
        throw std::runtime_error("Division by near-zero scalar");
    }
    return se3(omega_ / scalar, v_ / scalar);
}

se3 se3::operator-() const {
    return se3(-omega_, -v_);
}

bool se3::isApprox(const se3& other, double tol) const {
    return omega_.isApprox(other.omega_, tol) && v_.isApprox(other.v_, tol);
}

std::ostream& operator<<(std::ostream& os, const se3& xi) {
    os << "se3(omega=[" << xi.omega().transpose() << "], v=["
       << xi.v().transpose() << "])";
    return os;
}

se3 operator*(double scalar, const se3& xi) {
    return xi * scalar;
}

} // namespace math
} // namespace robospace
