#include <robospace/math/transform.hpp>
#include <cmath>

namespace robospace {
namespace math {

// Constructors

Transform::Transform() {
    T_ = Eigen::Matrix4d::Identity();
}

Transform::Transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
    T_ = Eigen::Matrix4d::Identity();
    T_.block<3, 3>(0, 0) = R;
    T_.block<3, 1>(0, 3) = p;
}

Transform::Transform(const Eigen::Matrix4d& T) : T_(T) {}

Transform::Transform(const Eigen::Vector3d& translation) {
    T_ = Eigen::Matrix4d::Identity();
    T_.block<3, 1>(0, 3) = translation;
}

Transform::Transform(const Eigen::Isometry3d& iso) {
    T_ = iso.matrix();
}

// Factory methods

Transform Transform::Identity() {
    return Transform();
}

Transform Transform::Translation(double x, double y, double z) {
    return Transform(Eigen::Vector3d(x, y, z));
}

Transform Transform::Translation(const Eigen::Vector3d& p) {
    return Transform(p);
}

Transform Transform::RotX(double angle) {
    Eigen::Matrix3d R;
    double c = std::cos(angle);
    double s = std::sin(angle);
    R << 1, 0,  0,
         0, c, -s,
         0, s,  c;
    return Transform(R, Eigen::Vector3d::Zero());
}

Transform Transform::RotY(double angle) {
    Eigen::Matrix3d R;
    double c = std::cos(angle);
    double s = std::sin(angle);
    R <<  c, 0, s,
          0, 1, 0,
         -s, 0, c;
    return Transform(R, Eigen::Vector3d::Zero());
}

Transform Transform::RotZ(double angle) {
    Eigen::Matrix3d R;
    double c = std::cos(angle);
    double s = std::sin(angle);
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return Transform(R, Eigen::Vector3d::Zero());
}

// Accessors

void Transform::decompose(Eigen::Matrix3d& R, Eigen::Vector3d& p) const {
    R = T_.block<3, 3>(0, 0);
    p = T_.block<3, 1>(0, 3);
}

Eigen::Isometry3d Transform::toIsometry() const {
    Eigen::Isometry3d iso;
    iso.matrix() = T_;
    return iso;
}

// Operations

Transform Transform::inverse() const {
    // For SE(3), inverse is:
    // ⎡ R^T  -R^T*p ⎤
    // ⎣ 0    1      ⎦
    Eigen::Matrix3d R = T_.block<3, 3>(0, 0);
    Eigen::Vector3d p = T_.block<3, 1>(0, 3);
    Eigen::Matrix3d R_T = R.transpose();
    Eigen::Vector3d p_inv = -R_T * p;
    return Transform(R_T, p_inv);
}

Transform Transform::operator*(const Transform& other) const {
    Eigen::Matrix4d result = T_ * other.T_;
    return Transform(result);
}

Eigen::Vector3d Transform::operator*(const Eigen::Vector3d& p) const {
    // Transform point using homogeneous coordinates
    Eigen::Vector4d p_h;
    p_h << p, 1.0;
    Eigen::Vector4d result = T_ * p_h;
    return result.head<3>();
}

Transform& Transform::operator*=(const Transform& other) {
    T_ *= other.T_;
    return *this;
}

bool Transform::isApprox(const Transform& other, double tol) const {
    return T_.isApprox(other.T_, tol);
}

std::ostream& operator<<(std::ostream& os, const Transform& T) {
    os << "Transform:\n" << T.matrix();
    return os;
}

} // namespace math
} // namespace robospace
