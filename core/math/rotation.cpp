#include "rotation.hpp"
#include <cmath>
#include <stdexcept>

namespace robospace {
namespace math {

// Constructors

Rotation::Rotation() {
    R_ = Eigen::Matrix3d::Identity();
}

Rotation::Rotation(const Eigen::Matrix3d& R) : R_(R) {
    // Could add validation that R is orthogonal (R^T * R = I)
    // For performance, we skip this in production, but could add in debug mode
}

Rotation::Rotation(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond q_normalized = q.normalized();
    R_ = q_normalized.toRotationMatrix();
}

Rotation::Rotation(const Eigen::Vector3d& axis, double angle) {
    Eigen::Vector3d axis_normalized = axis.normalized();
    Eigen::AngleAxisd aa(angle, axis_normalized);
    R_ = aa.toRotationMatrix();
}

// Factory methods

Rotation Rotation::Identity() {
    return Rotation();
}

Rotation Rotation::FromEuler(double roll, double pitch, double yaw,
                             EulerConvention convention) {
    return FromEuler(Eigen::Vector3d(roll, pitch, yaw), convention);
}

Rotation Rotation::FromEuler(const Eigen::Vector3d& angles,
                             EulerConvention convention) {
    double roll = angles(0);
    double pitch = angles(1);
    double yaw = angles(2);

    Eigen::Matrix3d R;

    switch (convention) {
        case EulerConvention::XYZ: {
            // Roll-Pitch-Yaw: R = Rz(yaw) * Ry(pitch) * Rx(roll)
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            R = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
            break;
        }
        case EulerConvention::ZYX: {
            // Yaw-Pitch-Roll: R = Rx(roll) * Ry(pitch) * Rz(yaw)
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            R = (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
            break;
        }
        case EulerConvention::ZYZ: {
            // ZYZ Euler angles
            Eigen::AngleAxisd z1(roll, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd y(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd z2(yaw, Eigen::Vector3d::UnitZ());
            R = (z1 * y * z2).toRotationMatrix();
            break;
        }
        case EulerConvention::XYX: {
            // XYX Euler angles
            Eigen::AngleAxisd x1(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd y(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd x2(yaw, Eigen::Vector3d::UnitX());
            R = (x1 * y * x2).toRotationMatrix();
            break;
        }
        default:
            throw std::runtime_error("Unknown Euler convention");
    }

    return Rotation(R);
}

Rotation Rotation::FromAxisAngle(const Eigen::Vector3d& axis, double angle) {
    return Rotation(axis, angle);
}

Rotation Rotation::FromRotationVector(const Eigen::Vector3d& rotvec) {
    double angle = rotvec.norm();
    if (angle < 1e-10) {
        return Identity();
    }
    Eigen::Vector3d axis = rotvec / angle;
    return Rotation(axis, angle);
}

Rotation Rotation::RotX(double angle) {
    return FromAxisAngle(Eigen::Vector3d::UnitX(), angle);
}

Rotation Rotation::RotY(double angle) {
    return FromAxisAngle(Eigen::Vector3d::UnitY(), angle);
}

Rotation Rotation::RotZ(double angle) {
    return FromAxisAngle(Eigen::Vector3d::UnitZ(), angle);
}

// Accessors

Eigen::Quaterniond Rotation::quaternion() const {
    return Eigen::Quaterniond(R_);
}

Eigen::Vector3d Rotation::eulerAngles(EulerConvention convention) const {
    // Using Eigen's built-in euler angles method
    // Note: Eigen uses intrinsic rotations (ZYX order for RPY)
    switch (convention) {
        case EulerConvention::XYZ:
            return R_.eulerAngles(2, 1, 0).reverse();  // ZYX -> XYZ
        case EulerConvention::ZYX:
            return R_.eulerAngles(0, 1, 2);
        case EulerConvention::ZYZ:
            return R_.eulerAngles(2, 1, 2);
        case EulerConvention::XYX:
            return R_.eulerAngles(0, 1, 0);
        default:
            throw std::runtime_error("Unknown Euler convention");
    }
}

void Rotation::axisAngle(Eigen::Vector3d& axis, double& angle) const {
    Eigen::AngleAxisd aa(R_);
    axis = aa.axis();
    angle = aa.angle();
}

Eigen::Vector3d Rotation::rotationVector() const {
    Eigen::Vector3d axis;
    double angle;
    axisAngle(axis, angle);
    return axis * angle;
}

// Operations

Rotation Rotation::inverse() const {
    return Rotation(R_.transpose());
}

Rotation Rotation::operator*(const Rotation& other) const {
    return Rotation(R_ * other.R_);
}

Eigen::Vector3d Rotation::operator*(const Eigen::Vector3d& v) const {
    return R_ * v;
}

Rotation& Rotation::operator*=(const Rotation& other) {
    R_ *= other.R_;
    return *this;
}

bool Rotation::isApprox(const Rotation& other, double tol) const {
    return R_.isApprox(other.R_, tol);
}

std::ostream& operator<<(std::ostream& os, const Rotation& R) {
    os << "Rotation:\n" << R.matrix();
    return os;
}

} // namespace math
} // namespace robospace
