#pragma once

#include <robospace/math/so3.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace robospace {
namespace math {

/// Euler angle conventions
enum class EulerConvention { XYZ, ZYX, ZYZ, XYX };

/**
 * @brief SO(3) Lie group element (rotation)
 *
 * Represents a rotation using the Modern Robotics formulation.
 * SO(3) is the special orthogonal group in 3D.
 *
 * Internally represented as a 3×3 rotation matrix R where:
 * R^T * R = I and det(R) = 1
 *
 * Related to so(3) via exponential map: R = exp(ω)
 */
class SO3 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct identity rotation
     */
    SO3();

    /**
     * @brief Construct from rotation matrix
     * @param R 3×3 rotation matrix
     */
    explicit SO3(const Eigen::Matrix3d& R);

    /**
     * @brief Construct from axis-angle
     * @param axis Rotation axis (will be normalized)
     * @param angle Rotation angle in radians
     */
    SO3(const Eigen::Vector3d& axis, double angle);

    // Factory methods

    /**
     * @brief Create identity rotation
     */
    static SO3 Identity();

    /**
     * @brief Create from axis-angle
     * @param axis Rotation axis
     * @param angle Rotation angle in radians
     */
    static SO3 FromAxisAngle(const Eigen::Vector3d& axis, double angle);

    /**
     * @brief Create rotation around X axis
     * @param angle Rotation angle in radians
     */
    static SO3 RotX(double angle);

    /**
     * @brief Create rotation around Y axis
     * @param angle Rotation angle in radians
     */
    static SO3 RotY(double angle);

    static SO3 RotZ(double angle);

    /// Create from Roll-Pitch-Yaw angles (XYZ convention)
    static SO3 FromRPY(double roll, double pitch, double yaw);

    /// Create from Euler angles with specified convention
    static SO3 FromEuler(double alpha, double beta, double gamma,
                         EulerConvention convention = EulerConvention::XYZ);

    /// Create from quaternion
    static SO3 FromQuaternion(const Eigen::Quaterniond& q);

    /// Create from rotation vector (axis * angle)
    static SO3 FromRotationVector(const Eigen::Vector3d& rotvec);

    // Accessors

    /**
     * @brief Get 3×3 rotation matrix
     */
    const Eigen::Matrix3d& matrix() const { return R_; }

    /**
     * @brief Get axis-angle representation
     * @param axis Output: rotation axis (unit vector)
     * @param angle Output: rotation angle in radians [0, π]
     */
    void axisAngle(Eigen::Vector3d& axis, double& angle) const;

    /// Get as quaternion
    Eigen::Quaterniond quaternion() const;

    /// Get Roll-Pitch-Yaw angles (XYZ convention)
    Eigen::Vector3d rpy() const;

    /// Get Euler angles with specified convention
    Eigen::Vector3d euler(EulerConvention convention = EulerConvention::XYZ) const;

    /// Get rotation vector (axis * angle)
    Eigen::Vector3d rotationVector() const;

    // Group operations

    /**
     * @brief Group composition (multiplication)
     * @param other Another SO3 element
     * @return R1 * R2
     */
    SO3 operator*(const SO3& other) const;

    /**
     * @brief Rotate a vector
     * @param v Vector to rotate
     * @return Rotated vector
     */
    Eigen::Vector3d operator*(const Eigen::Vector3d& v) const;

    /**
     * @brief Group inverse (transpose)
     * @return R^T
     */
    SO3 inverse() const;

    /**
     * @brief Check equality (within tolerance)
     */
    bool isApprox(const SO3& other, double tol = 1e-9) const;

    /**
     * @brief Print to output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const SO3& R);

private:
    Eigen::Matrix3d R_;  ///< Rotation matrix
};

/**
 * @brief Exponential map: so(3) → SO(3)
 *
 * Computes R = exp(ω) using Rodrigues' formula:
 * R = I + sin(θ)[ω̂] + (1-cos(θ))[ω̂]²
 * where θ = ||ω|| and ω̂ = ω/θ
 *
 * @param omega Angular velocity (so3 element)
 * @return SO3 rotation
 */
SO3 exp_so3(const so3& omega);

/**
 * @brief Logarithm map: SO(3) → so(3)
 *
 * Computes ω = log(R), the inverse of the exponential map.
 *
 * @param R SO3 rotation
 * @return Angular velocity (so3 element)
 */
so3 log_SO3(const SO3& R);

} // namespace math
} // namespace robospace
