#pragma once

#include <robospace/math/SO3.hpp>  // For EulerConvention
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace robospace {
namespace math {

// EulerConvention is now defined in SO3.hpp

/**
 * @brief SO(3) rotation with multiple representations
 *
 * Supports:
 * - Rotation matrix (3×3 orthogonal matrix)
 * - Quaternion (unit quaternion)
 * - Euler angles (various conventions)
 * - Axis-angle representation
 */
class Rotation {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct identity rotation
     */
    Rotation();

    /**
     * @brief Construct from rotation matrix
     * @param R 3×3 rotation matrix (must be orthogonal)
     */
    explicit Rotation(const Eigen::Matrix3d& R);

    /**
     * @brief Construct from quaternion
     * @param q Unit quaternion (will be normalized)
     */
    explicit Rotation(const Eigen::Quaterniond& q);

    /**
     * @brief Construct from axis-angle
     * @param axis Rotation axis (will be normalized)
     * @param angle Rotation angle in radians
     */
    Rotation(const Eigen::Vector3d& axis, double angle);

    // Factory methods

    /**
     * @brief Create identity rotation
     */
    static Rotation Identity();

    /**
     * @brief Create from Euler angles
     * @param roll Rotation around X (radians)
     * @param pitch Rotation around Y (radians)
     * @param yaw Rotation around Z (radians)
     * @param convention Euler angle convention (default: XYZ)
     */
    static Rotation FromEuler(double roll, double pitch, double yaw,
                              EulerConvention convention = EulerConvention::XYZ);

    /**
     * @brief Create from Euler angles (vector form)
     * @param angles [roll, pitch, yaw] in radians
     * @param convention Euler angle convention (default: XYZ)
     */
    static Rotation FromEuler(const Eigen::Vector3d& angles,
                              EulerConvention convention = EulerConvention::XYZ);

    /**
     * @brief Create from axis-angle
     * @param axis Rotation axis (will be normalized)
     * @param angle Rotation angle in radians
     */
    static Rotation FromAxisAngle(const Eigen::Vector3d& axis, double angle);

    /**
     * @brief Create from rotation vector (axis * angle)
     * @param rotvec Rotation vector (direction = axis, magnitude = angle)
     */
    static Rotation FromRotationVector(const Eigen::Vector3d& rotvec);

    /**
     * @brief Create rotation around X axis
     * @param angle Rotation angle in radians
     */
    static Rotation RotX(double angle);

    /**
     * @brief Create rotation around Y axis
     * @param angle Rotation angle in radians
     */
    static Rotation RotY(double angle);

    /**
     * @brief Create rotation around Z axis
     * @param angle Rotation angle in radians
     */
    static Rotation RotZ(double angle);

    // Accessors

    /**
     * @brief Get rotation as 3×3 matrix
     */
    const Eigen::Matrix3d& matrix() const { return R_; }

    /**
     * @brief Get rotation as unit quaternion
     */
    Eigen::Quaterniond quaternion() const;

    /**
     * @brief Get Euler angles
     * @param convention Euler angle convention
     * @return [roll, pitch, yaw] in radians
     */
    Eigen::Vector3d eulerAngles(EulerConvention convention = EulerConvention::XYZ) const;

    /**
     * @brief Get axis-angle representation
     * @param axis Output: rotation axis (unit vector)
     * @param angle Output: rotation angle in radians [0, π]
     */
    void axisAngle(Eigen::Vector3d& axis, double& angle) const;

    /**
     * @brief Get rotation vector (axis * angle)
     * @return Rotation vector (direction = axis, magnitude = angle)
     */
    Eigen::Vector3d rotationVector() const;

    // Operations

    /**
     * @brief Compute inverse rotation (transpose for orthogonal matrix)
     */
    Rotation inverse() const;

    /**
     * @brief Compose two rotations
     * @param other Another rotation
     * @return R1 * R2
     */
    Rotation operator*(const Rotation& other) const;

    /**
     * @brief Rotate a 3D vector
     * @param v Vector to rotate
     * @return Rotated vector
     */
    Eigen::Vector3d operator*(const Eigen::Vector3d& v) const;

    /**
     * @brief Compose with another rotation (in-place)
     * @param other Another rotation
     */
    Rotation& operator*=(const Rotation& other);

    /**
     * @brief Check equality (within tolerance)
     * @param other Another rotation
     * @param tol Tolerance for comparison (default: 1e-9)
     */
    bool isApprox(const Rotation& other, double tol = 1e-9) const;

    /**
     * @brief Print rotation to output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const Rotation& R);

private:
    Eigen::Matrix3d R_;  ///< Rotation matrix representation
};

} // namespace math
} // namespace robospace
