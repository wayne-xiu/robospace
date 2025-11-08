#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace robospace {
namespace math {

/**
 * @brief SE(3) transformation represented as a 4×4 homogeneous matrix
 *
 * Represents rigid body transformations in 3D space combining rotation and translation.
 * This is the classical representation used in robotics.
 *
 * Matrix form:
 * ⎡ R  p ⎤
 * ⎣ 0  1 ⎦
 *
 * where R ∈ SO(3) is rotation and p ∈ ℝ³ is translation.
 */
class Transform {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct identity transformation
     */
    Transform();

    /**
     * @brief Construct from rotation matrix and translation vector
     * @param R 3×3 rotation matrix
     * @param p 3D translation vector
     */
    Transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);

    /**
     * @brief Construct from 4×4 homogeneous matrix
     * @param T 4×4 homogeneous transformation matrix
     */
    explicit Transform(const Eigen::Matrix4d& T);

    /**
     * @brief Construct from translation only (identity rotation)
     * @param translation 3D translation vector
     */
    explicit Transform(const Eigen::Vector3d& translation);

    /**
     * @brief Construct from Eigen::Isometry3d
     * @param iso Eigen's isometry transformation
     */
    explicit Transform(const Eigen::Isometry3d& iso);

    // Factory methods

    /**
     * @brief Create identity transformation
     */
    static Transform Identity();

    /**
     * @brief Create translation-only transformation
     * @param x Translation in x
     * @param y Translation in y
     * @param z Translation in z
     */
    static Transform Translation(double x, double y, double z);

    /**
     * @brief Create translation-only transformation
     * @param p Translation vector
     */
    static Transform Translation(const Eigen::Vector3d& p);

    /**
     * @brief Create rotation around X axis
     * @param angle Rotation angle in radians
     */
    static Transform RotX(double angle);

    /**
     * @brief Create rotation around Y axis
     * @param angle Rotation angle in radians
     */
    static Transform RotY(double angle);

    /**
     * @brief Create rotation around Z axis
     * @param angle Rotation angle in radians
     */
    static Transform RotZ(double angle);

    // Accessors

    /**
     * @brief Get the 4×4 homogeneous matrix
     */
    const Eigen::Matrix4d& matrix() const { return T_; }

    /**
     * @brief Get the rotation part (3×3)
     */
    Eigen::Matrix3d rotation() const { return T_.block<3, 3>(0, 0); }

    /**
     * @brief Get the translation part (3D vector)
     */
    Eigen::Vector3d translation() const { return T_.block<3, 1>(0, 3); }

    /**
     * @brief Get rotation and translation separately
     * @param R Output rotation matrix
     * @param p Output translation vector
     */
    void decompose(Eigen::Matrix3d& R, Eigen::Vector3d& p) const;

    /**
     * @brief Convert to Eigen::Isometry3d
     */
    Eigen::Isometry3d toIsometry() const;

    // Operations

    /**
     * @brief Compute inverse transformation
     */
    Transform inverse() const;

    /**
     * @brief Compose two transformations (matrix multiplication)
     * @param other Another transformation
     * @return T1 * T2
     */
    Transform operator*(const Transform& other) const;

    /**
     * @brief Transform a 3D point
     * @param p Point to transform
     * @return Transformed point
     */
    Eigen::Vector3d operator*(const Eigen::Vector3d& p) const;

    /**
     * @brief Compose with another transformation (in-place)
     * @param other Another transformation
     */
    Transform& operator*=(const Transform& other);

    /**
     * @brief Check equality (within tolerance)
     * @param other Another transformation
     * @param tol Tolerance for comparison (default: 1e-9)
     */
    bool isApprox(const Transform& other, double tol = 1e-9) const;

    /**
     * @brief Print transformation to output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const Transform& T);

private:
    Eigen::Matrix4d T_;  ///< 4×4 homogeneous transformation matrix
};

} // namespace math
} // namespace robospace
