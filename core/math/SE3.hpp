#pragma once

#include "se3.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace robospace {
namespace math {

/**
 * @brief SE(3) Lie group element
 *
 * Represents a rigid body transformation using the Modern Robotics formulation.
 * SE(3) is the special Euclidean group in 3D.
 *
 * Internally represented as:
 * g = ⎡ R  p ⎤ where R ∈ SO(3), p ∈ ℝ³
 *     ⎣ 0  1 ⎦
 *
 * Related to se(3) via exponential map: g = exp(ξ)
 */
class SE3 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct identity element
     */
    SE3();

    /**
     * @brief Construct from rotation and translation
     * @param R 3×3 rotation matrix
     * @param p 3D translation vector
     */
    SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);

    /**
     * @brief Construct from 4×4 matrix
     * @param g 4×4 homogeneous transformation matrix
     */
    explicit SE3(const Eigen::Matrix4d& g);

    // Factory methods

    /**
     * @brief Create identity element
     */
    static SE3 Identity();

    /**
     * @brief Create from translation only
     * @param p Translation vector
     */
    static SE3 Translation(const Eigen::Vector3d& p);

    // Accessors

    /**
     * @brief Get 4×4 matrix representation
     */
    const Eigen::Matrix4d& matrix() const { return g_; }

    /**
     * @brief Get rotation part
     */
    Eigen::Matrix3d rotation() const { return g_.block<3, 3>(0, 0); }

    /**
     * @brief Get translation part
     */
    Eigen::Vector3d translation() const { return g_.block<3, 1>(0, 3); }

    /**
     * @brief Decompose into rotation and translation
     */
    void decompose(Eigen::Matrix3d& R, Eigen::Vector3d& p) const;

    // Group operations

    /**
     * @brief Group composition (multiplication)
     * @param other Another SE3 element
     * @return g1 * g2
     */
    SE3 operator*(const SE3& other) const;

    /**
     * @brief Transform a point
     * @param p Point in 3D
     * @return Transformed point
     */
    Eigen::Vector3d operator*(const Eigen::Vector3d& p) const;

    /**
     * @brief Group inverse
     * @return g^{-1}
     */
    SE3 inverse() const;

    /**
     * @brief Adjoint transformation matrix
     * @return 6×6 adjoint matrix Ad_g
     */
    Matrix6d adjoint() const;

    /**
     * @brief Check equality (within tolerance)
     */
    bool isApprox(const SE3& other, double tol = 1e-9) const;

    /**
     * @brief Print to output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const SE3& g);

private:
    Eigen::Matrix4d g_;  ///< 4×4 matrix representation
};

/**
 * @brief Exponential map: se(3) → SE(3)
 *
 * Computes g = exp(ξ) using Rodrigues' formula:
 * - If ||ω|| = 0: pure translation
 * - If ||ω|| ≠ 0: screw motion
 *
 * @param xi Twist (se3 element)
 * @return SE3 element
 */
SE3 exp_se3(const se3& xi);

/**
 * @brief Logarithm map: SE(3) → se(3)
 *
 * Computes ξ = log(g), the inverse of the exponential map.
 *
 * @param g SE3 element
 * @return Twist (se3 element)
 */
se3 log_SE3(const SE3& g);

/**
 * @brief Adjoint transformation: se(3) → se(3)
 *
 * Transforms a twist from one coordinate frame to another:
 * ξ' = Ad_g ξ
 *
 * @param g SE3 transformation
 * @param xi Twist in original frame
 * @return Twist in transformed frame
 */
se3 adjoint_SE3(const SE3& g, const se3& xi);

} // namespace math
} // namespace robospace
