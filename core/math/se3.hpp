#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace robospace {
namespace math {

// Forward declarations
class SE3;
class so3;

/**
 * @brief se(3) Lie algebra element (twist)
 *
 * Represents a twist (instantaneous velocity) in 3D rigid body motion.
 * A twist ξ ∈ se(3) consists of:
 * - Angular velocity ω ∈ ℝ³
 * - Linear velocity v ∈ ℝ³
 *
 * 6D vector form: ξ = [ω; v] ∈ ℝ⁶
 *
 * 4×4 matrix form (bracket [ξ]):
 * ⎡ [ω]  v ⎤
 * ⎣  0   0 ⎦
 * where [ω] is the skew-symmetric matrix from ω
 */
class se3 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct zero twist
     */
    se3();

    /**
     * @brief Construct from angular and linear velocities
     * @param omega Angular velocity (3D)
     * @param v Linear velocity (3D)
     */
    se3(const Eigen::Vector3d& omega, const Eigen::Vector3d& v);

    /**
     * @brief Construct from 6D vector [ω; v]
     * @param xi 6D twist vector
     */
    explicit se3(const Eigen::Vector6d& xi);

    /**
     * @brief Construct from 4×4 matrix representation
     * @param bracket 4×4 matrix [ξ]
     */
    explicit se3(const Eigen::Matrix4d& bracket);

    // Factory methods

    /**
     * @brief Create zero twist
     */
    static se3 Zero();

    // Accessors

    /**
     * @brief Get angular velocity
     */
    const Eigen::Vector3d& omega() const { return omega_; }

    /**
     * @brief Get linear velocity
     */
    const Eigen::Vector3d& v() const { return v_; }

    /**
     * @brief Get as 6D vector [ω; v]
     */
    Eigen::Vector6d vector() const;

    /**
     * @brief Get 4×4 matrix representation [ξ]
     */
    Eigen::Matrix4d bracket() const;

    // Operations

    /**
     * @brief Lie bracket (commutator) [ξ₁, ξ₂]
     * @param other Another twist
     * @return [this, other]
     */
    se3 lieBracket(const se3& other) const;

    /**
     * @brief Add two twists
     */
    se3 operator+(const se3& other) const;

    /**
     * @brief Subtract two twists
     */
    se3 operator-(const se3& other) const;

    /**
     * @brief Scalar multiplication
     */
    se3 operator*(double scalar) const;

    /**
     * @brief Scalar division
     */
    se3 operator/(double scalar) const;

    /**
     * @brief Negation
     */
    se3 operator-() const;

    /**
     * @brief Check equality (within tolerance)
     */
    bool isApprox(const se3& other, double tol = 1e-9) const;

    /**
     * @brief Print to output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const se3& xi);

private:
    Eigen::Vector3d omega_;  ///< Angular velocity
    Eigen::Vector3d v_;      ///< Linear velocity
};

/**
 * @brief Scalar multiplication (scalar * se3)
 */
se3 operator*(double scalar, const se3& xi);

/**
 * @brief Create skew-symmetric matrix from 3D vector
 * @param v 3D vector
 * @return 3×3 skew-symmetric matrix [v]×
 */
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

/**
 * @brief Extract vector from skew-symmetric matrix
 * @param S 3×3 skew-symmetric matrix
 * @return 3D vector v such that S = [v]×
 */
Eigen::Vector3d unskew(const Eigen::Matrix3d& S);

} // namespace math
} // namespace robospace
