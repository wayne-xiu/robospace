#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace robospace {
namespace math {

// Forward declarations
class SO3;

/**
 * @brief so(3) Lie algebra element (angular velocity)
 *
 * Represents angular velocity in 3D.
 * An element ω ∈ so(3) is a 3D vector.
 *
 * 3D vector form: ω ∈ ℝ³
 *
 * 3×3 matrix form (bracket [ω]):
 * ⎡  0   -ω₃   ω₂ ⎤
 * ⎢  ω₃   0   -ω₁ ⎥
 * ⎣ -ω₂   ω₁   0  ⎦
 * (skew-symmetric matrix)
 */
class so3 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct zero angular velocity
     */
    so3();

    /**
     * @brief Construct from 3D angular velocity vector
     * @param omega Angular velocity
     */
    explicit so3(const Eigen::Vector3d& omega);

    /**
     * @brief Construct from 3×3 skew-symmetric matrix
     * @param bracket 3×3 skew-symmetric matrix [ω]
     */
    explicit so3(const Eigen::Matrix3d& bracket);

    // Factory methods

    /**
     * @brief Create zero angular velocity
     */
    static so3 Zero();

    // Accessors

    /**
     * @brief Get as 3D vector
     */
    const Eigen::Vector3d& vector() const { return omega_; }

    /**
     * @brief Get 3×3 skew-symmetric matrix [ω]
     */
    Eigen::Matrix3d bracket() const;

    // Operations

    /**
     * @brief Lie bracket (cross product) [ω₁, ω₂] = ω₁ × ω₂
     */
    so3 lieBracket(const so3& other) const;

    /**
     * @brief Add two angular velocities
     */
    so3 operator+(const so3& other) const;

    /**
     * @brief Subtract two angular velocities
     */
    so3 operator-(const so3& other) const;

    /**
     * @brief Scalar multiplication
     */
    so3 operator*(double scalar) const;

    /**
     * @brief Scalar division
     */
    so3 operator/(double scalar) const;

    /**
     * @brief Negation
     */
    so3 operator-() const;

    /**
     * @brief Check equality (within tolerance)
     */
    bool isApprox(const so3& other, double tol = 1e-9) const;

    /**
     * @brief Print to output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const so3& omega);

private:
    Eigen::Vector3d omega_;  ///< Angular velocity
};

/**
 * @brief Scalar multiplication (scalar * so3)
 */
so3 operator*(double scalar, const so3& omega);

} // namespace math
} // namespace robospace
