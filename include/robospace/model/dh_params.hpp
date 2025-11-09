#pragma once

#include <robospace/math/SE3.hpp>

namespace robospace {
namespace model {

/**
 * @brief DH parameter convention
 *
 * Different robot manufacturers use different DH conventions:
 * - STANDARD: Universal Robots, Yaskawa/Motoman
 *   Transform: Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
 * - MODIFIED: ABB, Fanuc, Stäubli (Craig's convention)
 *   Transform: Rot_x(α) * Trans_x(a) * Trans_z(d) * Rot_z(θ)
 */
enum class DHConvention {
    STANDARD,   ///< Standard DH (Denavit-Hartenberg)
    MODIFIED    ///< Modified DH (Craig's convention)
};

/**
 * @brief Denavit-Hartenberg parameters for a robot joint
 *
 * Supports both Standard and Modified DH conventions. The four parameters are:
 * - α (alpha): Twist angle - rotation about x-axis (radians)
 * - a: Link length - translation along x-axis (meters)
 * - d: Link offset - translation along z-axis (meters)
 * - θ (theta): Joint angle - rotation about z-axis (radians)
 *
 * For a revolute joint, θ is the variable (q)
 * For a prismatic joint, d is the variable (q)
 */
struct DHParams {
    double alpha;           ///< Twist angle (radians)
    double a;               ///< Link length (meters)
    double d;               ///< Link offset (meters)
    double theta;           ///< Joint angle offset (radians)
    DHConvention convention; ///< DH convention used

    /**
     * @brief Construct DH parameters
     * @param alpha Twist angle (radians)
     * @param a Link length (meters)
     * @param d Link offset (meters)
     * @param theta Joint angle offset (radians)
     * @param convention DH convention (default: MODIFIED)
     */
    DHParams(double alpha = 0.0, double a = 0.0, double d = 0.0,
             double theta = 0.0, DHConvention convention = DHConvention::MODIFIED)
        : alpha(alpha), a(a), d(d), theta(theta), convention(convention) {}

    /**
     * @brief Compute transformation matrix for this DH parameter set
     *
     * For revolute joints: q is added to theta
     * For prismatic joints: q is added to d
     *
     * @param q Joint variable (angle for revolute, distance for prismatic)
     * @param is_prismatic True if joint is prismatic, false if revolute
     * @return SE3 transformation from frame i-1 to frame i
     */
    math::SE3 transform(double q, bool is_prismatic = false) const;

    /**
     * @brief Compute transformation using Standard DH convention
     * Transform: Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
     */
    math::SE3 transform_standard(double q, bool is_prismatic) const;

    /**
     * @brief Compute transformation using Modified DH convention
     * Transform: Rot_x(α) * Trans_x(a) * Trans_z(d) * Rot_z(θ)
     */
    math::SE3 transform_modified(double q, bool is_prismatic) const;
};

} // namespace model
} // namespace robospace
