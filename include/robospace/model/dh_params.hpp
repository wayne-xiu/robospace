#pragma once

#include <robospace/math/SE3.hpp>

namespace robospace {
namespace model {

/**
 * @brief DH convention used by different robot manufacturers
 *
 * STANDARD (Universal Robots, Yaskawa): Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
 * MODIFIED (ABB, Fanuc, Stäubli): Rot_x(α) * Trans_x(a) * Trans_z(d) * Rot_z(θ)
 */
enum class DHConvention {
    STANDARD,
    MODIFIED
};

/**
 * @brief Denavit-Hartenberg parameters for joint kinematics
 *
 * Standard DH parameters (α, a, d, θ) plus optional calibration offset.
 * Supports both Standard and Modified conventions.
 */
struct DHParams {
    double alpha;
    double a;
    double d;
    double theta;
    DHConvention convention;
    double offset;

    DHParams(double alpha = 0.0, double a = 0.0, double d = 0.0,
             double theta = 0.0, DHConvention convention = DHConvention::MODIFIED,
             double offset = 0.0)
        : alpha(alpha), a(a), d(d), theta(theta), convention(convention), offset(offset) {}

    // Compute transformation: q added to theta (revolute) or d (prismatic)
    math::SE3 transform(double q, bool is_prismatic = false) const;

    // Convention-specific transforms
    math::SE3 transform_standard(double q, bool is_prismatic) const;
    math::SE3 transform_modified(double q, bool is_prismatic) const;
};

} // namespace model
} // namespace robospace
