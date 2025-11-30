#pragma once

#include <robospace/model/dh_params.hpp>
#include <robospace/math/SE3.hpp>

namespace robospace {
namespace model {

/**
 * @brief Adapter for converting DH parameters to SE3 transformations
 *
 * Purpose: Provides a clean interface for DH → SE3 conversion.
 * Future: Will support one-time conversion at load time to eliminate runtime overhead.
 *
 * Phase 1, Step 1: Foundation - wraps existing DHParams::transform()
 * Phase 1, Step 2+: Will enable pre-computed SE3 storage
 */
class DHAdapter {
public:
    /**
     * @brief Convert DH parameters to SE3 transformation
     *
     * @param dh DH parameters (alpha, a, d, theta, convention, offset)
     * @param q Joint variable (added to theta for revolute, d for prismatic)
     * @param is_prismatic true if prismatic joint, false if revolute
     * @return SE3 transformation matrix
     */
    static math::SE3 convert(const DHParams& dh, double q, bool is_prismatic);

    /**
     * @brief Convert using standard DH convention
     *
     * Standard DH: Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
     *
     * @param dh DH parameters
     * @param q Joint variable
     * @param is_prismatic Joint type
     * @return SE3 transformation
     */
    static math::SE3 convert_standard(const DHParams& dh, double q, bool is_prismatic);

    /**
     * @brief Convert using modified DH convention (Craig)
     *
     * Modified DH: Rot_x(α) * Trans_x(a) * Trans_z(d) * Rot_z(θ)
     *
     * @param dh DH parameters
     * @param q Joint variable
     * @param is_prismatic Joint type
     * @return SE3 transformation
     */
    static math::SE3 convert_modified(const DHParams& dh, double q, bool is_prismatic);
};

} // namespace model
} // namespace robospace
