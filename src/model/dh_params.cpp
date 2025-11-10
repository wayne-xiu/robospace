#include <robospace/model/dh_params.hpp>
#include <robospace/math/SO3.hpp>
#include <cmath>

namespace robospace {
namespace model {

math::SE3 DHParams::transform(double q, bool is_prismatic) const {
    // Apply offset to joint variable
    double q_with_offset = q + offset;

    if (convention == DHConvention::STANDARD) {
        return transform_standard(q_with_offset, is_prismatic);
    } else {
        return transform_modified(q_with_offset, is_prismatic);
    }
}

math::SE3 DHParams::transform_standard(double q, bool is_prismatic) const {
    // Standard DH: Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)

    double theta_total = theta + (is_prismatic ? 0.0 : q);
    double d_total = d + (is_prismatic ? q : 0.0);

    // Build transformation step by step
    // T = Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)

    double ct = std::cos(theta_total);
    double st = std::sin(theta_total);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    // Combined rotation matrix: Rot_z(θ) * Rot_x(α)
    Eigen::Matrix3d R;
    R << ct,      -st*ca,  st*sa,
         st,       ct*ca, -ct*sa,
         0.0,      sa,     ca;

    // Translation: Rot_z(θ) * [a, 0, d]
    Eigen::Vector3d p;
    p << a*ct, a*st, d_total;

    return math::SE3(R, p);
}

math::SE3 DHParams::transform_modified(double q, bool is_prismatic) const {
    // Modified DH (Craig): Rot_x(α) * Trans_x(a) * Trans_z(d) * Rot_z(θ)

    double theta_total = theta + (is_prismatic ? 0.0 : q);
    double d_total = d + (is_prismatic ? q : 0.0);

    // Build transformation step by step
    // T = Rot_x(α) * Trans_x(a) * Trans_z(d) * Rot_z(θ)

    double ct = std::cos(theta_total);
    double st = std::sin(theta_total);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    // Combined rotation matrix: Rot_x(α) * Rot_z(θ)
    Eigen::Matrix3d R;
    R << ct,      -st,     0.0,
         st*ca,    ct*ca, -sa,
         st*sa,    ct*sa,  ca;

    // Translation: [a, -d*sa, d*ca]
    Eigen::Vector3d p;
    p << a, -d_total*sa, d_total*ca;

    return math::SE3(R, p);
}

} // namespace model
} // namespace robospace
