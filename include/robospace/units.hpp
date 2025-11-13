#pragma once

#include <Eigen/Dense>
#include <robospace/math/SE3.hpp>
#include <cmath>

namespace robospace {
namespace units {

// Angular conversions
constexpr double PI = 3.14159265358979323846;

inline double deg_to_rad(double degrees) {
    return degrees * PI / 180.0;
}

inline double rad_to_deg(double radians) {
    return radians * 180.0 / PI;
}

inline Eigen::VectorXd deg_to_rad(const Eigen::VectorXd& degrees) {
    return degrees * (PI / 180.0);
}

inline Eigen::VectorXd rad_to_deg(const Eigen::VectorXd& radians) {
    return radians * (180.0 / PI);
}

// Linear conversions
inline double mm_to_m(double millimeters) {
    return millimeters / 1000.0;
}

inline double m_to_mm(double meters) {
    return meters * 1000.0;
}

inline double inch_to_m(double inches) {
    return inches * 0.0254;
}

inline double m_to_inch(double meters) {
    return meters / 0.0254;
}

inline Eigen::Vector3d mm_to_m(const Eigen::Vector3d& millimeters) {
    return millimeters / 1000.0;
}

inline Eigen::Vector3d m_to_mm(const Eigen::Vector3d& meters) {
    return meters * 1000.0;
}

// SE3 conversions (position only, rotation unchanged)
inline math::SE3 mm_to_m(const math::SE3& T_mm) {
    return math::SE3(T_mm.rotation(), mm_to_m(T_mm.translation()));
}

inline math::SE3 m_to_mm(const math::SE3& T_m) {
    return math::SE3(T_m.rotation(), m_to_mm(T_m.translation()));
}

} // namespace units
} // namespace robospace
