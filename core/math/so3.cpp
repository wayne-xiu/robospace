#include "so3.hpp"
#include "se3.hpp"  // for skew() and unskew()
#include <stdexcept>

namespace robospace {
namespace math {

so3::so3() : omega_(Eigen::Vector3d::Zero()) {}

so3::so3(const Eigen::Vector3d& omega) : omega_(omega) {}

so3::so3(const Eigen::Matrix3d& bracket) {
    omega_ = unskew(bracket);
}

so3 so3::Zero() {
    return so3();
}

Eigen::Matrix3d so3::bracket() const {
    return skew(omega_);
}

so3 so3::lieBracket(const so3& other) const {
    // Lie bracket is cross product for so(3)
    Eigen::Vector3d result = omega_.cross(other.omega_);
    return so3(result);
}

so3 so3::operator+(const so3& other) const {
    Eigen::Vector3d result = omega_ + other.omega_;
    return so3(result);
}

so3 so3::operator-(const so3& other) const {
    Eigen::Vector3d result = omega_ - other.omega_;
    return so3(result);
}

so3 so3::operator*(double scalar) const {
    Eigen::Vector3d result = omega_ * scalar;
    return so3(result);
}

so3 so3::operator/(double scalar) const {
    if (std::abs(scalar) < 1e-10) {
        throw std::runtime_error("Division by near-zero scalar");
    }
    Eigen::Vector3d result = omega_ / scalar;
    return so3(result);
}

so3 so3::operator-() const {
    Eigen::Vector3d result = -omega_;
    return so3(result);
}

bool so3::isApprox(const so3& other, double tol) const {
    return omega_.isApprox(other.omega_, tol);
}

std::ostream& operator<<(std::ostream& os, const so3& omega) {
    os << "so3([" << omega.vector().transpose() << "])";
    return os;
}

so3 operator*(double scalar, const so3& omega) {
    return omega * scalar;
}

} // namespace math
} // namespace robospace
