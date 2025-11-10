#pragma once

#include <robospace/model/entity.hpp>
#include <string>
#include <Eigen/Dense>

namespace robospace {
namespace model {

/**
 * @brief Rigid body link with physical properties and geometry
 *
 * Inherits from Entity to participate in scene graph. Stores mass, inertia,
 * and mesh paths (full geometry support in later phases).
 */
class Link : public Entity {
public:
    explicit Link(const std::string& name);

    // Physical properties
    void set_mass(double mass) { mass_ = mass; }
    double mass() const { return mass_; }

    void set_com(const Eigen::Vector3d& com) { com_ = com; }
    const Eigen::Vector3d& com() const { return com_; }

    void set_inertia(const Eigen::Matrix3d& inertia) { inertia_ = inertia; }
    const Eigen::Matrix3d& inertia() const { return inertia_; }

    void set_inertia_diagonal(double ixx, double iyy, double izz) {
        inertia_ = Eigen::Vector3d(ixx, iyy, izz).asDiagonal();
    }

    bool has_inertial() const { return mass_ > 0.0; }

    // Geometry (mesh file paths for Phase 1)
    void set_visual_mesh(const std::string& mesh_path) {
        visual_mesh_ = mesh_path;
    }
    const std::string& visual_mesh() const { return visual_mesh_; }
    bool has_visual() const { return !visual_mesh_.empty(); }

    void set_collision_mesh(const std::string& mesh_path) {
        collision_mesh_ = mesh_path;
    }
    const std::string& collision_mesh() const { return collision_mesh_; }
    bool has_collision() const { return !collision_mesh_.empty(); }

private:
    double mass_ = 0.0;
    Eigen::Vector3d com_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Zero();

    std::string visual_mesh_;
    std::string collision_mesh_;
};

} // namespace model
} // namespace robospace
