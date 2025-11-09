#pragma once

#include <string>
#include <Eigen/Dense>

namespace robospace {
namespace model {

/**
 * @brief Robot link (rigid body)
 *
 * A link represents a rigid body in the robot with physical properties
 * (mass, center of mass, inertia) and geometry (visual and collision meshes).
 *
 * For Phase 1, geometry is just stored as file paths. Full geometry
 * representation will be added in later phases.
 */
class Link {
public:
    /**
     * @brief Construct a link
     * @param name Link name
     */
    explicit Link(const std::string& name);

    // ========================================================================
    // Identification
    // ========================================================================

    const std::string& name() const { return name_; }
    void set_name(const std::string& name) { name_ = name; }

    // ========================================================================
    // Physical Properties (for dynamics)
    // ========================================================================

    /**
     * @brief Set mass of the link
     * @param mass Mass in kilograms
     */
    void set_mass(double mass) { mass_ = mass; }
    double mass() const { return mass_; }

    /**
     * @brief Set center of mass in link frame
     * @param com Center of mass position (meters)
     */
    void set_com(const Eigen::Vector3d& com) { com_ = com; }
    const Eigen::Vector3d& com() const { return com_; }

    /**
     * @brief Set inertia tensor about center of mass
     * @param inertia 3x3 inertia matrix (kg⋅m²)
     */
    void set_inertia(const Eigen::Matrix3d& inertia) { inertia_ = inertia; }
    const Eigen::Matrix3d& inertia() const { return inertia_; }

    /**
     * @brief Set inertia from principal moments (diagonal inertia)
     * @param ixx Moment about x-axis
     * @param iyy Moment about y-axis
     * @param izz Moment about z-axis
     */
    void set_inertia_diagonal(double ixx, double iyy, double izz) {
        inertia_ = Eigen::Vector3d(ixx, iyy, izz).asDiagonal();
    }

    bool has_inertial() const { return mass_ > 0.0; }

    // ========================================================================
    // Geometry (placeholder for Phase 1)
    // ========================================================================

    /**
     * @brief Set visual mesh file path
     * @param mesh_path Path to mesh file (STL, OBJ, DAE, etc.)
     */
    void set_visual_mesh(const std::string& mesh_path) {
        visual_mesh_ = mesh_path;
    }
    const std::string& visual_mesh() const { return visual_mesh_; }
    bool has_visual() const { return !visual_mesh_.empty(); }

    /**
     * @brief Set collision mesh file path
     * @param mesh_path Path to collision mesh file
     */
    void set_collision_mesh(const std::string& mesh_path) {
        collision_mesh_ = mesh_path;
    }
    const std::string& collision_mesh() const { return collision_mesh_; }
    bool has_collision() const { return !collision_mesh_.empty(); }

private:
    // Identification
    std::string name_;

    // Physical properties
    double mass_ = 0.0;                              ///< Mass (kg)
    Eigen::Vector3d com_ = Eigen::Vector3d::Zero(); ///< Center of mass (m)
    Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Zero(); ///< Inertia tensor (kg⋅m²)

    // Geometry (just paths for Phase 1)
    std::string visual_mesh_;      ///< Visual mesh file path
    std::string collision_mesh_;   ///< Collision mesh file path
};

} // namespace model
} // namespace robospace
