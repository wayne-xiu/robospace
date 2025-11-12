#pragma once

#include <robospace/model/entity.hpp>
#include <robospace/model/kinematic_tree.hpp>
#include <robospace/model/link.hpp>
#include <robospace/model/joint.hpp>
#include <robospace/model/frame.hpp>
#include <robospace/model/tool.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace robospace {
namespace model {

/**
 * @brief High-level robot model with kinematic tree and tool management
 *
 * Wraps KinematicTree and manages robot frames and tools.
 * Provides user-friendly name-based API on top of ID-based KinematicTree.
 *
 * Frame hierarchy:
 *   parent (base_ref) -> FK origin (Entity::pose_) -> base_frame_ -> j1...jn -> flange -> tools
 *
 * Key semantics:
 * - Robot::pose() returns TCP pose (overrides Entity::pose() for intuitive API)
 * - Robot::set_pose() moves TCP to target via IK (overrides Entity::set_pose())
 * - Entity::pose_ stores FK origin position relative to parent (base_ref)
 * - base_frame_ is physical base offset from FK origin (for Fanuc-style robots)
 *
 * Inherits from Entity so robot can be placed in world (e.g., on table).
 */
class Robot : public Entity {
public:
    explicit Robot(const std::string& name);
    Robot(const std::string& name, Entity* parent);

    // === FACTORY METHODS ===

    /**
     * @brief Load robot from URDF file
     * @param urdf_path Path to URDF file
     * @return Robot object constructed from URDF
     * @throws std::runtime_error if file not found or parsing fails
     */
    static Robot from_urdf(const std::string& urdf_path);

    /**
     * @brief Load robot from URDF string
     * @param urdf_string URDF XML content as string
     * @return Robot object constructed from URDF
     * @throws std::runtime_error if parsing fails
     */
    static Robot from_urdf_string(const std::string& urdf_string);

    // === BUILDING (Internal use) ===

    // TODO: Make protected in Phase 2, add friend class URDFParser
    void add_link(const Link& link);
    void add_joint(const Joint& joint);

    // Tool management
    int add_tool(const Tool& tool);
    const Tool& tool(int id) const;
    Tool& tool(int id);
    const Tool& tool(const std::string& name) const;
    Tool& tool(const std::string& name);
    int tool_id(const std::string& name) const;
    bool has_tool(const std::string& name) const;
    int num_tools() const { return static_cast<int>(tools_.size()); }

    // Pose overrides (TCP semantics)
    math::SE3 pose() const override;  // Returns TCP pose via FK
    void set_pose(const math::SE3& tcp_target) override;  // IK to move TCP (not yet implemented)

    // Move robot base in scene
    void move_base_by(const math::SE3& delta);  // Move FK origin by relative amount

    // Link/Joint accessors by ID
    const Link& link(int id) const;
    const Joint& joint(int id) const;
    Link& link(int id);
    Joint& joint(int id);

    // Link/Joint accessors by name
    const Link& link(const std::string& name) const;
    const Joint& joint(const std::string& name) const;
    Link& link(const std::string& name);
    Joint& joint(const std::string& name);

    // Link/Joint lookups
    int link_id(const std::string& name) const;
    int joint_id(const std::string& name) const;
    bool has_link(const std::string& name) const;
    bool has_joint(const std::string& name) const;

    // Counts
    int num_links() const { return tree_.num_links(); }
    int num_joints() const { return tree_.num_joints(); }
    int dof() const;  // Degrees of freedom (number of actuated joints)

    // Base frame (physical base relative to base_ref)
    void set_base_frame(const math::SE3& frame);
    const math::SE3& base_frame() const { return base_frame_; }

    // Base link
    int base_link_id() const { return 0; }
    const Link& base_link() const { return link(0); }
    Link& base_link() { return link(0); }

    // Flange (last link by default)
    int flange_link_id() const { return num_links() - 1; }
    const Link& flange_link() const { return link(flange_link_id()); }
    Link& flange_link() { return link(flange_link_id()); }

    // Active tool management
    bool has_active_tool() const { return active_tool_id_ >= 0; }
    int active_tool_id() const { return active_tool_id_; }
    const Tool& active_tool() const;
    Tool& active_tool();
    void set_active_tool(int tool_id);
    void set_active_tool(const std::string& tool_name);

    // Kinematic tree access (for FK/IK later)
    const KinematicTree& kinematic_tree() const { return tree_; }
    KinematicTree& kinematic_tree() { return tree_; }

    // Validation
    bool is_valid() const { return tree_.is_valid(); }

private:
    // Kinematic chain
    KinematicTree tree_;

    // Base frame: physical base pose relative to base_ref (Entity::pose())
    math::SE3 base_frame_ = math::SE3::Identity();

    // Tools attached to flange
    std::vector<Tool> tools_;
    int active_tool_id_ = -1;

    // Name-to-ID maps
    std::unordered_map<std::string, int> link_name_to_id_;
    std::unordered_map<std::string, int> joint_name_to_id_;
    std::unordered_map<std::string, int> tool_name_to_id_;
};

} // namespace model
} // namespace robospace
