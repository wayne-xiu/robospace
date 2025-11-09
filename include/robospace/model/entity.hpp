#pragma once

#include <robospace/math/SE3.hpp>
#include <string>
#include <vector>

namespace robospace {
namespace model {

/**
 * @brief Entity is the base class for all scene graph objects
 *
 * Entities form a parent-child tree (scene graph) representing spatial
 * relationships in the robot workspace. Examples include:
 * - Frames (coordinate systems)
 * - Links (rigid bodies)
 * - Robots (articulated mechanisms)
 * - Tools, Targets (future)
 *
 * Design philosophy:
 * - Entity is abstract - inherit to create concrete types (Frame, Link, Robot)
 * - Parent-child relationships enable flexible scene hierarchies
 * - Pose is always relative to parent
 * - World pose computed by traversing tree to root
 *
 * This pattern enables:
 * - Robot on table: Robot's base link parents to table Frame
 * - Workpiece fixtures: Nested Frames with targets as children
 * - Sensor mounts: Frame on Link with relative offset
 * - Dynamic reparenting: Move robot to different table/rail
 */
class Entity {
public:
    /**
     * @brief Entity types for runtime type identification
     */
    enum class Type {
        FRAME,      ///< Coordinate system
        LINK,       ///< Physical rigid body
        ROBOT,      ///< Articulated mechanism (future)
        TOOL,       ///< End-effector (future)
        TARGET      ///< Pose target (future)
    };

    /**
     * @brief Construct an entity
     * @param name Entity name (unique identifier)
     * @param type Entity type
     */
    Entity(const std::string& name, Type type);

    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~Entity() = default;

    // ========================================================================
    // Identification
    // ========================================================================

    const std::string& name() const { return name_; }
    void set_name(const std::string& name) { name_ = name; }
    Type type() const { return type_; }

    // ========================================================================
    // Scene Graph (Parent-Child Relationships)
    // ========================================================================

    /**
     * @brief Set parent entity
     * @param parent New parent (nullptr for root)
     *
     * Updates both this entity's parent pointer and parent's children list.
     * Removes from old parent's children if changing parents.
     */
    void set_parent(Entity* parent);

    /**
     * @brief Get parent entity
     * @return Parent pointer (nullptr if root)
     */
    Entity* parent() const { return parent_; }

    /**
     * @brief Get all children
     * @return Vector of child entity pointers
     */
    const std::vector<Entity*>& children() const { return children_; }

    /**
     * @brief Check if this is a root entity (no parent)
     */
    bool is_root() const { return parent_ == nullptr; }

    /**
     * @brief Get depth in tree (distance from root)
     * @return 0 for root, 1 for direct children of root, etc.
     */
    int depth() const;

    // ========================================================================
    // Pose (Transform relative to parent)
    // ========================================================================

    /**
     * @brief Set pose relative to parent
     * @param pose SE3 transformation from parent to this entity
     */
    void set_pose(const math::SE3& pose) { pose_ = pose; }

    /**
     * @brief Get pose relative to parent
     * @return SE3 transformation
     */
    const math::SE3& pose() const { return pose_; }

    /**
     * @brief Compute world pose by traversing tree to root
     * @return SE3 transformation from world to this entity
     *
     * Computed by composing transforms: root → ... → parent → this
     */
    math::SE3 pose_world() const;

    // ========================================================================
    // Convenience accessors
    // ========================================================================

    /**
     * @brief Get translation component of pose
     */
    Eigen::Vector3d translation() const { return pose_.translation(); }

    /**
     * @brief Get rotation component of pose
     */
    Eigen::Matrix3d rotation() const { return pose_.rotation(); }

protected:
    // Identification
    std::string name_;          ///< Entity name
    Type type_;                 ///< Entity type (for runtime checks)

    // Scene graph
    math::SE3 pose_;            ///< Pose relative to parent
    Entity* parent_ = nullptr;  ///< Parent entity (nullptr = root)
    std::vector<Entity*> children_;  ///< Child entities

    // Internal helpers for scene graph management
    void add_child(Entity* child);
    void remove_child(Entity* child);
};

} // namespace model
} // namespace robospace
