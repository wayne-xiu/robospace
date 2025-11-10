#pragma once

#include <robospace/math/SE3.hpp>
#include <string>
#include <vector>

namespace robospace {
namespace model {

/**
 * @brief Base class for scene graph objects (Frame, Link, Robot)
 *
 * Entities form parent-child trees representing spatial relationships.
 * Each entity has a pose relative to its parent. World pose is computed
 * by traversing the tree to the root.
 *
 * Enables flexible scene graphs: robot on table, sensor on link, nested frames.
 */
class Entity {
public:
    enum class Type { FRAME, LINK, ROBOT, TOOL, TARGET };

    Entity(const std::string& name, Type type);
    virtual ~Entity() = default;

    // Identification
    const std::string& name() const { return name_; }
    void set_name(const std::string& name) { name_ = name; }
    Type type() const { return type_; }

    // Scene graph
    void set_parent(Entity* parent);
    Entity* parent() const { return parent_; }
    const std::vector<Entity*>& children() const { return children_; }
    bool is_root() const { return parent_ == nullptr; }
    int depth() const;

    // Pose
    void set_pose(const math::SE3& pose) { pose_ = pose; }
    const math::SE3& pose() const { return pose_; }
    math::SE3 pose_world() const;
    Eigen::Vector3d translation() const { return pose_.translation(); }
    Eigen::Matrix3d rotation() const { return pose_.rotation(); }

protected:
    std::string name_;
    Type type_;
    math::SE3 pose_;
    Entity* parent_ = nullptr;
    std::vector<Entity*> children_;

    void add_child(Entity* child);
    void remove_child(Entity* child);
};

/**
 * @brief Compute relative transform between entities
 *
 * Returns T_target_source: transform that expresses source in target coordinates.
 * Example: compute_transform(tcp, base) returns T_base_tcp.
 */
math::SE3 compute_transform(const Entity* source, const Entity* target);

} // namespace model
} // namespace robospace
