#pragma once

#include <robospace/model/entity.hpp>
#include <robospace/math/SE3.hpp>
#include <string>

namespace robospace {
namespace model {

/**
 * @brief Frame represents a named coordinate system in the scene graph
 *
 * Frames are first-class coordinate systems used to represent:
 * - Tool Center Points (TCP) for end-effectors
 * - Sensor mounting points (cameras, force sensors, etc.)
 * - Calibration target locations (laser tracker SMR mounts)
 * - Workpiece coordinate systems
 * - Task frames for manipulation
 * - Reference frames (world, table, fixture)
 *
 * Design philosophy (inspired by RoboDK/Drake):
 * - Frames inherit from Entity, so they participate in the scene graph
 * - Frames can be children of other Frames (e.g., TCP on gripper frame)
 * - Frames can be children of Links (sensor mounted on robot link)
 * - Frames can be parents of any Entity (robot on table frame)
 * - Frame poses can be updated dynamically (e.g., after calibration)
 *
 * The pose() inherited from Entity represents the transformation from
 * parent to this frame. Use pose_world() to get world pose.
 */
class Frame : public Entity {
public:
    /**
     * @brief Construct a frame
     * @param name Frame name (e.g., "tcp", "camera", "gripper_tip", "world")
     * @param pose Transformation from parent to this frame (default: identity)
     *
     * After construction, use set_parent() to attach to parent Entity.
     * Root frames (like "world") should not have a parent.
     */
    explicit Frame(const std::string& name, const math::SE3& pose = math::SE3::Identity());

    // Identification and scene graph inherited from Entity:
    // - name(), set_name()
    // - parent(), set_parent(Entity*)
    // - pose(), set_pose(), pose_world()
    // - type() returns Type::FRAME

    // ========================================================================
    // Convenience Methods (delegate to Entity)
    // ========================================================================

    /**
     * @brief Update pose by composing with additional transform
     * @param delta_transform Additional transformation to apply
     *
     * New pose = current_pose * delta_transform
     * Useful for incremental calibration updates.
     */
    void update_pose(const math::SE3& delta_transform) {
        set_pose(pose() * delta_transform);
    }

    /**
     * @brief Check if this is a root frame (no parent)
     */
    bool is_root() const { return parent() == nullptr; }
};

} // namespace model
} // namespace robospace
