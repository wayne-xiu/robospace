#pragma once

#include <robospace/math/SE3.hpp>
#include <string>

namespace robospace {
namespace model {

/**
 * @brief Frame represents a named coordinate system attached to a link
 *
 * Frames are lightweight coordinate systems used to represent:
 * - Tool Center Points (TCP) for end-effectors
 * - Sensor mounting points (cameras, force sensors, etc.)
 * - Calibration target locations (laser tracker SMR mounts)
 * - Workpiece coordinate systems
 * - Task frames for manipulation
 *
 * A frame is defined by:
 * - Name: Unique identifier (e.g., "tcp", "camera", "target_1")
 * - Parent link: The link this frame is attached to
 * - Offset: SE3 transformation from link origin to frame
 *
 * Design philosophy (from Drake/ROS):
 * - One link can have many frames
 * - Frames are separate from links (not a property of Link class)
 * - Frames can be added dynamically (e.g., for calibration)
 * - Frame offsets can be updated (e.g., after hand-eye calibration)
 */
class Frame {
public:
    /**
     * @brief Construct a frame
     * @param name Frame name (e.g., "tcp", "camera", "gripper_tip")
     * @param parent_link_id Index of the parent link this frame is attached to
     * @param offset SE3 transformation from link origin to frame
     */
    Frame(const std::string& name, int parent_link_id, const math::SE3& offset);

    /**
     * @brief Construct a frame with identity offset
     * @param name Frame name
     * @param parent_link_id Index of parent link
     */
    Frame(const std::string& name, int parent_link_id);

    // ========================================================================
    // Identification
    // ========================================================================

    const std::string& name() const { return name_; }
    void set_name(const std::string& name) { name_ = name; }

    // ========================================================================
    // Parent Relationship
    // ========================================================================

    /**
     * @brief Get parent link ID
     * @return Index of parent link (-1 if world/root frame)
     */
    int parent_link_id() const { return parent_link_id_; }

    /**
     * @brief Set parent link
     * @param parent_link_id New parent link index
     *
     * Note: This changes which link the frame is attached to.
     * The offset remains unchanged (still relative to new parent).
     */
    void set_parent_link_id(int parent_link_id) { parent_link_id_ = parent_link_id; }

    // ========================================================================
    // Offset (Transform from parent link to frame)
    // ========================================================================

    /**
     * @brief Get offset transformation
     * @return SE3 transform from parent link origin to this frame
     */
    const math::SE3& offset() const { return offset_; }

    /**
     * @brief Set offset transformation
     * @param offset New SE3 transform from parent link to frame
     *
     * Use cases:
     * - Initial frame definition (TCP offset from flange)
     * - Calibration updates (hand-eye calibration, laser tracker results)
     * - Dynamic tool changes
     */
    void set_offset(const math::SE3& offset) { offset_ = offset; }

    /**
     * @brief Update offset by composing with additional transform
     * @param delta_transform Additional transformation to apply
     *
     * New offset = current_offset * delta_transform
     * Useful for incremental calibration updates.
     */
    void update_offset(const math::SE3& delta_transform) {
        offset_ = offset_ * delta_transform;
    }

    // ========================================================================
    // Convenience queries
    // ========================================================================

    /**
     * @brief Check if this is a root/world frame (no parent link)
     */
    bool is_root() const { return parent_link_id_ < 0; }

    /**
     * @brief Get translation component of offset
     */
    Eigen::Vector3d translation() const { return offset_.translation(); }

    /**
     * @brief Get rotation component of offset
     */
    Eigen::Matrix3d rotation() const { return offset_.rotation(); }

private:
    std::string name_;          ///< Frame name (unique identifier)
    int parent_link_id_;        ///< Parent link index (-1 for world/root)
    math::SE3 offset_;          ///< Transform from parent link to frame
};

} // namespace model
} // namespace robospace
