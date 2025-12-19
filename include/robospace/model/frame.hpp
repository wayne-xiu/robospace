#pragma once

#include <robospace/model/entity.hpp>
#include <robospace/math/SE3.hpp>
#include <string>

namespace robospace {
namespace model {

/**
 * @brief Named coordinate system in the scene graph
 *
 * Used for TCPs, sensors, calibration targets, workpiece coordinates, etc.
 * Inherits from Entity to support flexible parent-child relationships
 * (Frame→Frame, Link→Frame, Frame→Entity).
 */
class Frame : public Entity {
public:
    explicit Frame(const std::string& name,
                   const math::SE3& pose = math::SE3::Identity(),
                   Entity* parent = nullptr);

    // Inherits from Entity: name(), parent(), pose(), pose_world(), etc.

    void update_pose(const math::SE3& delta_transform) {
        set_pose(pose() * delta_transform);
    }

    // is_root() inherited from Entity
};

} // namespace model
} // namespace robospace
