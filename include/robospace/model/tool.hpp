#pragma once

#include <robospace/model/frame.hpp>
#include <robospace/math/SE3.hpp>
#include <string>

namespace robospace {
namespace model {

/**
 * @brief Robot end-effector tool
 *
 * Represents a tool attached to the robot flange.
 * Contains the TCP (Tool Center Point) pose relative to the flange.
 */
class Tool : public Frame {
public:
    /**
     * @brief Construct a tool
     * @param name Tool name
     * @param tcp_pose TCP pose relative to flange frame
     */
    Tool(const std::string& name, const math::SE3& tcp_pose = math::SE3::Identity());

    // TCP pose relative to flange
    math::SE3 tcp_pose() const { return pose(); }
    void set_tcp_pose(const math::SE3& tcp) { set_pose(tcp); }
};

} // namespace model
} // namespace robospace
