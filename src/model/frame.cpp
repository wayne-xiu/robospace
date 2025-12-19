#include <robospace/model/frame.hpp>

namespace robospace {
namespace model {

Frame::Frame(const std::string& name, const math::SE3& pose, Entity* parent)
    : Entity(name, Type::FRAME, parent) {
    set_pose(pose);
}

} // namespace model
} // namespace robospace
