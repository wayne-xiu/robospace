#include <robospace/model/frame.hpp>

namespace robospace {
namespace model {

Frame::Frame(const std::string& name, const math::SE3& pose)
    : Entity(name, Type::FRAME) {
    set_pose(pose);
}

} // namespace model
} // namespace robospace
