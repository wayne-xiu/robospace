#include <robospace/model/tool.hpp>

namespace robospace {
namespace model {

Tool::Tool(const std::string& name, const math::SE3& tcp_pose)
    : Frame(name, tcp_pose) {
}

} // namespace model
} // namespace robospace
