#include <robospace/model/frame.hpp>

namespace robospace {
namespace model {

Frame::Frame(const std::string& name, int parent_link_id, const math::SE3& offset)
    : name_(name), parent_link_id_(parent_link_id), offset_(offset) {
}

Frame::Frame(const std::string& name, int parent_link_id)
    : name_(name), parent_link_id_(parent_link_id), offset_(math::SE3::Identity()) {
}

} // namespace model
} // namespace robospace
