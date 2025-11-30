#include <robospace/model/dh_adapter.hpp>

namespace robospace {
namespace model {

math::SE3 DHAdapter::convert(const DHParams& dh, double q, bool is_prismatic) {
    // Delegate to existing DHParams implementation
    // Future: This will pre-compute at load time instead
    return dh.transform(q, is_prismatic);
}

math::SE3 DHAdapter::convert_standard(const DHParams& dh, double q, bool is_prismatic) {
    return dh.transform_standard(q, is_prismatic);
}

math::SE3 DHAdapter::convert_modified(const DHParams& dh, double q, bool is_prismatic) {
    return dh.transform_modified(q, is_prismatic);
}

} // namespace model
} // namespace robospace
