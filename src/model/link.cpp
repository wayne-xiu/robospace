#include <robospace/model/link.hpp>

namespace robospace {
namespace model {

Link::Link(const std::string& name)
    : Entity(name, Type::LINK) {
    // Default initialization is handled by member initializers
}

} // namespace model
} // namespace robospace
