#include <robospace/model/entity.hpp>
#include <algorithm>

namespace robospace {
namespace model {

Entity::Entity(const std::string& name, Type type)
    : name_(name), type_(type), pose_(math::SE3::Identity()) {
}

void Entity::set_parent(Entity* new_parent) {
    // Remove from old parent's children list
    if (parent_ != nullptr) {
        parent_->remove_child(this);
    }

    // Set new parent
    parent_ = new_parent;

    // Add to new parent's children list
    if (parent_ != nullptr) {
        parent_->add_child(this);
    }
}

int Entity::depth() const {
    int d = 0;
    const Entity* current = this;
    while (current->parent_ != nullptr) {
        d++;
        current = current->parent_;
    }
    return d;
}

math::SE3 Entity::pose_world() const {
    // Base case: root entity has world pose = its pose
    if (parent_ == nullptr) {
        return pose_;
    }

    // Recursive case: compose parent's world pose with our local pose
    // World → Parent → This
    return parent_->pose_world() * pose_;
}

void Entity::add_child(Entity* child) {
    // Avoid duplicates
    auto it = std::find(children_.begin(), children_.end(), child);
    if (it == children_.end()) {
        children_.push_back(child);
    }
}

void Entity::remove_child(Entity* child) {
    auto it = std::find(children_.begin(), children_.end(), child);
    if (it != children_.end()) {
        children_.erase(it);
    }
}

math::SE3 compute_transform(const Entity* source, const Entity* target) {
    // Special case: same entity
    if (source == target) {
        return math::SE3::Identity();
    }

    // Get world poses of both entities
    math::SE3 T_world_source = source->pose_world();
    math::SE3 T_world_target = target->pose_world();

    // Compute relative transform:
    // T_target_source = T_target_world * T_world_source
    //                 = (T_world_target)^-1 * T_world_source
    return T_world_target.inverse() * T_world_source;
}

} // namespace model
} // namespace robospace
