#include <robospace/model/entity.hpp>
#include <algorithm>
#include <utility>

namespace robospace {
namespace model {

Entity::Entity(const std::string& name, Type type, Entity* parent)
    : name_(name), type_(type), pose_(math::SE3::Identity()) {
    if (parent != nullptr) {
        set_parent(parent);
    }
}

Entity::Entity(const Entity& other)
    : name_(other.name_), type_(other.type_), pose_(other.pose_) {}

Entity& Entity::operator=(const Entity& other) {
    if (this == &other) {
        return *this;
    }

    if (parent_ != nullptr) {
        parent_->remove_child(this);
    }
    for (auto* child : children_) {
        child->parent_ = nullptr;
    }
    children_.clear();

    name_ = other.name_;
    type_ = other.type_;
    pose_ = other.pose_;
    parent_ = nullptr;

    return *this;
}

Entity::Entity(Entity&& other) noexcept
    : name_(std::move(other.name_)),
      type_(other.type_),
      pose_(other.pose_),
      parent_(other.parent_),
      children_(std::move(other.children_)) {
    if (parent_ != nullptr) {
        bool replaced = false;
        for (auto*& child : parent_->children_) {
            if (child == &other) {
                child = this;
                replaced = true;
                break;
            }
        }
        if (!replaced) {
            auto it = std::find(parent_->children_.begin(), parent_->children_.end(), this);
            if (it == parent_->children_.end()) {
                parent_->children_.push_back(this);
            }
        }
    }

    for (auto* child : children_) {
        if (child != nullptr) {
            child->parent_ = this;
        }
    }

    other.parent_ = nullptr;
    other.children_.clear();
}

Entity& Entity::operator=(Entity&& other) noexcept {
    if (this == &other) {
        return *this;
    }

    if (parent_ != nullptr) {
        parent_->remove_child(this);
    }
    for (auto* child : children_) {
        if (child != nullptr) {
            child->parent_ = nullptr;
        }
    }
    children_.clear();

    name_ = std::move(other.name_);
    type_ = other.type_;
    pose_ = other.pose_;
    parent_ = other.parent_;
    children_ = std::move(other.children_);

    if (parent_ != nullptr) {
        bool replaced = false;
        for (auto*& child : parent_->children_) {
            if (child == &other) {
                child = this;
                replaced = true;
                break;
            }
        }
        if (!replaced) {
            auto it = std::find(parent_->children_.begin(), parent_->children_.end(), this);
            if (it == parent_->children_.end()) {
                parent_->children_.push_back(this);
            }
        }
    }

    for (auto* child : children_) {
        if (child != nullptr) {
            child->parent_ = this;
        }
    }

    other.parent_ = nullptr;
    other.children_.clear();

    return *this;
}

Entity::~Entity() {
    if (parent_ != nullptr) {
        parent_->remove_child(this);
    }

    for (auto* child : children_) {
        child->parent_ = nullptr;
    }
    children_.clear();
}

void Entity::set_parent(Entity* new_parent) {
    if (parent_ != nullptr) {
        parent_->remove_child(this);
    }

    parent_ = new_parent;

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
