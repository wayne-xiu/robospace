#pragma once

#include <robospace/model/entity.hpp>
#include <robospace/model/kinematic_tree.hpp>
#include <robospace/model/link.hpp>
#include <robospace/model/joint.hpp>
#include <robospace/model/frame.hpp>
#include <robospace/model/tool.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace robospace {
namespace model {

class Robot : public Entity {
public:
    explicit Robot(const std::string& name);
    Robot(const std::string& name, Entity* parent);

    static Robot from_urdf(const std::string& urdf_path);
    static Robot from_urdf_string(const std::string& urdf_string);

    void add_link(const Link& link);
    void add_joint(const Joint& joint);

    int add_tool(const Tool& tool);
    const Tool& tool(int id) const;
    Tool& tool(int id);
    const Tool& tool(const std::string& name) const;
    Tool& tool(const std::string& name);
    int tool_id(const std::string& name) const;
    bool has_tool(const std::string& name) const;
    int num_tools() const { return static_cast<int>(tools_.size()); }

    math::SE3 pose() const override;
    void set_pose(const math::SE3& tcp_target) override;
    void move_base_by(const math::SE3& delta);

    const Link& link(int id) const;
    const Joint& joint(int id) const;
    Link& link(int id);
    Joint& joint(int id);

    const Link& link(const std::string& name) const;
    const Joint& joint(const std::string& name) const;
    Link& link(const std::string& name);
    Joint& joint(const std::string& name);

    int link_id(const std::string& name) const;
    int joint_id(const std::string& name) const;
    bool has_link(const std::string& name) const;
    bool has_joint(const std::string& name) const;

    int num_links() const { return tree_.num_links(); }
    int num_joints() const { return tree_.num_joints(); }
    int dof() const;

    void set_base_frame(const math::SE3& frame);
    const math::SE3& base_frame() const { return base_frame_; }

    int base_link_id() const { return 0; }
    const Link& base_link() const { return link(0); }
    Link& base_link() { return link(0); }

    int flange_link_id() const { return num_links() - 1; }
    const Link& flange_link() const { return link(flange_link_id()); }
    Link& flange_link() { return link(flange_link_id()); }

    bool has_active_tool() const { return active_tool_id_ >= 0; }
    int active_tool_id() const { return active_tool_id_; }
    const Tool& active_tool() const;
    Tool& active_tool();
    void set_active_tool(int tool_id);
    void set_active_tool(const std::string& tool_name);

    const KinematicTree& kinematic_tree() const { return tree_; }
    KinematicTree& kinematic_tree() { return tree_; }

    const Eigen::VectorXd& joints() const { return tree_.configuration(); }
    void set_joints(const Eigen::VectorXd& q) { tree_.set_configuration(q); }

    const Eigen::VectorXd& home() const { return home_position_; }
    void set_home(const Eigen::VectorXd& q);
    bool has_home() const { return home_position_.size() > 0; }

    math::SE3 compute_fk(const Eigen::VectorXd& q, const std::string& link_name) const;
    math::SE3 get_tcp_pose(const Eigen::VectorXd& q) const;
    std::vector<math::SE3> compute_all_link_poses(const Eigen::VectorXd& q) const;
    math::SE3 get_current_tcp_pose() const;

    bool is_valid() const { return tree_.is_valid(); }

private:
    KinematicTree tree_;
    math::SE3 base_frame_ = math::SE3::Identity();
    std::vector<Tool> tools_;
    int active_tool_id_ = -1;
    Eigen::VectorXd home_position_;
    std::unordered_map<std::string, int> link_name_to_id_;
    std::unordered_map<std::string, int> joint_name_to_id_;
    std::unordered_map<std::string, int> tool_name_to_id_;
};

} // namespace model
} // namespace robospace
