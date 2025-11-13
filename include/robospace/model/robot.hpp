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
    // Construction
    explicit Robot(const std::string& name, Entity* parent = nullptr);
    static Robot from_urdf(const std::string& urdf_path);
    static Robot from_urdf_string(const std::string& urdf_string);

    void add_link(const Link& link);
    void add_joint(const Joint& joint);

    // Model properties
    int dof() const;
    int num_links() const { return tree_.num_links(); }
    int num_joints() const { return tree_.num_joints(); }
    bool is_valid() const { return tree_.is_valid(); }
    std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_limits() const;

    // Joint space configuration
    const Eigen::VectorXd& joints() const { return tree_.configuration(); }
    void set_joints(const Eigen::VectorXd& q) { tree_.set_configuration(q); }

    const Eigen::VectorXd& home() const { return home_position_; }
    void set_home(const Eigen::VectorXd& q);
    bool has_home() const { return home_position_.size() > 0; }

    // Forward kinematics (stateless - explicit q)
    math::SE3 fk(const Eigen::VectorXd& q) const;
    math::SE3 fk(const Eigen::VectorXd& q, const std::string& link_name) const;
    math::SE3 fk(const Eigen::VectorXd& q, int link_id) const;
    std::vector<math::SE3> fk_all(const Eigen::VectorXd& q) const;

    // Forward kinematics (stateful - uses stored q)
    math::SE3 fk() const;
    math::SE3 fk(const std::string& link_name) const;
    math::SE3 fk(int link_id) const;
    std::vector<math::SE3> fk_all() const;

    // Differential kinematics
    Eigen::MatrixXd jacob0(const Eigen::VectorXd& q) const;
    Eigen::MatrixXd jacobe(const Eigen::VectorXd& q) const;
    Eigen::MatrixXd jacob0() const;
    Eigen::MatrixXd jacobe() const;

    // Base frame
    void set_base(const math::SE3& base);
    const math::SE3& base() const { return base_frame_; }

    // Tool management
    int add_tool(const Tool& tool);
    int num_tools() const { return static_cast<int>(tools_.size()); }
    const Tool& tool(int id) const;
    const Tool& tool(const std::string& name) const;

    void set_active_tool(int id);
    void set_active_tool(const std::string& name);
    bool has_active_tool() const { return active_tool_id_ >= 0; }
    const Tool& active_tool() const;

    // Links and joints access
    const Link& link(int id) const;
    const Link& link(const std::string& name) const;
    bool has_link(const std::string& name) const;

    const Joint& joint(int id) const;
    const Joint& joint(const std::string& name) const;
    bool has_joint(const std::string& name) const;

    // Entity interface
    math::SE3 pose() const override;

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
