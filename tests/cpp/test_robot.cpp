#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/robot.hpp>
#include <robospace/math/SO3.hpp>
#include <cmath>

using namespace robospace::model;
using namespace robospace::math;

// Helper: Create a simple 2-DOF robot
Robot create_2dof_robot() {
    Robot robot("test_robot");

    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));
    robot.add_link(Link("link2"));

    DHParams dh1(0, 1.0, 0, 0, DHConvention::STANDARD);
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);
    robot.add_joint(joint1);

    DHParams dh2(0, 0.5, 0, 0, DHConvention::STANDARD);
    Joint joint2("joint2", JointType::REVOLUTE, 1, 2);
    joint2.set_dh_params(dh2);
    robot.add_joint(joint2);

    return robot;
}

TEST_CASE("Robot: Construction", "[robot]") {
    Robot robot("my_robot");

    REQUIRE(robot.name() == "my_robot");
    REQUIRE(robot.type() == Entity::Type::ROBOT);
    REQUIRE(robot.num_links() == 0);
    REQUIRE(robot.num_joints() == 0);
    REQUIRE(robot.num_tools() == 0);
    REQUIRE(robot.dof() == 0);
    REQUIRE_FALSE(robot.is_valid());
    REQUIRE_FALSE(robot.has_active_tool());
}

TEST_CASE("Robot: Add links", "[robot]") {
    Robot robot("test_robot");

    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));

    REQUIRE(robot.num_links() == 2);
    REQUIRE(robot.link(0).name() == "base");
    REQUIRE(robot.link(1).name() == "link1");
}

TEST_CASE("Robot: Add joints", "[robot]") {
    Robot robot("test_robot");

    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));

    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    robot.add_joint(joint1);

    REQUIRE(robot.num_joints() == 1);
    REQUIRE(robot.joint(0).name() == "joint1");
}

TEST_CASE("Robot: Add tools", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("tcp", SE3::Translation(Eigen::Vector3d(0, 0, 0.1)));
    int tool_id = robot.add_tool(tool1);

    REQUIRE(robot.num_tools() == 1);
    REQUIRE(tool_id == 0);
    REQUIRE(robot.tool(0).name() == "tcp");
    REQUIRE_THAT(robot.tool(0).tcp_pose().translation().z(), Catch::Matchers::WithinAbs(0.1, 1e-10));
}

TEST_CASE("Robot: Tool accessors by name", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("gripper", SE3::Translation(Eigen::Vector3d(0, 0, 0.15)));
    robot.add_tool(tool1);

    REQUIRE(robot.tool("gripper").name() == "gripper");
    REQUIRE_THAT(robot.tool("gripper").tcp_pose().translation().z(),
                 Catch::Matchers::WithinAbs(0.15, 1e-10));
}

TEST_CASE("Robot: Link accessors by ID", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.link(0).name() == "base");
    REQUIRE(robot.link(1).name() == "link1");
    REQUIRE(robot.link(2).name() == "link2");
}

TEST_CASE("Robot: Link accessors by name", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.link("base").name() == "base");
    REQUIRE(robot.link("link1").name() == "link1");
    REQUIRE(robot.link("link2").name() == "link2");
}

TEST_CASE("Robot: Joint accessors by ID", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.joint(0).name() == "joint1");
    REQUIRE(robot.joint(1).name() == "joint2");
}

TEST_CASE("Robot: Joint accessors by name", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.joint("joint1").name() == "joint1");
    REQUIRE(robot.joint("joint2").name() == "joint2");
}

TEST_CASE("Robot: DOF (degrees of freedom)", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.dof() == 2);
}

TEST_CASE("Robot: DOF (degrees of freedom) with fixed joint", "[robot]") {
    Robot robot("test_robot");

    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));
    robot.add_link(Link("link2"));

    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    robot.add_joint(joint1);

    Joint joint2("joint2", JointType::FIXED, 1, 2);
    robot.add_joint(joint2);

    REQUIRE(robot.dof() == 1);
}

TEST_CASE("Robot: joint_limits()", "[robot]") {
    Robot robot("test_robot");

    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));
    robot.add_link(Link("link2"));

    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_limits(-1.5, 1.5);
    robot.add_joint(joint1);

    Joint joint2("joint2", JointType::REVOLUTE, 1, 2);
    joint2.set_limits(-2.0, 2.0);
    robot.add_joint(joint2);

    auto [qmin, qmax] = robot.joint_limits();

    REQUIRE(qmin.size() == 2);
    REQUIRE(qmax.size() == 2);
    REQUIRE_THAT(qmin(0), Catch::Matchers::WithinAbs(-1.5, 1e-10));
    REQUIRE_THAT(qmax(0), Catch::Matchers::WithinAbs(1.5, 1e-10));
    REQUIRE_THAT(qmin(1), Catch::Matchers::WithinAbs(-2.0, 1e-10));
    REQUIRE_THAT(qmax(1), Catch::Matchers::WithinAbs(2.0, 1e-10));
}

TEST_CASE("Robot: Base frame", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.base_frame().pose().isApprox(SE3::Identity(), 1e-10));

    SE3 base_offset = SE3::Translation(Eigen::Vector3d(0, 0, -0.525));
    robot.set_base_pose(base_offset);

    REQUIRE(robot.base_frame().pose().isApprox(base_offset, 1e-10));
    REQUIRE_THAT(robot.base_frame().pose().translation().z(),
                 Catch::Matchers::WithinAbs(-0.525, 1e-10));
}

TEST_CASE("Robot: Frame parenting and link poses", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.base_frame().parent() == &robot);
    REQUIRE(robot.link(0).parent() == &robot.base_frame());
    REQUIRE(robot.flange_frame().parent() == &robot.link(robot.num_links() - 1));

    Tool tool("tcp", SE3::Translation(Eigen::Vector3d(0, 0, 0.1)));
    int tool_id = robot.add_tool(tool);
    REQUIRE(robot.tool(tool_id).parent() == &robot.flange_frame());

    SE3 base_offset = SE3::Translation(Eigen::Vector3d(0.1, 0.2, 0.3));
    robot.set_base_pose(base_offset);

    Eigen::VectorXd q(2);
    q << 0.3, -0.4;
    robot.set_config(q);

    SE3 link_pose = robot.link(2).pose_world();
    SE3 fk_pose = robot.fk(q);
    REQUIRE(link_pose.isApprox(fk_pose));
}

TEST_CASE("Robot: Copy preserves entity graph", "[robot]") {
    Robot robot = create_2dof_robot();

    Eigen::VectorXd q(2);
    q << 0.3, -0.4;
    robot.set_config(q);

    Robot copy = robot;

    const auto& base_children = copy.base_frame().children();
    REQUIRE(base_children.size() == 1);
    REQUIRE(base_children[0] == &copy.link(0));
    REQUIRE(copy.link(0).parent() == &copy.base_frame());
    REQUIRE(copy.flange_frame().parent() == &copy.link(copy.num_links() - 1));
    REQUIRE(copy.fk(q).isApprox(robot.fk(q)));
}

TEST_CASE("Robot: Active tool management", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("tcp1", SE3::Translation(Eigen::Vector3d(0, 0, 0.1)));
    Tool tool2("tcp2", SE3::Translation(Eigen::Vector3d(0, 0, 0.2)));
    robot.add_tool(tool1);
    robot.add_tool(tool2);

    REQUIRE_FALSE(robot.has_active_tool());

    robot.set_active_tool(0);
    REQUIRE(robot.has_active_tool());
    REQUIRE(robot.active_tool().name() == "tcp1");

    robot.set_active_tool("tcp2");
    REQUIRE(robot.active_tool().name() == "tcp2");
}

TEST_CASE("Robot: Validation", "[robot]") {
    Robot robot("test_robot");

    REQUIRE_FALSE(robot.is_valid());

    robot.add_link(Link("base"));
    REQUIRE(robot.is_valid());

    robot.add_link(Link("link1"));
    REQUIRE_FALSE(robot.is_valid());

    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    robot.add_joint(joint1);
    REQUIRE(robot.is_valid());
}

TEST_CASE("Robot: Error - duplicate link name", "[robot]") {
    Robot robot("test_robot");

    robot.add_link(Link("link1"));
    robot.add_link(Link("link1"));

    REQUIRE(robot.num_links() == 2);
}

TEST_CASE("Robot: Error - duplicate tool name", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("tcp", SE3::Identity());
    robot.add_tool(tool1);

    Tool tool2("tcp", SE3::Identity());
    REQUIRE_THROWS_AS(robot.add_tool(tool2), std::runtime_error);
}

TEST_CASE("Robot: Error - link not found by name", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE_THROWS_AS(robot.link("nonexistent"), std::runtime_error);
}

TEST_CASE("Robot: Error - joint not found by name", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE_THROWS_AS(robot.joint("nonexistent"), std::runtime_error);
}

TEST_CASE("Robot: Error - tool not found by name", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE_THROWS_AS(robot.tool("nonexistent"), std::runtime_error);
}

TEST_CASE("Robot: Error - invalid link ID", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE_THROWS_AS(robot.link(99), std::out_of_range);
}

TEST_CASE("Robot: Error - invalid joint ID", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE_THROWS_AS(robot.joint(99), std::out_of_range);
}

TEST_CASE("Robot: Error - invalid tool ID", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE_THROWS_AS(robot.tool(99), std::out_of_range);
}

TEST_CASE("Robot: Error - set active tool with invalid ID", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("tcp", SE3::Identity());
    robot.add_tool(tool1);

    REQUIRE_THROWS_AS(robot.set_active_tool(99), std::out_of_range);
}

TEST_CASE("Robot: Error - get active tool when not set", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE_THROWS_AS(robot.active_tool(), std::runtime_error);
}

TEST_CASE("Robot: config() and set_config() API", "[robot]") {
    Robot robot = create_2dof_robot();

    Eigen::VectorXd q(2);
    q << 0.5, -0.3;
    robot.set_config(q);

    const Eigen::VectorXd& q_retrieved = robot.config();
    REQUIRE(q_retrieved.size() == 2);
    REQUIRE_THAT(q_retrieved(0), Catch::Matchers::WithinAbs(0.5, 1e-10));
    REQUIRE_THAT(q_retrieved(1), Catch::Matchers::WithinAbs(-0.3, 1e-10));
}

TEST_CASE("Robot: home position API", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE_FALSE(robot.has_home());

    Eigen::VectorXd home(2);
    home << 0.1, -0.2;
    robot.set_home(home);

    REQUIRE(robot.has_home());
    const Eigen::VectorXd& home_retrieved = robot.home();
    REQUIRE(home_retrieved.size() == 2);
    REQUIRE_THAT(home_retrieved(0), Catch::Matchers::WithinAbs(0.1, 1e-10));
    REQUIRE_THAT(home_retrieved(1), Catch::Matchers::WithinAbs(-0.2, 1e-10));
}

TEST_CASE("Robot: set_home with wrong size throws", "[robot]") {
    Robot robot = create_2dof_robot();

    Eigen::VectorXd wrong_size(3);
    wrong_size << 0.0, 0.0, 0.0;

    REQUIRE_THROWS_AS(robot.set_home(wrong_size), std::invalid_argument);
}
