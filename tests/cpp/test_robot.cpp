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

    // Add links
    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));
    robot.add_link(Link("link2"));

    // Add joints
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

TEST_CASE("Robot: Mutable tool accessors", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("tcp", SE3::Identity());
    robot.add_tool(tool1);

    // Modify tool through mutable accessor
    robot.tool(0).set_tcp_pose(SE3::Translation(Eigen::Vector3d(0, 0, 0.2)));

    REQUIRE_THAT(robot.tool(0).tcp_pose().translation().z(),
                 Catch::Matchers::WithinAbs(0.2, 1e-10));
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

TEST_CASE("Robot: Mutable link accessors", "[robot]") {
    Robot robot = create_2dof_robot();

    // Modify link through mutable accessor
    robot.link(1).set_mass(5.0);

    REQUIRE_THAT(robot.link(1).mass(), Catch::Matchers::WithinAbs(5.0, 1e-10));
}

TEST_CASE("Robot: Mutable joint accessors", "[robot]") {
    Robot robot = create_2dof_robot();

    // Modify joint through mutable accessor
    robot.joint(0).set_limits(-1.0, 2.0);

    REQUIRE(robot.joint(0).has_position_limits());
    REQUIRE_THAT(robot.joint(0).lower_limit(), Catch::Matchers::WithinAbs(-1.0, 1e-10));
    REQUIRE_THAT(robot.joint(0).upper_limit(), Catch::Matchers::WithinAbs(2.0, 1e-10));
}

TEST_CASE("Robot: Link name lookups", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.link_id("base") == 0);
    REQUIRE(robot.link_id("link1") == 1);
    REQUIRE(robot.link_id("link2") == 2);

    REQUIRE(robot.has_link("base"));
    REQUIRE(robot.has_link("link1"));
    REQUIRE_FALSE(robot.has_link("nonexistent"));
}

TEST_CASE("Robot: Joint name lookups", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.joint_id("joint1") == 0);
    REQUIRE(robot.joint_id("joint2") == 1);

    REQUIRE(robot.has_joint("joint1"));
    REQUIRE(robot.has_joint("joint2"));
    REQUIRE_FALSE(robot.has_joint("nonexistent"));
}

TEST_CASE("Robot: Tool name lookups", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("tcp", SE3::Identity());
    Tool tool2("gripper", SE3::Identity());
    robot.add_tool(tool1);
    robot.add_tool(tool2);

    REQUIRE(robot.tool_id("tcp") == 0);
    REQUIRE(robot.tool_id("gripper") == 1);

    REQUIRE(robot.has_tool("tcp"));
    REQUIRE(robot.has_tool("gripper"));
    REQUIRE_FALSE(robot.has_tool("nonexistent"));
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

TEST_CASE("Robot: Base link accessors", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.base_link_id() == 0);
    REQUIRE(robot.base_link().name() == "base");
}

TEST_CASE("Robot: Flange link accessors", "[robot]") {
    Robot robot = create_2dof_robot();

    REQUIRE(robot.flange_link_id() == 2);
    REQUIRE(robot.flange_link().name() == "link2");
}

TEST_CASE("Robot: Base frame", "[robot]") {
    Robot robot = create_2dof_robot();

    // Default base frame is identity
    REQUIRE(robot.base_frame().isApprox(SE3::Identity(), 1e-10));

    // Set base frame (e.g., Fanuc style: physical base below base_ref)
    SE3 base_offset = SE3::Translation(Eigen::Vector3d(0, 0, -0.525));
    robot.set_base_frame(base_offset);

    REQUIRE(robot.base_frame().isApprox(base_offset, 1e-10));
    REQUIRE_THAT(robot.base_frame().translation().z(),
                 Catch::Matchers::WithinAbs(-0.525, 1e-10));
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
    REQUIRE(robot.active_tool_id() == 0);
    REQUIRE(robot.active_tool().name() == "tcp1");

    robot.set_active_tool("tcp2");
    REQUIRE(robot.active_tool_id() == 1);
    REQUIRE(robot.active_tool().name() == "tcp2");
}

TEST_CASE("Robot: Active tool modification", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("tcp", SE3::Identity());
    robot.add_tool(tool1);
    robot.set_active_tool(0);

    // Modify active tool
    robot.active_tool().set_tcp_pose(SE3::Translation(Eigen::Vector3d(0, 0, 0.3)));

    REQUIRE_THAT(robot.tool(0).tcp_pose().translation().z(),
                 Catch::Matchers::WithinAbs(0.3, 1e-10));
}

TEST_CASE("Robot: Tool parent is flange", "[robot]") {
    Robot robot = create_2dof_robot();

    Tool tool1("tcp", SE3::Identity());
    robot.add_tool(tool1);

    // Tool parent should be the flange link
    REQUIRE(robot.tool(0).parent() == &robot.flange_link());
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
    // Note: Current implementation doesn't prevent duplicate names
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

TEST_CASE("Robot: Kinematic tree access", "[robot]") {
    Robot robot = create_2dof_robot();

    const KinematicTree& tree = robot.kinematic_tree();
    REQUIRE(tree.num_links() == 3);
    REQUIRE(tree.num_joints() == 2);
}

TEST_CASE("Robot: joints() and set_joints() API", "[robot]") {
    Robot robot = create_2dof_robot();

    Eigen::VectorXd q(2);
    q << 0.5, -0.3;
    robot.set_joints(q);

    const Eigen::VectorXd& q_retrieved = robot.joints();
    REQUIRE(q_retrieved.size() == 2);
    REQUIRE_THAT(q_retrieved(0), Catch::Matchers::WithinAbs(0.5, 1e-10));
    REQUIRE_THAT(q_retrieved(1), Catch::Matchers::WithinAbs(-0.3, 1e-10));
}
