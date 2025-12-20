#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/robot.hpp>
#include <robospace/model/urdf_parser.hpp>
#include <cmath>

using namespace robospace::model;
using namespace robospace::math;

TEST_CASE("URDF Parser: Load simple 2R robot", "[urdf][parser]") {
    Robot robot = Robot::from_urdf("test_data/simple_2r.urdf");

    SECTION("Robot name") {
        REQUIRE(robot.name() == "simple_2r");
    }

    SECTION("Number of links") {
        REQUIRE(robot.num_links() == 3);  // base, link1, link2
    }

    SECTION("Number of joints") {
        REQUIRE(robot.num_joints() == 2);  // joint1, joint2
    }

    SECTION("DOF (positions)") {
        REQUIRE(robot.dof() == 2);  // 2 revolute joints
    }

    SECTION("Link names") {
        REQUIRE(robot.has_link("base_link"));
        REQUIRE(robot.has_link("link1"));
        REQUIRE(robot.has_link("link2"));
    }

    SECTION("Joint names") {
        REQUIRE(robot.has_joint("joint1"));
        REQUIRE(robot.has_joint("joint2"));
    }
}

TEST_CASE("URDF Parser: Link properties", "[urdf][parser]") {
    Robot robot = Robot::from_urdf("test_data/simple_2r.urdf");

    SECTION("Link mass") {
        const Link& base = robot.link("base_link");
        REQUIRE_THAT(base.mass(), Catch::Matchers::WithinRel(1.0, 0.001));

        const Link& link1 = robot.link("link1");
        REQUIRE_THAT(link1.mass(), Catch::Matchers::WithinRel(2.0, 0.001));

        const Link& link2 = robot.link("link2");
        REQUIRE_THAT(link2.mass(), Catch::Matchers::WithinRel(1.5, 0.001));
    }

    SECTION("Link center of mass") {
        const Link& link1 = robot.link("link1");
        Eigen::Vector3d com = link1.com();
        REQUIRE_THAT(com(0), Catch::Matchers::WithinAbs(0.5, 1e-6));
        REQUIRE_THAT(com(1), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(com(2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    }

    SECTION("Link inertia tensor") {
        const Link& base = robot.link("base_link");
        Eigen::Matrix3d I = base.inertia();
        REQUIRE_THAT(I(0, 0), Catch::Matchers::WithinRel(0.1, 0.001));
        REQUIRE_THAT(I(1, 1), Catch::Matchers::WithinRel(0.1, 0.001));
        REQUIRE_THAT(I(2, 2), Catch::Matchers::WithinRel(0.1, 0.001));
        REQUIRE_THAT(I(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("URDF Parser: Joint properties", "[urdf][parser]") {
    Robot robot = Robot::from_urdf("test_data/simple_2r.urdf");

    SECTION("Joint types") {
        const Joint& joint1 = robot.joint("joint1");
        REQUIRE(joint1.type() == JointType::REVOLUTE);

        const Joint& joint2 = robot.joint("joint2");
        REQUIRE(joint2.type() == JointType::REVOLUTE);
    }

    SECTION("Joint parent-child") {
        const Joint& joint1 = robot.joint("joint1");
        REQUIRE(joint1.parent_link_id() == 0);  // base_link
        REQUIRE(joint1.child_link_id() == 1);   // link1

        const Joint& joint2 = robot.joint("joint2");
        REQUIRE(joint2.parent_link_id() == 1);  // link1
        REQUIRE(joint2.child_link_id() == 2);   // link2
    }

    SECTION("Joint axes") {
        const Joint& joint1 = robot.joint("joint1");
        Eigen::Vector3d axis1 = joint1.axis();
        REQUIRE_THAT(axis1(0), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(axis1(1), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(axis1(2), Catch::Matchers::WithinAbs(1.0, 1e-6));
    }

    SECTION("Joint limits") {
        const Joint& joint1 = robot.joint("joint1");
        REQUIRE_THAT(joint1.lower_limit(), Catch::Matchers::WithinRel(-M_PI, 0.001));
        REQUIRE_THAT(joint1.upper_limit(), Catch::Matchers::WithinRel(M_PI, 0.001));
        REQUIRE_THAT(joint1.velocity_limit(), Catch::Matchers::WithinRel(2.0, 0.001));
        REQUIRE_THAT(joint1.effort_limit(), Catch::Matchers::WithinRel(100.0, 0.001));
    }

    SECTION("Joint origins") {
        const Joint& joint2 = robot.joint("joint2");
        SE3 origin = joint2.origin();
        Eigen::Vector3d translation = origin.translation();

        // Joint2 origin is (1.0, 0, 0) from link1
        REQUIRE_THAT(translation(0), Catch::Matchers::WithinAbs(1.0, 1e-6));
        REQUIRE_THAT(translation(1), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(translation(2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("URDF Parser: Load UR5 robot", "[urdf][parser][ur5]") {
    Robot robot = Robot::from_urdf("test_data/ur5_simplified.urdf");

    SECTION("Robot name") {
        REQUIRE(robot.name() == "ur5");
    }

    SECTION("Number of links") {
        REQUIRE(robot.num_links() == 7);  // base + 6 links
    }

    SECTION("Number of joints") {
        REQUIRE(robot.num_joints() == 6);  // 6-DOF
    }

    SECTION("DOF (positions)") {
        REQUIRE(robot.dof() == 6);  // 6 revolute joints
    }

    SECTION("All links present") {
        REQUIRE(robot.has_link("base_link"));
        REQUIRE(robot.has_link("shoulder_link"));
        REQUIRE(robot.has_link("upper_arm_link"));
        REQUIRE(robot.has_link("forearm_link"));
        REQUIRE(robot.has_link("wrist_1_link"));
        REQUIRE(robot.has_link("wrist_2_link"));
        REQUIRE(robot.has_link("wrist_3_link"));
    }

    SECTION("All joints present") {
        REQUIRE(robot.has_joint("shoulder_pan_joint"));
        REQUIRE(robot.has_joint("shoulder_lift_joint"));
        REQUIRE(robot.has_joint("elbow_joint"));
        REQUIRE(robot.has_joint("wrist_1_joint"));
        REQUIRE(robot.has_joint("wrist_2_joint"));
        REQUIRE(robot.has_joint("wrist_3_joint"));
    }

    SECTION("Joint limits") {
        const Joint& elbow = robot.joint("elbow_joint");
        REQUIRE_THAT(elbow.lower_limit(), Catch::Matchers::WithinRel(-M_PI, 0.001));
        REQUIRE_THAT(elbow.upper_limit(), Catch::Matchers::WithinRel(M_PI, 0.001));
    }

    SECTION("Link masses") {
        const Link& shoulder = robot.link("shoulder_link");
        REQUIRE_THAT(shoulder.mass(), Catch::Matchers::WithinRel(3.7, 0.001));

        const Link& upper_arm = robot.link("upper_arm_link");
        REQUIRE_THAT(upper_arm.mass(), Catch::Matchers::WithinRel(8.393, 0.001));
    }
}

TEST_CASE("URDF Parser: from_urdf_string", "[urdf][parser]") {
    std::string urdf = R"(
<?xml version="1.0"?>
<robot name="minimal">
  <link name="base"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
</robot>
    )";

    Robot robot = Robot::from_urdf_string(urdf);

    REQUIRE(robot.name() == "minimal");
    REQUIRE(robot.num_links() == 2);
    REQUIRE(robot.num_joints() == 1);
    REQUIRE(robot.has_link("base"));
    REQUIRE(robot.has_link("link1"));
    REQUIRE(robot.has_joint("joint1"));
}

TEST_CASE("URDF Parser: Out-of-order elements are reordered", "[urdf][parser][ordering]") {
    Robot robot = Robot::from_urdf("test_data/simple_2r_swapped.urdf");

    REQUIRE(robot.num_links() == 3);
    REQUIRE(robot.num_joints() == 2);

    // Links should be ordered as a serial chain: base -> link1 -> link2
    REQUIRE(robot.link(0).name() == "base_link");
    REQUIRE(robot.link(1).name() == "link1");
    REQUIRE(robot.link(2).name() == "link2");

    // Joints should follow the same order
    REQUIRE(robot.joint(0).name() == "joint1");
    REQUIRE(robot.joint(1).name() == "joint2");
}

TEST_CASE("URDF Parser: Fixed joints map configs correctly", "[urdf][parser][fixed]") {
    Robot robot = Robot::from_urdf("test_data/simple_2r_fixed.urdf");

    REQUIRE(robot.num_joints() == 3);
    REQUIRE(robot.dof() == 2);
    REQUIRE(robot.joint(0).is_fixed());

    Eigen::VectorXd q_dof(2);
    q_dof << 0.2, -0.5;

    Eigen::VectorXd q_full = robot.expand_config(q_dof);
    REQUIRE(q_full.size() == robot.num_joints());

    Eigen::VectorXd q_roundtrip = robot.compress_config(q_full);
    REQUIRE(q_roundtrip.isApprox(q_dof));

    SE3 T_dof = robot.fk(q_dof);
    SE3 T_full = robot.fk(q_full);
    REQUIRE(T_dof.isApprox(T_full));
}

TEST_CASE("URDF Parser: Error handling", "[urdf][parser][error]") {
    SECTION("File not found") {
        REQUIRE_THROWS_AS(Robot::from_urdf("nonexistent.urdf"), std::runtime_error);
    }

    SECTION("Invalid XML") {
        std::string invalid_xml = "<robot><broken";
        REQUIRE_THROWS_AS(Robot::from_urdf_string(invalid_xml), std::runtime_error);
    }

    SECTION("Missing robot element") {
        std::string no_robot = R"(<?xml version="1.0"?><link name="test"/>)";
        REQUIRE_THROWS_AS(Robot::from_urdf_string(no_robot), std::runtime_error);
    }

    SECTION("Missing parent link") {
        std::string missing_parent = R"(
<?xml version="1.0"?>
<robot name="test">
  <link name="child"/>
  <joint name="j1" type="revolute">
    <parent link="nonexistent"/>
    <child link="child"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
        )";
        REQUIRE_THROWS_AS(Robot::from_urdf_string(missing_parent), std::runtime_error);
    }
}

TEST_CASE("URDF Parser: Origin with RPY", "[urdf][parser][transform]") {
    std::string urdf = R"(
<?xml version="1.0"?>
<robot name="rpy_test">
  <link name="base"/>
  <link name="rotated"/>
  <joint name="j1" type="fixed">
    <parent link="base"/>
    <child link="rotated"/>
    <origin xyz="1.0 2.0 3.0" rpy="1.57079632679 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
    )";

    Robot robot = Robot::from_urdf_string(urdf);
    const Joint& joint = robot.joint("j1");
    SE3 origin = joint.origin();

    SECTION("Translation") {
        Eigen::Vector3d t = origin.translation();
        REQUIRE_THAT(t(0), Catch::Matchers::WithinAbs(1.0, 1e-6));
        REQUIRE_THAT(t(1), Catch::Matchers::WithinAbs(2.0, 1e-6));
        REQUIRE_THAT(t(2), Catch::Matchers::WithinAbs(3.0, 1e-6));
    }

    SECTION("Rotation (90 deg around X)") {
        Eigen::Matrix3d R = origin.rotation();
        // R = Rx(pi/2): rotates Y->Z, Z->-Y
        // Check that [0,1,0] -> [0,0,1] (approximately)
        Eigen::Vector3d y_axis(0, 1, 0);
        Eigen::Vector3d rotated = R * y_axis;
        REQUIRE_THAT(rotated(0), Catch::Matchers::WithinAbs(0.0, 1e-4));
        REQUIRE_THAT(rotated(1), Catch::Matchers::WithinAbs(0.0, 1e-4));
        REQUIRE_THAT(rotated(2), Catch::Matchers::WithinAbs(1.0, 1e-4));
    }
}
