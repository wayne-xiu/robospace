#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/kinematic_tree.hpp>
#include <robospace/model/dh_params.hpp>
#include <robospace/math/SO3.hpp>

using namespace robospace::model;
using namespace robospace::math;

// ============================================================================
// Construction and Validation
// ============================================================================

TEST_CASE("KinematicTree: Empty construction", "[model][kinematics]") {
    KinematicTree tree;
    REQUIRE(tree.num_joints() == 0);
    REQUIRE(tree.num_links() == 0);
    REQUIRE_FALSE(tree.is_valid());  // 0 links ≠ 0+1 joints
}

TEST_CASE("KinematicTree: Add base link", "[model][kinematics]") {
    KinematicTree tree;
    Link base("base");
    tree.add_link(base);

    REQUIRE(tree.num_links() == 1);
    REQUIRE(tree.num_joints() == 0);
    REQUIRE(tree.is_valid());  // 1 link, 0 joints → valid
    REQUIRE(tree.link(0).name() == "base");
}

TEST_CASE("KinematicTree: Build 2-link chain", "[model][kinematics]") {
    KinematicTree tree;

    Link base("base");
    Link link1("link1");

    // DH parameters for simple revolute joint
    DHParams dh1(0, 0, 0, 0);  // alpha=0, a=0, d=0, theta=0
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);

    REQUIRE(tree.num_links() == 2);
    REQUIRE(tree.num_joints() == 1);
    REQUIRE(tree.is_valid());
}

TEST_CASE("KinematicTree: Invalid tree (too many joints)", "[model][kinematics]") {
    KinematicTree tree;

    Link base("base");
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);

    tree.add_link(base);
    tree.add_joint(joint1);
    // Missing link1!

    REQUIRE_FALSE(tree.is_valid());  // 1 link, 1 joint → invalid
}

// ============================================================================
// Configuration Size Validation
// ============================================================================

TEST_CASE("KinematicTree: Configuration size mismatch throws", "[model][kinematics]") {
    KinematicTree tree;

    Link base("base");
    Link link1("link1");
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);

    Eigen::VectorXd q_wrong(2);  // Need 1, providing 2
    q_wrong << 0, 0;

    REQUIRE_THROWS_AS(tree.compute_forward_kinematics(q_wrong), std::invalid_argument);
}

// ============================================================================
// Forward Kinematics - Simple Cases
// ============================================================================

TEST_CASE("KinematicTree: FK base link at identity", "[model][kinematics][fk]") {
    KinematicTree tree;
    Link base("base");
    tree.add_link(base);

    Eigen::VectorXd q(0);  // No joints
    SE3 T_base = tree.compute_link_pose(q, 0);
    REQUIRE(T_base.isApprox(SE3::Identity()));
}

TEST_CASE("KinematicTree: FK 2-link planar arm (zero config)", "[model][kinematics][fk]") {
    // Planar 2R arm:
    // - Link0 (base) at origin
    // - Joint1: revolute about Z, at origin
    // - Link1: extends along X by length L1 = 1.0
    // - Joint2: revolute about Z, at end of Link1
    // - Link2: extends along X by length L2 = 0.5

    KinematicTree tree;

    Link base("base");
    Link link1("link1");
    Link link2("link2");

    // Standard DH: a (link length along X after rotation)
    DHParams dh1(0, 1.0, 0, 0, DHConvention::STANDARD);  // Link1: length 1.0
    DHParams dh2(0, 0.5, 0, 0, DHConvention::STANDARD);  // Link2: length 0.5

    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);
    Joint joint2("joint2", JointType::REVOLUTE, 1, 2);
    joint2.set_dh_params(dh2);

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);
    tree.add_joint(joint2);
    tree.add_link(link2);

    // Zero configuration
    Eigen::VectorXd q(2);
    q << 0, 0;

    // Link2 should be at (1.5, 0, 0)
    SE3 T_link2 = tree.compute_link_pose(q, 2);
    Eigen::Vector3d pos = T_link2.translation();

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(1.5, 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(pos(2), Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("KinematicTree: FK 2-link planar arm (90° elbow)", "[model][kinematics][fk]") {
    // Same 2R arm, but with Joint2 at 90°
    KinematicTree tree;

    Link base("base");
    Link link1("link1");
    Link link2("link2");

    DHParams dh1(0, 1.0, 0, 0, DHConvention::STANDARD);
    DHParams dh2(0, 0.5, 0, 0, DHConvention::STANDARD);

    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);
    Joint joint2("joint2", JointType::REVOLUTE, 1, 2);
    joint2.set_dh_params(dh2);

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);
    tree.add_joint(joint2);
    tree.add_link(link2);

    // q = [0, 90°]
    Eigen::VectorXd q(2);
    q << 0, M_PI / 2;

    // Link1 at (1.0, 0, 0)
    SE3 T_link1 = tree.compute_link_pose(q, 1);
    Eigen::Vector3d pos1 = T_link1.translation();
    REQUIRE_THAT(pos1(0), Catch::Matchers::WithinAbs(1.0, 1e-10));

    // Link2 at (1.0, 0.5, 0) due to 90° bend
    SE3 T_link2 = tree.compute_link_pose(q, 2);
    Eigen::Vector3d pos2 = T_link2.translation();
    REQUIRE_THAT(pos2(0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(pos2(1), Catch::Matchers::WithinAbs(0.5, 1e-10));
    REQUIRE_THAT(pos2(2), Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("KinematicTree: FK 2-link planar arm (both joints 45°)", "[model][kinematics][fk]") {
    KinematicTree tree;

    Link base("base");
    Link link1("link1");
    Link link2("link2");

    DHParams dh1(0, 1.0, 0, 0, DHConvention::STANDARD);
    DHParams dh2(0, 1.0, 0, 0, DHConvention::STANDARD);  // Both links length 1.0

    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);
    Joint joint2("joint2", JointType::REVOLUTE, 1, 2);
    joint2.set_dh_params(dh2);

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);
    tree.add_joint(joint2);
    tree.add_link(link2);

    // Both joints at 45°
    Eigen::VectorXd q(2);
    q << M_PI / 4, M_PI / 4;

    SE3 T_link2 = tree.compute_link_pose(q, 2);
    Eigen::Vector3d pos = T_link2.translation();

    // With both at 45°, total angle = 90°
    // X = cos(45°)*1 + cos(90°)*1 = 0.707 + 0 = 0.707
    // Y = sin(45°)*1 + sin(90°)*1 = 0.707 + 1 = 1.707
    double sqrt2_2 = std::sqrt(2.0) / 2.0;
    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(sqrt2_2, 1e-4));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(sqrt2_2 + 1.0, 1e-4));
}

// ============================================================================
// Industrial Robots (axis direction + coupling)
// ============================================================================

TEST_CASE("KinematicTree: FK with axis direction", "[model][kinematics][industrial]") {
    // Simple 1-joint arm with inverted axis (like KUKA J1)
    KinematicTree tree;

    Link base("base");
    Link link1("link1");

    DHParams dh1(0, 1.0, 0, 0, DHConvention::STANDARD);
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);
    joint1.set_axis_direction(-1);  // Inverted axis

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);

    // Input q=45° → effective angle = -45° due to axis_direction=-1
    Eigen::VectorXd q(1);
    q << M_PI / 4;  // +45°

    SE3 T = tree.compute_link_pose(q, 1);

    // With axis_direction=-1, joint rotates by -45°
    // Link at (cos(-45°), sin(-45°), 0) * 1.0 = (0.707, -0.707, 0)
    double sqrt2_2 = std::sqrt(2.0) / 2.0;
    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(sqrt2_2, 1e-4));
    REQUIRE_THAT(T.translation()(1), Catch::Matchers::WithinAbs(-sqrt2_2, 1e-4));
}

TEST_CASE("KinematicTree: FK with J2-J3 coupling (Fanuc style)", "[model][kinematics][industrial]") {
    // 3-joint arm with J2-J3 coupling (coefficient -1)
    KinematicTree tree;

    Link base("base");
    Link link1("link1");
    Link link2("link2");
    Link link3("link3");

    DHParams dh1(0, 1.0, 0, 0, DHConvention::STANDARD);
    DHParams dh2(0, 0.5, 0, 0, DHConvention::STANDARD);
    DHParams dh3(0, 0.5, 0, 0, DHConvention::STANDARD);

    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);

    Joint joint2("joint2", JointType::REVOLUTE, 1, 2);
    joint2.set_dh_params(dh2);

    Joint joint3("joint3", JointType::REVOLUTE, 2, 3);
    joint3.set_dh_params(dh3);
    joint3.add_coupling(1, -1.0);  // J3 couples with J2 (index 1), coefficient -1

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);
    tree.add_joint(joint2);
    tree.add_link(link2);
    tree.add_joint(joint3);
    tree.add_link(link3);

    // Configuration: J1=0°, J2=30°, J3=45°
    Eigen::VectorXd q(3);
    q << 0, M_PI/6, M_PI/4;  // 0°, 30°, 45°

    // J3 effective = J3_input + coupling_coef * J2 = 45° + (-1)*30° = 15°
    // Total angle at Link3 = J2 + J3_effective = 30° + 15° = 45°

    SE3 T3 = tree.compute_link_pose(q, 3);

    // Link1: 1.0 along X → (1, 0, 0)
    // Link2: 0.5 at 30° → (1 + 0.5*cos(30°), 0.5*sin(30°), 0) = (1.433, 0.25, 0)
    // Link3: 0.5 at 45° → additional (0.5*cos(45°), 0.5*sin(45°), 0) = (0.354, 0.354, 0)
    // Total: (1.787, 0.604, 0)

    REQUIRE_THAT(T3.translation()(0), Catch::Matchers::WithinAbs(1.787, 0.01));
    REQUIRE_THAT(T3.translation()(1), Catch::Matchers::WithinAbs(0.604, 0.01));
    REQUIRE_THAT(T3.translation()(2), Catch::Matchers::WithinAbs(0.0, 1e-4));
}
