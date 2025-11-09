#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/kinematic_tree.hpp>
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
// Configuration
// ============================================================================

TEST_CASE("KinematicTree: Set configuration", "[model][kinematics]") {
    KinematicTree tree;

    Link base("base");
    Link link1("link1");
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);

    Eigen::VectorXd q(1);
    q << M_PI / 4;

    tree.set_configuration(q);
    REQUIRE(tree.configuration().size() == 1);
    REQUIRE_THAT(tree.configuration()(0), Catch::Matchers::WithinAbs(M_PI/4, 1e-10));
}

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

    REQUIRE_THROWS_AS(tree.set_configuration(q_wrong), std::invalid_argument);
}

// ============================================================================
// Forward Kinematics - Simple Cases
// ============================================================================

TEST_CASE("KinematicTree: FK base link at identity", "[model][kinematics][fk]") {
    KinematicTree tree;
    Link base("base");
    tree.add_link(base);

    Eigen::VectorXd q(0);  // No joints
    tree.set_configuration(q);
    tree.compute_forward_kinematics();

    SE3 T_base = tree.link_pose(0);
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

    // Modified DH: a (link length along X)
    DHParams dh1(0, 1.0, 0, 0);  // Link1: length 1.0
    DHParams dh2(0, 0.5, 0, 0);  // Link2: length 0.5

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

    tree.set_configuration(q);
    tree.compute_forward_kinematics();

    // Link2 should be at (1.5, 0, 0)
    SE3 T_link2 = tree.link_pose(2);
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

    DHParams dh1(0, 1.0, 0, 0);
    DHParams dh2(0, 0.5, 0, 0);

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

    tree.set_configuration(q);
    tree.compute_forward_kinematics();

    // Link1 at (1.0, 0, 0)
    SE3 T_link1 = tree.link_pose(1);
    Eigen::Vector3d pos1 = T_link1.translation();
    REQUIRE_THAT(pos1(0), Catch::Matchers::WithinAbs(1.0, 1e-10));

    // Link2 at (1.0, 0.5, 0) due to 90° bend
    SE3 T_link2 = tree.link_pose(2);
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

    DHParams dh1(0, 1.0, 0, 0);
    DHParams dh2(0, 1.0, 0, 0);  // Both links length 1.0

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

    tree.set_configuration(q);
    tree.compute_forward_kinematics();

    SE3 T_link2 = tree.link_pose(2);
    Eigen::Vector3d pos = T_link2.translation();

    // With both at 45°, total angle = 90°
    // X = cos(45°)*1 + cos(90°)*1 = 0.707 + 0 = 0.707
    // Y = sin(45°)*1 + sin(90°)*1 = 0.707 + 1 = 1.707
    double sqrt2_2 = std::sqrt(2.0) / 2.0;
    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(sqrt2_2, 1e-4));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(sqrt2_2 + 1.0, 1e-4));
}

// ============================================================================
// Error Handling
// ============================================================================

TEST_CASE("KinematicTree: FK on invalid tree throws", "[model][kinematics][error]") {
    KinematicTree tree;

    Link base("base");
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);

    tree.add_link(base);
    tree.add_joint(joint1);
    // Missing link!

    Eigen::VectorXd q(1);
    q << 0;
    tree.set_configuration(q);

    REQUIRE_THROWS_AS(tree.compute_forward_kinematics(), std::runtime_error);
}

TEST_CASE("KinematicTree: FK before set_configuration throws", "[model][kinematics][error]") {
    KinematicTree tree;

    Link base("base");
    Link link1("link1");
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);

    // Don't set configuration!
    REQUIRE_THROWS_AS(tree.compute_forward_kinematics(), std::runtime_error);
}

TEST_CASE("KinematicTree: link_pose before FK throws", "[model][kinematics][error]") {
    KinematicTree tree;

    Link base("base");
    Link link1("link1");
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);

    tree.add_link(base);
    tree.add_joint(joint1);
    tree.add_link(link1);

    Eigen::VectorXd q(1);
    q << 0;
    tree.set_configuration(q);

    // Call link_pose before compute_forward_kinematics
    REQUIRE_THROWS_AS(tree.link_pose(1), std::runtime_error);
}
