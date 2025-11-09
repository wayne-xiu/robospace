#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/frame.hpp>
#include <robospace/model/link.hpp>
#include <robospace/math/SO3.hpp>

using namespace robospace::model;
using namespace robospace::math;

// ============================================================================
// Frame Construction Tests
// ============================================================================

TEST_CASE("Frame: Construction with pose", "[model][frame]") {
    SE3 pose = SE3::Translation(Eigen::Vector3d(0.1, 0.2, 0.3));
    Frame f("tcp", pose);

    REQUIRE(f.name() == "tcp");
    REQUIRE(f.type() == Entity::Type::FRAME);
    REQUIRE(f.pose().isApprox(pose));
    REQUIRE(f.is_root());  // No parent set yet
}

TEST_CASE("Frame: Construction with identity pose", "[model][frame]") {
    Frame f("camera");

    REQUIRE(f.name() == "camera");
    REQUIRE(f.type() == Entity::Type::FRAME);
    REQUIRE(f.pose().isApprox(SE3::Identity()));
}

TEST_CASE("Frame: Root frame (world)", "[model][frame]") {
    Frame f("world");

    REQUIRE(f.name() == "world");
    REQUIRE(f.is_root());
    REQUIRE(f.parent() == nullptr);
}

// ============================================================================
// Name and Parent Modification
// ============================================================================

TEST_CASE("Frame: Set name", "[model][frame]") {
    Frame f("temp");

    f.set_name("new_name");
    REQUIRE(f.name() == "new_name");
}

TEST_CASE("Frame: Set parent (Frame -> Link)", "[model][frame]") {
    Link link("link5");
    Frame f("sensor");

    REQUIRE(f.parent() == nullptr);
    REQUIRE(f.is_root());

    f.set_parent(&link);
    REQUIRE(f.parent() == &link);
    REQUIRE_FALSE(f.is_root());
}

TEST_CASE("Frame: Set parent (Frame -> Frame)", "[model][frame]") {
    Frame parent_frame("gripper");
    Frame child_frame("tcp");

    child_frame.set_parent(&parent_frame);
    REQUIRE(child_frame.parent() == &parent_frame);
    REQUIRE_FALSE(child_frame.is_root());
    REQUIRE(parent_frame.is_root());
}

// ============================================================================
// Pose Manipulation
// ============================================================================

TEST_CASE("Frame: Set pose", "[model][frame]") {
    Frame f("tcp");

    // Initially identity
    REQUIRE(f.pose().isApprox(SE3::Identity()));

    // Set new pose
    SE3 new_pose = SE3::Translation(Eigen::Vector3d(0.05, 0, 0.1));
    f.set_pose(new_pose);

    REQUIRE(f.pose().isApprox(new_pose));
    REQUIRE_THAT(f.pose().translation()(0), Catch::Matchers::WithinAbs(0.05, 1e-10));
    REQUIRE_THAT(f.pose().translation()(2), Catch::Matchers::WithinAbs(0.1, 1e-10));
}

TEST_CASE("Frame: Update pose (incremental)", "[model][frame]") {
    // Initial pose: translate by (0.1, 0, 0)
    SE3 initial = SE3::Translation(Eigen::Vector3d(0.1, 0, 0));
    Frame f("tcp", initial);

    // Add delta: translate by (0.02, 0, 0)
    SE3 delta = SE3::Translation(Eigen::Vector3d(0.02, 0, 0));
    f.update_pose(delta);

    // Result should be (0.12, 0, 0)
    REQUIRE_THAT(f.pose().translation()(0), Catch::Matchers::WithinAbs(0.12, 1e-10));
}

TEST_CASE("Frame: Update pose with rotation", "[model][frame]") {
    // Initial: no pose offset
    Frame f("camera");

    // Rotate 90° about Z
    SE3 rotation = SE3(SO3::RotZ(M_PI/2).matrix(), Eigen::Vector3d::Zero());
    f.update_pose(rotation);

    // Check rotation matrix
    Eigen::Matrix3d R_expected;
    R_expected << 0, -1, 0,
                  1,  0, 0,
                  0,  0, 1;

    REQUIRE((f.pose().rotation() - R_expected).norm() < 1e-10);
}

// ============================================================================
// World Pose Tests (Scene Graph)
// ============================================================================

TEST_CASE("Frame: World pose without parent", "[model][frame]") {
    SE3 pose = SE3::Translation(Eigen::Vector3d(1, 2, 3));
    Frame f("test", pose);

    // No parent, so world pose = local pose
    REQUIRE(f.pose_world().isApprox(pose));

    Eigen::Vector3d t = f.pose_world().translation();
    REQUIRE(t(0) == 1.0);
    REQUIRE(t(1) == 2.0);
    REQUIRE(t(2) == 3.0);
}

TEST_CASE("Frame: World pose with Link parent", "[model][frame]") {
    // Link at (1, 0, 0)
    Link link("link5");
    link.set_pose(SE3::Translation(Eigen::Vector3d(1, 0, 0)));

    // Frame offset by (0, 1, 0) from link
    Frame f("tcp", SE3::Translation(Eigen::Vector3d(0, 1, 0)));
    f.set_parent(&link);

    // World pose should be (1, 1, 0)
    Eigen::Vector3d t = f.pose_world().translation();
    REQUIRE_THAT(t(0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(t(1), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(t(2), Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("Frame: World pose with Frame parent (Frame -> Frame)", "[model][frame]") {
    // Parent frame at (1, 0, 0)
    Frame parent("gripper", SE3::Translation(Eigen::Vector3d(1, 0, 0)));

    // Child frame offset by (0, 0, 0.1) from parent
    Frame child("tcp", SE3::Translation(Eigen::Vector3d(0, 0, 0.1)));
    child.set_parent(&parent);

    // World pose should be (1, 0, 0.1)
    Eigen::Vector3d t = child.pose_world().translation();
    REQUIRE_THAT(t(0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(t(1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(t(2), Catch::Matchers::WithinAbs(0.1, 1e-10));
}

TEST_CASE("Frame: Rotation composition in world pose", "[model][frame]") {
    // Parent rotated 90° about Z
    SE3 parent_pose = SE3(SO3::RotZ(M_PI/2).matrix(), Eigen::Vector3d::Zero());
    Frame parent("base", parent_pose);

    // Child rotated 90° about X in parent frame
    SE3 child_pose = SE3(SO3::RotX(M_PI/2).matrix(), Eigen::Vector3d::Zero());
    Frame child("sensor", child_pose);
    child.set_parent(&parent);

    // World rotation should be composition
    Eigen::Matrix3d R_world = child.pose_world().rotation();

    // Verify it's not identity and is a valid rotation
    REQUIRE((R_world - Eigen::Matrix3d::Identity()).norm() > 0.1);
    REQUIRE_THAT((R_world * R_world.transpose() - Eigen::Matrix3d::Identity()).norm(),
                 Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(R_world.determinant(), Catch::Matchers::WithinAbs(1.0, 1e-10));
}

// ============================================================================
// Use Case Tests
// ============================================================================

TEST_CASE("Use Case: TCP frame on robot flange", "[model][frame][usecase]") {
    // Robot has 6 links (0-5), flange is link 5
    Link flange("link5");

    // TCP is 10cm forward (Z-axis) from flange
    SE3 tcp_pose = SE3::Translation(Eigen::Vector3d(0, 0, 0.1));
    Frame tcp("tcp", tcp_pose);
    tcp.set_parent(&flange);

    REQUIRE(tcp.name() == "tcp");
    REQUIRE(tcp.parent() == &flange);
    REQUIRE_THAT(tcp.pose().translation()(2), Catch::Matchers::WithinAbs(0.1, 1e-10));
}

TEST_CASE("Use Case: Camera mounted on end-effector", "[model][frame][usecase]") {
    Link flange("link5");

    // Camera is 5cm to the side, rotated 90° looking sideways
    Eigen::Vector3d cam_pos(0.05, 0, 0);
    SE3 cam_rot = SE3(SO3::RotY(M_PI/2).matrix(), Eigen::Vector3d::Zero());
    SE3 cam_pose = SE3::Translation(cam_pos) * cam_rot;

    Frame camera("wrist_camera", cam_pose);
    camera.set_parent(&flange);

    REQUIRE(camera.name() == "wrist_camera");
    REQUIRE_THAT(camera.pose().translation()(0), Catch::Matchers::WithinAbs(0.05, 1e-10));
}

TEST_CASE("Use Case: Calibration target for laser tracker", "[model][frame][usecase]") {
    Link link3("link3");

    // SMR mount on link 3, initially at link origin
    Frame target("smr_target_3");
    target.set_parent(&link3);

    REQUIRE(target.name() == "smr_target_3");
    REQUIRE(target.pose().isApprox(SE3::Identity()));

    // After calibration measurement, update pose
    SE3 calibrated_pose = SE3::Translation(Eigen::Vector3d(0.001, 0.002, 0.003));
    target.set_pose(calibrated_pose);

    REQUIRE_THAT(target.pose().translation()(0), Catch::Matchers::WithinAbs(0.001, 1e-10));
    REQUIRE_THAT(target.pose().translation()(1), Catch::Matchers::WithinAbs(0.002, 1e-10));
    REQUIRE_THAT(target.pose().translation()(2), Catch::Matchers::WithinAbs(0.003, 1e-10));
}

TEST_CASE("Use Case: Multiple frames on same link", "[model][frame][usecase]") {
    // Link 5 (flange) has multiple frames
    Link flange("link5");

    Frame tcp("tcp", SE3::Translation(Eigen::Vector3d(0, 0, 0.1)));
    Frame camera("camera", SE3::Translation(Eigen::Vector3d(0.05, 0, 0.05)));
    Frame force_sensor("force_sensor", SE3::Translation(Eigen::Vector3d(0, 0, 0.02)));

    tcp.set_parent(&flange);
    camera.set_parent(&flange);
    force_sensor.set_parent(&flange);

    // All attached to same link
    REQUIRE(tcp.parent() == &flange);
    REQUIRE(camera.parent() == &flange);
    REQUIRE(force_sensor.parent() == &flange);

    // Different poses
    REQUIRE_FALSE(tcp.pose().isApprox(camera.pose()));
    REQUIRE_FALSE(tcp.pose().isApprox(force_sensor.pose()));
}

TEST_CASE("Use Case: Hand-eye calibration update", "[model][frame][usecase]") {
    Link flange("link5");

    // Initial camera frame (approximate)
    SE3 initial_guess = SE3::Translation(Eigen::Vector3d(0.05, 0, 0.05));
    Frame camera("camera", initial_guess);
    camera.set_parent(&flange);

    // After hand-eye calibration, we get a correction
    // Small rotation (1°) and translation adjustment
    SE3 correction = SE3::Translation(Eigen::Vector3d(0.001, 0.002, -0.001)) *
                     SE3(SO3::RotZ(0.017).matrix(), Eigen::Vector3d::Zero());  // ~1°

    camera.update_pose(correction);

    // Camera frame is now corrected
    // Translation should be approximately (0.051, 0.002, 0.049)
    REQUIRE(camera.pose().translation()(0) > 0.05);  // Slightly more than 0.05
    REQUIRE(camera.pose().translation()(1) > 0.0);   // Shifted in Y
}

TEST_CASE("Use Case: Robot on table (Frame as world reference)", "[model][frame][usecase]") {
    // Table frame in world
    Frame table("table", SE3::Translation(Eigen::Vector3d(1, 0, 0.8)));

    // Robot base link on table
    Link robot_base("base_link");
    robot_base.set_parent(&table);
    robot_base.set_pose(SE3::Translation(Eigen::Vector3d(0, 0, 0.1)));  // 10cm above table

    // World pose of robot should be (1, 0, 0.9)
    Eigen::Vector3d world_pos = robot_base.pose_world().translation();
    REQUIRE_THAT(world_pos(0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(world_pos(1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(world_pos(2), Catch::Matchers::WithinAbs(0.9, 1e-10));
}

TEST_CASE("Use Case: Nested frames (gripper -> adapter -> tool)", "[model][frame][usecase]") {
    Link flange("link6");

    // Gripper frame 10cm from flange
    Frame gripper("gripper", SE3::Translation(Eigen::Vector3d(0, 0, 0.1)));
    gripper.set_parent(&flange);

    // Adapter frame 5cm from gripper
    Frame adapter("adapter", SE3::Translation(Eigen::Vector3d(0, 0, 0.05)));
    adapter.set_parent(&gripper);

    // Tool TCP 8cm from adapter
    Frame tool_tcp("tool_tcp", SE3::Translation(Eigen::Vector3d(0, 0, 0.08)));
    tool_tcp.set_parent(&adapter);

    // TCP should be 23cm (10+5+8) from flange in flange's Z direction
    flange.set_pose(SE3::Identity());  // For easy calculation
    Eigen::Vector3d tcp_world = tool_tcp.pose_world().translation();

    REQUIRE_THAT(tcp_world(2), Catch::Matchers::WithinAbs(0.23, 1e-10));
}
