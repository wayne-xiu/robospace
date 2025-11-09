#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/frame.hpp>
#include <robospace/math/SO3.hpp>

using namespace robospace::model;
using namespace robospace::math;

// ============================================================================
// Frame Construction Tests
// ============================================================================

TEST_CASE("Frame: Construction with offset", "[model][frame]") {
    SE3 offset = SE3::Translation(Eigen::Vector3d(0.1, 0.2, 0.3));
    Frame f("tcp", 5, offset);

    REQUIRE(f.name() == "tcp");
    REQUIRE(f.parent_link_id() == 5);
    REQUIRE(f.offset().isApprox(offset));
    REQUIRE_FALSE(f.is_root());
}

TEST_CASE("Frame: Construction with identity offset", "[model][frame]") {
    Frame f("camera", 3);

    REQUIRE(f.name() == "camera");
    REQUIRE(f.parent_link_id() == 3);
    REQUIRE(f.offset().isApprox(SE3::Identity()));
}

TEST_CASE("Frame: Root frame (world)", "[model][frame]") {
    Frame f("world", -1);

    REQUIRE(f.name() == "world");
    REQUIRE(f.parent_link_id() == -1);
    REQUIRE(f.is_root());
}

// ============================================================================
// Name and Parent Modification
// ============================================================================

TEST_CASE("Frame: Set name", "[model][frame]") {
    Frame f("temp", 0);

    f.set_name("new_name");
    REQUIRE(f.name() == "new_name");
}

TEST_CASE("Frame: Change parent link", "[model][frame]") {
    Frame f("sensor", 2);

    REQUIRE(f.parent_link_id() == 2);

    f.set_parent_link_id(5);
    REQUIRE(f.parent_link_id() == 5);
}

// ============================================================================
// Offset Manipulation
// ============================================================================

TEST_CASE("Frame: Set offset", "[model][frame]") {
    Frame f("tcp", 5);

    // Initially identity
    REQUIRE(f.offset().isApprox(SE3::Identity()));

    // Set new offset
    SE3 new_offset = SE3::Translation(Eigen::Vector3d(0.05, 0, 0.1));
    f.set_offset(new_offset);

    REQUIRE(f.offset().isApprox(new_offset));
    REQUIRE_THAT(f.translation()(0), Catch::Matchers::WithinAbs(0.05, 1e-10));
    REQUIRE_THAT(f.translation()(2), Catch::Matchers::WithinAbs(0.1, 1e-10));
}

TEST_CASE("Frame: Update offset (incremental)", "[model][frame]") {
    // Initial offset: translate by (0.1, 0, 0)
    SE3 initial = SE3::Translation(Eigen::Vector3d(0.1, 0, 0));
    Frame f("tcp", 5, initial);

    // Add delta: translate by (0.02, 0, 0)
    SE3 delta = SE3::Translation(Eigen::Vector3d(0.02, 0, 0));
    f.update_offset(delta);

    // Result should be (0.12, 0, 0)
    REQUIRE_THAT(f.translation()(0), Catch::Matchers::WithinAbs(0.12, 1e-10));
}

TEST_CASE("Frame: Update offset with rotation", "[model][frame]") {
    // Initial: no offset
    Frame f("camera", 3, SE3::Identity());

    // Rotate 90° about Z
    SE3 rotation = SE3(SO3::RotZ(M_PI/2).matrix(), Eigen::Vector3d::Zero());
    f.update_offset(rotation);

    // Check rotation matrix
    Eigen::Matrix3d R_expected;
    R_expected << 0, -1, 0,
                  1,  0, 0,
                  0,  0, 1;

    REQUIRE((f.rotation() - R_expected).norm() < 1e-10);
}

// ============================================================================
// Convenience Accessors
// ============================================================================

TEST_CASE("Frame: Translation accessor", "[model][frame]") {
    SE3 offset = SE3::Translation(Eigen::Vector3d(1, 2, 3));
    Frame f("test", 0, offset);

    Eigen::Vector3d t = f.translation();
    REQUIRE(t(0) == 1.0);
    REQUIRE(t(1) == 2.0);
    REQUIRE(t(2) == 3.0);
}

TEST_CASE("Frame: Rotation accessor", "[model][frame]") {
    SE3 offset = SE3(SO3::RotX(M_PI/2).matrix(), Eigen::Vector3d::Zero());
    Frame f("test", 0, offset);

    Eigen::Matrix3d R = f.rotation();
    Eigen::Matrix3d R_expected;
    R_expected << 1,  0,  0,
                  0,  0, -1,
                  0,  1,  0;

    REQUIRE((R - R_expected).norm() < 1e-10);
}

// ============================================================================
// Use Case Tests
// ============================================================================

TEST_CASE("Use Case: TCP frame on robot flange", "[model][frame][usecase]") {
    // Robot has 6 links (0-5), flange is link 5
    // TCP is 10cm forward (Z-axis) from flange
    SE3 tcp_offset = SE3::Translation(Eigen::Vector3d(0, 0, 0.1));
    Frame tcp("tcp", 5, tcp_offset);

    REQUIRE(tcp.name() == "tcp");
    REQUIRE(tcp.parent_link_id() == 5);
    REQUIRE_THAT(tcp.translation()(2), Catch::Matchers::WithinAbs(0.1, 1e-10));
}

TEST_CASE("Use Case: Camera mounted on end-effector", "[model][frame][usecase]") {
    // Camera is 5cm to the side, rotated 90° looking sideways
    Eigen::Vector3d cam_pos(0.05, 0, 0);
    SE3 cam_rot = SE3(SO3::RotY(M_PI/2).matrix(), Eigen::Vector3d::Zero());
    SE3 cam_offset = SE3::Translation(cam_pos) * cam_rot;

    Frame camera("wrist_camera", 5, cam_offset);

    REQUIRE(camera.name() == "wrist_camera");
    REQUIRE_THAT(camera.translation()(0), Catch::Matchers::WithinAbs(0.05, 1e-10));
}

TEST_CASE("Use Case: Calibration target for laser tracker", "[model][frame][usecase]") {
    // SMR mount on link 3, initially at link origin
    Frame target("smr_target_3", 3);

    REQUIRE(target.name() == "smr_target_3");
    REQUIRE(target.offset().isApprox(SE3::Identity()));

    // After calibration measurement, update offset
    SE3 calibrated_offset = SE3::Translation(Eigen::Vector3d(0.001, 0.002, 0.003));
    target.set_offset(calibrated_offset);

    REQUIRE_THAT(target.translation()(0), Catch::Matchers::WithinAbs(0.001, 1e-10));
    REQUIRE_THAT(target.translation()(1), Catch::Matchers::WithinAbs(0.002, 1e-10));
    REQUIRE_THAT(target.translation()(2), Catch::Matchers::WithinAbs(0.003, 1e-10));
}

TEST_CASE("Use Case: Multiple frames on same link", "[model][frame][usecase]") {
    // Link 5 (flange) has multiple frames
    Frame tcp("tcp", 5, SE3::Translation(Eigen::Vector3d(0, 0, 0.1)));
    Frame camera("camera", 5, SE3::Translation(Eigen::Vector3d(0.05, 0, 0.05)));
    Frame force_sensor("force_sensor", 5, SE3::Translation(Eigen::Vector3d(0, 0, 0.02)));

    // All attached to same link
    REQUIRE(tcp.parent_link_id() == 5);
    REQUIRE(camera.parent_link_id() == 5);
    REQUIRE(force_sensor.parent_link_id() == 5);

    // Different offsets
    REQUIRE_FALSE(tcp.offset().isApprox(camera.offset()));
    REQUIRE_FALSE(tcp.offset().isApprox(force_sensor.offset()));
}

TEST_CASE("Use Case: Hand-eye calibration update", "[model][frame][usecase]") {
    // Initial camera frame (approximate)
    SE3 initial_guess = SE3::Translation(Eigen::Vector3d(0.05, 0, 0.05));
    Frame camera("camera", 5, initial_guess);

    // After hand-eye calibration, we get a correction
    // Small rotation (1°) and translation adjustment
    SE3 correction = SE3::Translation(Eigen::Vector3d(0.001, 0.002, -0.001)) *
                     SE3(SO3::RotZ(0.017).matrix(), Eigen::Vector3d::Zero());  // ~1°

    camera.update_offset(correction);

    // Camera frame is now corrected
    // Translation should be approximately (0.051, 0.002, 0.049)
    REQUIRE(camera.translation()(0) > 0.05);  // Slightly more than 0.05
    REQUIRE(camera.translation()(1) > 0.0);   // Shifted in Y
}
