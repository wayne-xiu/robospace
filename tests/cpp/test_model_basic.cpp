#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/dh_params.hpp>
#include <robospace/model/joint.hpp>
#include <robospace/model/link.hpp>

using namespace robospace::model;
using namespace robospace::math;

// ============================================================================
// DHParams Tests
// ============================================================================

TEST_CASE("DHParams: Construction", "[model][dh]") {
    DHParams dh(M_PI/2, 0.1, 0.2, 0.0, DHConvention::MODIFIED);

    REQUIRE(dh.alpha == M_PI/2);
    REQUIRE(dh.a == 0.1);
    REQUIRE(dh.d == 0.2);
    REQUIRE(dh.theta == 0.0);
    REQUIRE(dh.convention == DHConvention::MODIFIED);
}

TEST_CASE("DHParams: Modified DH transform (revolute, zero config)", "[model][dh]") {
    // Modified DH: Rot_x(α) * Trans_x(a) * Trans_z(d) * Rot_z(θ)
    DHParams dh(M_PI/2, 0.1, 0.2, 0.0, DHConvention::MODIFIED);

    SE3 T = dh.transform(0.0, false);  // q=0, revolute

    // Expected translation: [a, -d*sin(α), d*cos(α)] = [0.1, -0.2*1, 0.2*0] = [0.1, -0.2, 0]
    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(0.1, 1e-10));
    REQUIRE_THAT(T.translation()(1), Catch::Matchers::WithinAbs(-0.2, 1e-10));
    REQUIRE_THAT(T.translation()(2), Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("DHParams: Standard DH transform (revolute, zero config)", "[model][dh]") {
    // Standard DH: Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
    DHParams dh(M_PI/2, 0.1, 0.2, 0.0, DHConvention::STANDARD);

    SE3 T = dh.transform(0.0, false);  // q=0, revolute

    // Expected translation: [a*cos(θ), a*sin(θ), d] = [0.1*1, 0.1*0, 0.2] = [0.1, 0, 0.2]
    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(0.1, 1e-10));
    REQUIRE_THAT(T.translation()(1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(T.translation()(2), Catch::Matchers::WithinAbs(0.2, 1e-10));
}

TEST_CASE("DHParams: Revolute joint with q != 0", "[model][dh]") {
    DHParams dh(0, 0.5, 0, 0, DHConvention::MODIFIED);

    SE3 T = dh.transform(M_PI/2, false);  // 90° rotation

    // Should have translation [0.5, 0, 0] and rotation of 90° about Z
    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(0.5, 1e-10));

    // Rotation matrix should be Rot_z(90°)
    Eigen::Matrix3d R_expected;
    R_expected << 0, -1, 0,
                  1,  0, 0,
                  0,  0, 1;
    REQUIRE((T.rotation() - R_expected).norm() < 1e-10);
}

TEST_CASE("DHParams: Prismatic joint", "[model][dh]") {
    DHParams dh(0, 0.1, 0, 0, DHConvention::MODIFIED);

    SE3 T = dh.transform(0.3, true);  // prismatic, d += 0.3

    // Translation should be [0.1, 0, 0.3] for modified DH
    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(0.1, 1e-10));
    REQUIRE_THAT(T.translation()(2), Catch::Matchers::WithinAbs(0.3, 1e-10));
}

// ============================================================================
// Joint Tests
// ============================================================================

TEST_CASE("Joint: Construction", "[model][joint]") {
    Joint j("shoulder_pan", JointType::REVOLUTE, 0, 1);

    REQUIRE(j.name() == "shoulder_pan");
    REQUIRE(j.type() == JointType::REVOLUTE);
    REQUIRE(j.parent_link_id() == 0);
    REQUIRE(j.child_link_id() == 1);
    REQUIRE(j.is_revolute());
    REQUIRE_FALSE(j.is_prismatic());
    REQUIRE_FALSE(j.is_fixed());
}

TEST_CASE("Joint: Default limits for revolute", "[model][joint]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);

    REQUIRE(j.lower_limit() == -M_PI);
    REQUIRE(j.upper_limit() == M_PI);
    REQUIRE(j.has_position_limits());
}

TEST_CASE("Joint: Continuous joint has no limits", "[model][joint]") {
    Joint j("test", JointType::CONTINUOUS, 0, 1);

    REQUIRE_FALSE(j.has_position_limits());
    REQUIRE(j.is_within_limits(100.0));  // Any value is valid
}

TEST_CASE("Joint: Set custom limits", "[model][joint]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);
    j.set_limits(-1.5, 1.5);

    REQUIRE(j.lower_limit() == -1.5);
    REQUIRE(j.upper_limit() == 1.5);
    REQUIRE(j.is_within_limits(0.0));
    REQUIRE(j.is_within_limits(1.0));
    REQUIRE_FALSE(j.is_within_limits(2.0));
}

TEST_CASE("Joint: DH parameters", "[model][joint]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);

    REQUIRE_FALSE(j.has_dh_params());

    DHParams dh(M_PI/2, 0.1, 0.2, 0.0, DHConvention::MODIFIED);
    j.set_dh_params(dh);

    REQUIRE(j.has_dh_params());
    REQUIRE(j.dh_params().alpha == M_PI/2);
}

TEST_CASE("Joint: Transform using DH", "[model][joint]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);
    DHParams dh(0, 0.5, 0, 0, DHConvention::MODIFIED);
    j.set_dh_params(dh);

    SE3 T = j.transform(M_PI/2);

    // Should get Rot_z(90°) with translation [0.5, 0, 0]
    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(0.5, 1e-10));
}

TEST_CASE("Joint: Transform using origin + axis", "[model][joint]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);
    j.set_origin(SE3::Translation(Eigen::Vector3d(0.1, 0.2, 0.3)));
    j.set_axis(Eigen::Vector3d::UnitZ());  // Rotate about Z

    SE3 T = j.transform(M_PI/2);

    // Should have rotation about Z by 90° plus origin translation
    // Translation = origin.p + R(axis, 90°) * 0
    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(0.1, 1e-10));
    REQUIRE_THAT(T.translation()(1), Catch::Matchers::WithinAbs(0.2, 1e-10));
    REQUIRE_THAT(T.translation()(2), Catch::Matchers::WithinAbs(0.3, 1e-10));
}

TEST_CASE("Joint: Fixed joint returns origin", "[model][joint]") {
    Joint j("test", JointType::FIXED, 0, 1);
    j.set_origin(SE3::Translation(Eigen::Vector3d(1, 2, 3)));

    SE3 T = j.transform(0);  // q is ignored for fixed

    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(T.translation()(1), Catch::Matchers::WithinAbs(2.0, 1e-10));
    REQUIRE_THAT(T.translation()(2), Catch::Matchers::WithinAbs(3.0, 1e-10));
}

// ============================================================================
// Link Tests
// ============================================================================

TEST_CASE("Link: Construction", "[model][link]") {
    Link link("base_link");

    REQUIRE(link.name() == "base_link");
    REQUIRE(link.mass() == 0.0);
    REQUIRE_FALSE(link.has_inertial());
    REQUIRE_FALSE(link.has_visual());
    REQUIRE_FALSE(link.has_collision());
}

TEST_CASE("Link: Set physical properties", "[model][link]") {
    Link link("test");

    link.set_mass(5.0);
    link.set_com(Eigen::Vector3d(0.1, 0.2, 0.3));
    link.set_inertia_diagonal(1.0, 2.0, 3.0);

    REQUIRE(link.mass() == 5.0);
    REQUIRE(link.has_inertial());
    REQUIRE(link.com()(0) == 0.1);
    REQUIRE(link.com()(1) == 0.2);
    REQUIRE(link.com()(2) == 0.3);
    REQUIRE(link.inertia()(0, 0) == 1.0);
    REQUIRE(link.inertia()(1, 1) == 2.0);
    REQUIRE(link.inertia()(2, 2) == 3.0);
}

TEST_CASE("Link: Set geometry", "[model][link]") {
    Link link("test");

    link.set_visual_mesh("meshes/visual.stl");
    link.set_collision_mesh("meshes/collision.stl");

    REQUIRE(link.has_visual());
    REQUIRE(link.has_collision());
    REQUIRE(link.visual_mesh() == "meshes/visual.stl");
    REQUIRE(link.collision_mesh() == "meshes/collision.stl");
}
