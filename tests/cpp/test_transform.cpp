#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/math/transform.hpp>

using namespace robospace::math;
using Catch::Matchers::WithinAbs;

TEST_CASE("Transform: Identity construction", "[transform]") {
    Transform T;

    REQUIRE(T.matrix().isApprox(Eigen::Matrix4d::Identity()));
    REQUIRE(T.rotation().isApprox(Eigen::Matrix3d::Identity()));
    REQUIRE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("Transform: Identity factory", "[transform]") {
    Transform T = Transform::Identity();

    REQUIRE(T.matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST_CASE("Transform: Translation only", "[transform]") {
    Eigen::Vector3d p(1.0, 2.0, 3.0);
    Transform T(p);

    REQUIRE(T.rotation().isApprox(Eigen::Matrix3d::Identity()));
    REQUIRE(T.translation().isApprox(p));
}

TEST_CASE("Transform: Translation factory", "[transform]") {
    Transform T = Transform::Translation(1.0, 2.0, 3.0);

    REQUIRE(T.translation().x() == 1.0);
    REQUIRE(T.translation().y() == 2.0);
    REQUIRE(T.translation().z() == 3.0);
}

TEST_CASE("Transform: Rotation around X axis", "[transform]") {
    double angle = M_PI / 2.0;  // 90 degrees
    Transform T = Transform::RotX(angle);

    // Rotation matrix for 90° around X
    Eigen::Matrix3d expected_R;
    expected_R << 1,  0,  0,
                  0,  0, -1,
                  0,  1,  0;

    REQUIRE(T.rotation().isApprox(expected_R, 1e-10));
    REQUIRE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("Transform: Rotation around Y axis", "[transform]") {
    double angle = M_PI / 2.0;  // 90 degrees
    Transform T = Transform::RotY(angle);

    // Rotation matrix for 90° around Y
    Eigen::Matrix3d expected_R;
    expected_R <<  0, 0, 1,
                   0, 1, 0,
                  -1, 0, 0;

    REQUIRE(T.rotation().isApprox(expected_R, 1e-10));
}

TEST_CASE("Transform: Rotation around Z axis", "[transform]") {
    double angle = M_PI / 2.0;  // 90 degrees
    Transform T = Transform::RotZ(angle);

    // Rotation matrix for 90° around Z
    Eigen::Matrix3d expected_R;
    expected_R << 0, -1, 0,
                  1,  0, 0,
                  0,  0, 1;

    REQUIRE(T.rotation().isApprox(expected_R, 1e-10));
}

TEST_CASE("Transform: Construction from R and p", "[transform]") {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p(1.0, 2.0, 3.0);

    Transform T(R, p);

    REQUIRE(T.rotation().isApprox(R));
    REQUIRE(T.translation().isApprox(p));
}

TEST_CASE("Transform: Decompose", "[transform]") {
    Eigen::Matrix3d R = Transform::RotZ(M_PI / 4.0).rotation();
    Eigen::Vector3d p(1.0, 2.0, 3.0);

    Transform T(R, p);

    Eigen::Matrix3d R_out;
    Eigen::Vector3d p_out;
    T.decompose(R_out, p_out);

    REQUIRE(R_out.isApprox(R));
    REQUIRE(p_out.isApprox(p));
}

TEST_CASE("Transform: Inverse of pure translation", "[transform]") {
    Transform T = Transform::Translation(1.0, 2.0, 3.0);
    Transform T_inv = T.inverse();

    REQUIRE(T_inv.translation().isApprox(Eigen::Vector3d(-1.0, -2.0, -3.0)));

    // T * T_inv = I
    Transform result = T * T_inv;
    REQUIRE(result.isApprox(Transform::Identity()));
}

TEST_CASE("Transform: Inverse of pure rotation", "[transform]") {
    Transform T = Transform::RotZ(M_PI / 3.0);
    Transform T_inv = T.inverse();

    // R * R^T = I
    Transform result = T * T_inv;
    REQUIRE(result.isApprox(Transform::Identity(), 1e-10));
}

TEST_CASE("Transform: Inverse of combined transformation", "[transform]") {
    Transform T_rot = Transform::RotZ(M_PI / 4.0);
    Transform T_trans = Transform::Translation(1.0, 2.0, 3.0);
    Transform T = T_trans * T_rot;

    Transform T_inv = T.inverse();
    Transform result = T * T_inv;

    REQUIRE(result.isApprox(Transform::Identity(), 1e-10));
}

TEST_CASE("Transform: Composition", "[transform]") {
    Transform T1 = Transform::Translation(1.0, 0.0, 0.0);
    Transform T2 = Transform::Translation(2.0, 0.0, 0.0);

    Transform T = T1 * T2;

    REQUIRE(T.translation().x() == 3.0);
}

TEST_CASE("Transform: Transform point", "[transform]") {
    Transform T = Transform::Translation(1.0, 2.0, 3.0);
    Eigen::Vector3d p(1.0, 1.0, 1.0);

    Eigen::Vector3d p_transformed = T * p;

    REQUIRE(p_transformed.isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));
}

TEST_CASE("Transform: Transform point with rotation", "[transform]") {
    // Rotate 90° around Z, then translate
    Transform T_rot = Transform::RotZ(M_PI / 2.0);
    Transform T_trans = Transform::Translation(1.0, 0.0, 0.0);
    Transform T = T_trans * T_rot;

    Eigen::Vector3d p(1.0, 0.0, 0.0);
    Eigen::Vector3d p_transformed = T * p;

    // After rotation: (0, 1, 0), after translation: (1, 1, 0)
    REQUIRE(p_transformed.isApprox(Eigen::Vector3d(1.0, 1.0, 0.0), 1e-10));
}

TEST_CASE("Transform: Composition order matters", "[transform]") {
    Transform T_rot = Transform::RotZ(M_PI / 2.0);
    Transform T_trans = Transform::Translation(1.0, 0.0, 0.0);

    Transform T1 = T_trans * T_rot;  // Translate first, then rotate
    Transform T2 = T_rot * T_trans;  // Rotate first, then translate

    REQUIRE_FALSE(T1.isApprox(T2));
}

TEST_CASE("Transform: In-place composition", "[transform]") {
    Transform T1 = Transform::Translation(1.0, 0.0, 0.0);
    Transform T2 = Transform::Translation(2.0, 0.0, 0.0);

    T1 *= T2;

    REQUIRE(T1.translation().x() == 3.0);
}

TEST_CASE("Transform: isApprox", "[transform]") {
    Transform T1 = Transform::Translation(1.0, 2.0, 3.0);
    Transform T2 = Transform::Translation(1.0 + 1e-11, 2.0, 3.0);

    REQUIRE(T1.isApprox(T2, 1e-9));
    REQUIRE_FALSE(T1.isApprox(T2, 1e-12));
}

TEST_CASE("Transform: Conversion to Isometry3d", "[transform]") {
    Eigen::Matrix3d R = Transform::RotZ(M_PI / 4.0).rotation();
    Eigen::Vector3d p(1.0, 2.0, 3.0);
    Transform T(R, p);

    Eigen::Isometry3d iso = T.toIsometry();

    REQUIRE(iso.matrix().isApprox(T.matrix()));
}

TEST_CASE("Transform: Construction from Isometry3d", "[transform]") {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

    Transform T(iso);

    REQUIRE(T.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}
