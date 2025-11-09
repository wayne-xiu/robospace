#include <catch2/catch_test_macros.hpp>
#include <robospace/math/rotation.hpp>

using namespace robospace::math;

TEST_CASE("Rotation: Identity construction", "[rotation]") {
    Rotation R;

    REQUIRE(R.matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST_CASE("Rotation: Identity factory", "[rotation]") {
    Rotation R = Rotation::Identity();

    REQUIRE(R.matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST_CASE("Rotation: Construction from matrix", "[rotation]") {
    Eigen::Matrix3d mat;
    mat << 0, -1, 0,
           1,  0, 0,
           0,  0, 1;  // 90° around Z

    Rotation R(mat);

    REQUIRE(R.matrix().isApprox(mat));
}

TEST_CASE("Rotation: Construction from quaternion", "[rotation]") {
    // 90° around Z axis
    Eigen::Quaterniond q(std::cos(M_PI/4), 0, 0, std::sin(M_PI/4));

    Rotation R(q);

    REQUIRE(R.quaternion().isApprox(q.normalized()));
}

TEST_CASE("Rotation: Construction from axis-angle", "[rotation]") {
    Eigen::Vector3d axis(0, 0, 1);  // Z axis
    double angle = M_PI / 2;         // 90°

    Rotation R(axis, angle);

    Eigen::Matrix3d expected;
    expected << 0, -1, 0,
                1,  0, 0,
                0,  0, 1;

    REQUIRE(R.matrix().isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: RotX factory", "[rotation]") {
    double angle = M_PI / 2;  // 90°
    Rotation R = Rotation::RotX(angle);

    Eigen::Matrix3d expected;
    expected << 1,  0,  0,
                0,  0, -1,
                0,  1,  0;

    REQUIRE(R.matrix().isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: RotY factory", "[rotation]") {
    double angle = M_PI / 2;  // 90°
    Rotation R = Rotation::RotY(angle);

    Eigen::Matrix3d expected;
    expected <<  0, 0, 1,
                 0, 1, 0,
                -1, 0, 0;

    REQUIRE(R.matrix().isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: RotZ factory", "[rotation]") {
    double angle = M_PI / 2;  // 90°
    Rotation R = Rotation::RotZ(angle);

    Eigen::Matrix3d expected;
    expected << 0, -1, 0,
                1,  0, 0,
                0,  0, 1;

    REQUIRE(R.matrix().isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: FromEuler XYZ (Roll-Pitch-Yaw)", "[rotation]") {
    double roll = M_PI / 4;   // 45°
    double pitch = 0.0;
    double yaw = 0.0;

    Rotation R = Rotation::FromEuler(roll, pitch, yaw, EulerConvention::XYZ);

    // Should be same as RotX(45°)
    Rotation expected = Rotation::RotX(roll);
    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: FromEuler with vector", "[rotation]") {
    Eigen::Vector3d angles(M_PI/4, 0, 0);  // 45° roll

    Rotation R = Rotation::FromEuler(angles, EulerConvention::XYZ);
    Rotation expected = Rotation::RotX(M_PI/4);

    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: FromAxisAngle factory", "[rotation]") {
    Eigen::Vector3d axis(1, 1, 1);  // Will be normalized
    double angle = M_PI / 3;

    Rotation R = Rotation::FromAxisAngle(axis, angle);

    Eigen::Vector3d axis_out;
    double angle_out;
    R.axisAngle(axis_out, angle_out);

    REQUIRE(axis_out.isApprox(axis.normalized(), 1e-10));
    REQUIRE(std::abs(angle_out - angle) < 1e-10);
}

TEST_CASE("Rotation: FromRotationVector", "[rotation]") {
    Eigen::Vector3d rotvec(0, 0, M_PI/2);  // 90° around Z

    Rotation R = Rotation::FromRotationVector(rotvec);
    Rotation expected = Rotation::RotZ(M_PI/2);

    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: FromRotationVector near-zero", "[rotation]") {
    Eigen::Vector3d rotvec(1e-12, 1e-12, 1e-12);

    Rotation R = Rotation::FromRotationVector(rotvec);

    REQUIRE(R.matrix().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST_CASE("Rotation: Quaternion conversion", "[rotation]") {
    Rotation R = Rotation::RotZ(M_PI / 2);
    Eigen::Quaterniond q = R.quaternion();

    // Reconstruct rotation from quaternion
    Rotation R2(q);

    REQUIRE(R.isApprox(R2, 1e-10));
}

TEST_CASE("Rotation: Euler angles round-trip XYZ", "[rotation]") {
    double roll = M_PI / 6;
    double pitch = M_PI / 4;
    double yaw = M_PI / 3;

    Rotation R = Rotation::FromEuler(roll, pitch, yaw, EulerConvention::XYZ);
    Eigen::Vector3d angles_out = R.eulerAngles(EulerConvention::XYZ);

    // Note: Euler angles are not unique, so we check by reconstructing
    Rotation R2 = Rotation::FromEuler(angles_out, EulerConvention::XYZ);
    REQUIRE(R.isApprox(R2, 1e-8));
}

TEST_CASE("Rotation: Axis-angle round-trip", "[rotation]") {
    Eigen::Vector3d axis_in(1, 2, 3);
    axis_in.normalize();
    double angle_in = M_PI / 3;

    Rotation R = Rotation::FromAxisAngle(axis_in, angle_in);

    Eigen::Vector3d axis_out;
    double angle_out;
    R.axisAngle(axis_out, angle_out);

    REQUIRE(axis_out.isApprox(axis_in, 1e-10));
    REQUIRE(std::abs(angle_out - angle_in) < 1e-10);
}

TEST_CASE("Rotation: Rotation vector round-trip", "[rotation]") {
    Eigen::Vector3d rotvec_in(0.5, 0.3, 0.2);

    Rotation R = Rotation::FromRotationVector(rotvec_in);
    Eigen::Vector3d rotvec_out = R.rotationVector();

    REQUIRE(rotvec_out.isApprox(rotvec_in, 1e-10));
}

TEST_CASE("Rotation: Inverse", "[rotation]") {
    Rotation R = Rotation::RotZ(M_PI / 4);
    Rotation R_inv = R.inverse();

    // R * R^-1 = I
    Rotation I = R * R_inv;
    REQUIRE(I.isApprox(Rotation::Identity(), 1e-10));
}

TEST_CASE("Rotation: Composition", "[rotation]") {
    Rotation R1 = Rotation::RotZ(M_PI / 4);   // 45° around Z
    Rotation R2 = Rotation::RotZ(M_PI / 4);   // Another 45° around Z

    Rotation R = R1 * R2;
    Rotation expected = Rotation::RotZ(M_PI / 2);  // 90° total

    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: Rotate vector", "[rotation]") {
    Rotation R = Rotation::RotZ(M_PI / 2);  // 90° around Z
    Eigen::Vector3d v(1, 0, 0);  // X axis

    Eigen::Vector3d v_rotated = R * v;
    Eigen::Vector3d expected(0, 1, 0);  // Y axis

    REQUIRE(v_rotated.isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: Rotate vector (45°)", "[rotation]") {
    Rotation R = Rotation::RotZ(M_PI / 4);  // 45° around Z
    Eigen::Vector3d v(1, 0, 0);

    Eigen::Vector3d v_rotated = R * v;
    double sqrt2_2 = std::sqrt(2.0) / 2.0;
    Eigen::Vector3d expected(sqrt2_2, sqrt2_2, 0);

    REQUIRE(v_rotated.isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: In-place composition", "[rotation]") {
    Rotation R1 = Rotation::RotZ(M_PI / 4);
    Rotation R2 = Rotation::RotZ(M_PI / 4);

    R1 *= R2;
    Rotation expected = Rotation::RotZ(M_PI / 2);

    REQUIRE(R1.isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: isApprox", "[rotation]") {
    Rotation R1 = Rotation::RotZ(M_PI / 4);
    Rotation R2 = Rotation::RotZ(M_PI / 4 + 1e-11);

    REQUIRE(R1.isApprox(R2, 1e-9));
    REQUIRE_FALSE(R1.isApprox(R2, 1e-12));
}

TEST_CASE("Rotation: Non-commutative composition", "[rotation]") {
    Rotation Rx = Rotation::RotX(M_PI / 4);
    Rotation Ry = Rotation::RotY(M_PI / 4);

    Rotation R1 = Rx * Ry;
    Rotation R2 = Ry * Rx;

    // Rotations don't commute
    REQUIRE_FALSE(R1.isApprox(R2, 1e-6));
}

TEST_CASE("Rotation: Identity inverse", "[rotation]") {
    Rotation R = Rotation::Identity();
    Rotation R_inv = R.inverse();

    REQUIRE(R_inv.isApprox(Rotation::Identity()));
}

TEST_CASE("Rotation: 180° rotation", "[rotation]") {
    Rotation R = Rotation::RotZ(M_PI);  // 180°
    Eigen::Vector3d v(1, 0, 0);

    Eigen::Vector3d v_rotated = R * v;
    Eigen::Vector3d expected(-1, 0, 0);

    REQUIRE(v_rotated.isApprox(expected, 1e-10));
}

TEST_CASE("Rotation: Multiple axes composition", "[rotation]") {
    Rotation Rx = Rotation::RotX(M_PI / 6);
    Rotation Ry = Rotation::RotY(M_PI / 4);
    Rotation Rz = Rotation::RotZ(M_PI / 3);

    Rotation R = Rz * Ry * Rx;

    // Test that inverse works
    Rotation R_inv = R.inverse();
    Rotation I = R * R_inv;

    REQUIRE(I.isApprox(Rotation::Identity(), 1e-10));
}
