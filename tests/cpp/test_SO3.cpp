#include <catch2/catch_test_macros.hpp>
#include <robospace/math/SO3.hpp>

using namespace robospace::math;

TEST_CASE("SO3: Identity construction", "[SO3]") {
    SO3 R;

    REQUIRE(R.matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST_CASE("SO3: Identity factory", "[SO3]") {
    SO3 R = SO3::Identity();

    REQUIRE(R.matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST_CASE("SO3: Construction from matrix", "[SO3]") {
    Eigen::Matrix3d mat;
    mat << 0, -1, 0,
           1,  0, 0,
           0,  0, 1;  // 90° around Z

    SO3 R(mat);

    REQUIRE(R.matrix().isApprox(mat));
}

TEST_CASE("SO3: Construction from axis-angle", "[SO3]") {
    Eigen::Vector3d axis(0, 0, 1);  // Z axis
    double angle = M_PI / 2;         // 90°

    SO3 R(axis, angle);

    Eigen::Matrix3d expected;
    expected << 0, -1, 0,
                1,  0, 0,
                0,  0, 1;

    REQUIRE(R.matrix().isApprox(expected, 1e-10));
}

TEST_CASE("SO3: FromAxisAngle factory", "[SO3]") {
    Eigen::Vector3d axis(1, 0, 0);
    double angle = M_PI / 4;

    SO3 R = SO3::FromAxisAngle(axis, angle);

    Eigen::Vector3d axis_out;
    double angle_out;
    R.axisAngle(axis_out, angle_out);

    REQUIRE(axis_out.isApprox(axis, 1e-10));
    REQUIRE(std::abs(angle_out - angle) < 1e-10);
}

TEST_CASE("SO3: RotX factory", "[SO3]") {
    SO3 R = SO3::RotX(M_PI / 2);

    Eigen::Matrix3d expected;
    expected << 1,  0,  0,
                0,  0, -1,
                0,  1,  0;

    REQUIRE(R.matrix().isApprox(expected, 1e-10));
}

TEST_CASE("SO3: RotY factory", "[SO3]") {
    SO3 R = SO3::RotY(M_PI / 2);

    Eigen::Matrix3d expected;
    expected <<  0, 0, 1,
                 0, 1, 0,
                -1, 0, 0;

    REQUIRE(R.matrix().isApprox(expected, 1e-10));
}

TEST_CASE("SO3: RotZ factory", "[SO3]") {
    SO3 R = SO3::RotZ(M_PI / 2);

    Eigen::Matrix3d expected;
    expected << 0, -1, 0,
                1,  0, 0,
                0,  0, 1;

    REQUIRE(R.matrix().isApprox(expected, 1e-10));
}

TEST_CASE("SO3: Axis-angle extraction", "[SO3]") {
    Eigen::Vector3d axis_in(1, 1, 1);
    axis_in.normalize();
    double angle_in = M_PI / 3;

    SO3 R(axis_in, angle_in);

    Eigen::Vector3d axis_out;
    double angle_out;
    R.axisAngle(axis_out, angle_out);

    REQUIRE(axis_out.isApprox(axis_in, 1e-10));
    REQUIRE(std::abs(angle_out - angle_in) < 1e-10);
}

TEST_CASE("SO3: Group composition", "[SO3]") {
    SO3 R1 = SO3::RotZ(M_PI / 4);
    SO3 R2 = SO3::RotZ(M_PI / 4);

    SO3 R = R1 * R2;
    SO3 expected = SO3::RotZ(M_PI / 2);

    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("SO3: Rotate vector", "[SO3]") {
    SO3 R = SO3::RotZ(M_PI / 2);
    Eigen::Vector3d v(1, 0, 0);

    Eigen::Vector3d v_rotated = R * v;
    Eigen::Vector3d expected(0, 1, 0);

    REQUIRE(v_rotated.isApprox(expected, 1e-10));
}

TEST_CASE("SO3: Group inverse", "[SO3]") {
    SO3 R = SO3::RotZ(M_PI / 3);
    SO3 R_inv = R.inverse();

    // R * R^-1 = I
    SO3 I = R * R_inv;
    REQUIRE(I.isApprox(SO3::Identity(), 1e-10));
}

TEST_CASE("SO3: Exponential map (identity)", "[SO3]") {
    so3 omega_zero = so3::Zero();

    SO3 R = exp_so3(omega_zero);

    REQUIRE(R.isApprox(SO3::Identity()));
}

TEST_CASE("SO3: Exponential map (90° around Z)", "[SO3]") {
    so3 omega(Eigen::Vector3d(0, 0, M_PI / 2));

    SO3 R = exp_so3(omega);
    SO3 expected = SO3::RotZ(M_PI / 2);

    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("SO3: Exponential map (general rotation)", "[SO3]") {
    Eigen::Vector3d axis(1, 1, 1);
    axis.normalize();
    double angle = M_PI / 3;
    Eigen::Vector3d omega_vec = axis * angle;
    so3 omega(omega_vec);

    SO3 R = exp_so3(omega);
    SO3 expected(axis, angle);

    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("SO3: Logarithm map (identity)", "[SO3]") {
    SO3 R = SO3::Identity();

    so3 omega = log_SO3(R);

    REQUIRE(omega.vector().isApprox(Eigen::Vector3d::Zero(), 1e-10));
}

TEST_CASE("SO3: Logarithm map (90° around Z)", "[SO3]") {
    SO3 R = SO3::RotZ(M_PI / 2);

    so3 omega = log_SO3(R);
    Eigen::Vector3d expected(0, 0, M_PI / 2);

    REQUIRE(omega.vector().isApprox(expected, 1e-10));
}

TEST_CASE("SO3: Exponential-Logarithm round-trip", "[SO3]") {
    Eigen::Vector3d axis(1, 2, 3);
    axis.normalize();
    double angle = M_PI / 4;
    SO3 R_in(axis, angle);

    so3 omega = log_SO3(R_in);
    SO3 R_out = exp_so3(omega);

    REQUIRE(R_out.isApprox(R_in, 1e-10));
}

TEST_CASE("SO3: Logarithm-Exponential round-trip", "[SO3]") {
    so3 omega_in(Eigen::Vector3d(0.5, 0.3, 0.2));

    SO3 R = exp_so3(omega_in);
    so3 omega_out = log_SO3(R);

    REQUIRE(omega_out.isApprox(omega_in, 1e-10));
}

TEST_CASE("SO3: Exponential map near-zero", "[SO3]") {
    so3 omega(Eigen::Vector3d(1e-12, 1e-12, 1e-12));

    SO3 R = exp_so3(omega);

    REQUIRE(R.isApprox(SO3::Identity(), 1e-9));
}

TEST_CASE("SO3: isApprox", "[SO3]") {
    SO3 R1 = SO3::RotZ(M_PI / 4);
    SO3 R2 = SO3::RotZ(M_PI / 4 + 1e-11);

    REQUIRE(R1.isApprox(R2, 1e-9));
    REQUIRE_FALSE(R1.isApprox(R2, 1e-12));
}

TEST_CASE("SO3: Non-commutative composition", "[SO3]") {
    SO3 Rx = SO3::RotX(M_PI / 4);
    SO3 Ry = SO3::RotY(M_PI / 4);

    SO3 R1 = Rx * Ry;
    SO3 R2 = Ry * Rx;

    REQUIRE_FALSE(R1.isApprox(R2, 1e-6));
}

TEST_CASE("SO3: Exponential of Lie bracket", "[SO3]") {
    // For small angles, exp([ω1, ω2]) ≈ exp(ω1) exp(ω2) exp(-ω1) exp(-ω2)
    // This is the Baker-Campbell-Hausdorff formula (first order)
    Eigen::Vector3d v1(0.1, 0, 0);
    Eigen::Vector3d v2(0, 0.1, 0);

    so3 omega1(v1);
    so3 omega2(v2);
    so3 commutator = omega1.lieBracket(omega2);

    SO3 R_commutator = exp_so3(commutator);

    SO3 R1 = exp_so3(omega1);
    SO3 R2 = exp_so3(omega2);
    SO3 R1_inv = R1.inverse();
    SO3 R2_inv = R2.inverse();

    SO3 R_bch = R1 * R2 * R1_inv * R2_inv;

    // Should be approximately equal for small angles
    REQUIRE(R_commutator.isApprox(R_bch, 1e-3));
}

TEST_CASE("SO3: 180° rotation", "[SO3]") {
    SO3 R = SO3::RotZ(M_PI);
    Eigen::Vector3d v(1, 0, 0);

    Eigen::Vector3d v_rotated = R * v;
    Eigen::Vector3d expected(-1, 0, 0);

    REQUIRE(v_rotated.isApprox(expected, 1e-10));
}

// Tests for new convenience methods

TEST_CASE("SO3: FromRPY", "[SO3]") {
    SO3 R = SO3::FromRPY(0, 0, M_PI / 2);  // 90° yaw
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rot = R * v;

    REQUIRE(v_rot.isApprox(Eigen::Vector3d(0, 1, 0), 1e-10));
}

TEST_CASE("SO3: FromEuler XYZ", "[SO3]") {
    SO3 R = SO3::FromEuler(M_PI / 2, 0, 0, EulerConvention::XYZ);  // 90° roll
    SO3 Rx = SO3::RotX(M_PI / 2);

    REQUIRE(R.isApprox(Rx, 1e-10));
}

TEST_CASE("SO3: FromQuaternion", "[SO3]") {
    Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    SO3 R = SO3::FromQuaternion(q);
    SO3 expected = SO3::RotZ(M_PI / 2);

    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("SO3: FromRotationVector", "[SO3]") {
    Eigen::Vector3d rotvec(0, 0, M_PI / 2);
    SO3 R = SO3::FromRotationVector(rotvec);
    SO3 expected = SO3::RotZ(M_PI / 2);

    REQUIRE(R.isApprox(expected, 1e-10));
}

TEST_CASE("SO3: quaternion accessor", "[SO3]") {
    SO3 R = SO3::RotZ(M_PI / 2);
    Eigen::Quaterniond q = R.quaternion();

    // Convert back and verify
    SO3 R2 = SO3::FromQuaternion(q);
    REQUIRE(R.isApprox(R2, 1e-10));
}

TEST_CASE("SO3: rpy accessor", "[SO3]") {
    double roll = 0.1, pitch = 0.2, yaw = 0.3;
    SO3 R = SO3::FromRPY(roll, pitch, yaw);
    Eigen::Vector3d rpy = R.rpy();

    REQUIRE(std::abs(rpy(0) - roll) < 1e-10);
    REQUIRE(std::abs(rpy(1) - pitch) < 1e-10);
    REQUIRE(std::abs(rpy(2) - yaw) < 1e-10);
}

TEST_CASE("SO3: rotationVector accessor", "[SO3]") {
    Eigen::Vector3d axis = Eigen::Vector3d(1, 2, 3).normalized();
    double angle = 0.5;
    SO3 R = SO3::FromAxisAngle(axis, angle);

    Eigen::Vector3d rotvec = R.rotationVector();

    REQUIRE(rotvec.isApprox(axis * angle, 1e-10));
}
