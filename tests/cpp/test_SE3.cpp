#include <catch2/catch_test_macros.hpp>
#include <robospace/math/SE3.hpp>

using namespace robospace::math;

TEST_CASE("SE3: Identity construction", "[SE3]") {
    SE3 g;

    REQUIRE(g.matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST_CASE("SE3: Identity factory", "[SE3]") {
    SE3 g = SE3::Identity();

    REQUIRE(g.matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST_CASE("SE3: Construction from R and p", "[SE3]") {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p(1, 2, 3);

    SE3 g(R, p);

    REQUIRE(g.rotation().isApprox(R));
    REQUIRE(g.translation().isApprox(p));
}

TEST_CASE("SE3: Construction from 4×4 matrix", "[SE3]") {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 1>(0, 3) = Eigen::Vector3d(1, 2, 3);

    SE3 g(mat);

    REQUIRE(g.matrix().isApprox(mat));
}

TEST_CASE("SE3: Translation factory", "[SE3]") {
    Eigen::Vector3d p(1, 2, 3);
    SE3 g = SE3::Translation(p);

    REQUIRE(g.rotation().isApprox(Eigen::Matrix3d::Identity()));
    REQUIRE(g.translation().isApprox(p));
}

TEST_CASE("SE3: Decompose", "[SE3]") {
    Eigen::Matrix3d R_in;
    R_in << 0, -1, 0,
            1,  0, 0,
            0,  0, 1;
    Eigen::Vector3d p_in(1, 2, 3);

    SE3 g(R_in, p_in);

    Eigen::Matrix3d R_out;
    Eigen::Vector3d p_out;
    g.decompose(R_out, p_out);

    REQUIRE(R_out.isApprox(R_in));
    REQUIRE(p_out.isApprox(p_in));
}

TEST_CASE("SE3: Group composition", "[SE3]") {
    SE3 g1 = SE3::Translation(Eigen::Vector3d(1, 0, 0));
    SE3 g2 = SE3::Translation(Eigen::Vector3d(2, 0, 0));

    SE3 g = g1 * g2;

    REQUIRE(g.translation().x() == 3.0);
}

TEST_CASE("SE3: Transform point", "[SE3]") {
    SE3 g = SE3::Translation(Eigen::Vector3d(1, 2, 3));
    Eigen::Vector3d p(1, 1, 1);

    Eigen::Vector3d p_transformed = g * p;

    REQUIRE(p_transformed.isApprox(Eigen::Vector3d(2, 3, 4)));
}

TEST_CASE("SE3: Group inverse (pure translation)", "[SE3]") {
    SE3 g = SE3::Translation(Eigen::Vector3d(1, 2, 3));
    SE3 g_inv = g.inverse();

    SE3 identity = g * g_inv;
    REQUIRE(identity.isApprox(SE3::Identity(), 1e-10));
}

TEST_CASE("SE3: Group inverse (rotation + translation)", "[SE3]") {
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;  // 90° around Z
    SE3 g(R, Eigen::Vector3d(1, 2, 3));

    SE3 g_inv = g.inverse();
    SE3 identity = g * g_inv;

    REQUIRE(identity.isApprox(SE3::Identity(), 1e-10));
}

TEST_CASE("SE3: Exponential map (pure translation)", "[SE3]") {
    se3 xi(Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 2, 3));

    SE3 g = exp_se3(xi);

    REQUIRE(g.rotation().isApprox(Eigen::Matrix3d::Identity()));
    REQUIRE(g.translation().isApprox(Eigen::Vector3d(1, 2, 3)));
}

TEST_CASE("SE3: Exponential map (pure rotation)", "[SE3]") {
    // 90° around Z
    se3 xi(Eigen::Vector3d(0, 0, M_PI/2), Eigen::Vector3d::Zero());

    SE3 g = exp_se3(xi);

    Eigen::Matrix3d expected_R;
    expected_R << 0, -1, 0,
                  1,  0, 0,
                  0,  0, 1;

    REQUIRE(g.rotation().isApprox(expected_R, 1e-10));
    REQUIRE(g.translation().norm() < 1e-10);
}

TEST_CASE("SE3: Exponential map (screw motion)", "[SE3]") {
    // Screw motion: rotation around Z + translation along Z
    double angle = M_PI / 4;
    Eigen::Vector3d omega(0, 0, angle);
    Eigen::Vector3d v(0, 0, 1);  // Translation component

    se3 xi(omega, v);
    SE3 g = exp_se3(xi);

    // Should have both rotation and translation
    REQUIRE_FALSE(g.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-6));
    REQUIRE(g.translation().norm() > 0.1);
}

TEST_CASE("SE3: Logarithm map (identity)", "[SE3]") {
    SE3 g = SE3::Identity();

    se3 xi = log_SE3(g);

    REQUIRE(xi.omega().norm() < 1e-10);
    REQUIRE(xi.v().norm() < 1e-10);
}

TEST_CASE("SE3: Logarithm map (pure translation)", "[SE3]") {
    SE3 g = SE3::Translation(Eigen::Vector3d(1, 2, 3));

    se3 xi = log_SE3(g);

    REQUIRE(xi.omega().norm() < 1e-10);
    REQUIRE(xi.v().isApprox(Eigen::Vector3d(1, 2, 3), 1e-10));
}

TEST_CASE("SE3: Exponential-Logarithm round-trip", "[SE3]") {
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
    SE3 g_in(R, Eigen::Vector3d(1, 2, 3));

    se3 xi = log_SE3(g_in);
    SE3 g_out = exp_se3(xi);

    // Use relaxed tolerance due to numerical precision in log map (cot, matrix inverse)
    REQUIRE(g_out.isApprox(g_in, 1e-6));
}

TEST_CASE("SE3: Logarithm-Exponential round-trip", "[SE3]") {
    se3 xi_in(Eigen::Vector3d(0.5, 0.3, 0.2), Eigen::Vector3d(1, 2, 3));

    SE3 g = exp_se3(xi_in);
    se3 xi_out = log_SE3(g);

    // Use relaxed tolerance due to numerical precision in log map
    REQUIRE(xi_out.isApprox(xi_in, 1e-8));
}

TEST_CASE("SE3: Adjoint matrix structure", "[SE3]") {
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
    SE3 g(R, Eigen::Vector3d(1, 2, 3));

    Matrix6d Ad = g.adjoint();

    // Top-left should be R
    REQUIRE(Ad.block<3, 3>(0, 0).isApprox(R));

    // Top-right should be zero
    REQUIRE(Ad.block<3, 3>(0, 3).isApprox(Eigen::Matrix3d::Zero()));

    // Bottom-right should be R
    REQUIRE(Ad.block<3, 3>(3, 3).isApprox(R));
}

TEST_CASE("SE3: Adjoint transformation", "[SE3]") {
    SE3 g = SE3::Translation(Eigen::Vector3d(1, 0, 0));
    se3 xi(Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 0, 0));

    se3 xi_transformed = adjoint_SE3(g, xi);

    // For pure translation, adjoint should preserve the twist
    REQUIRE(xi_transformed.isApprox(xi, 1e-10));
}

TEST_CASE("SE3: Adjoint preserves Lie bracket", "[SE3]") {
    // Ad_g([ξ₁, ξ₂]) = [Ad_g(ξ₁), Ad_g(ξ₂)]
    SE3 g(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 2, 3));
    se3 xi1(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0));
    se3 xi2(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0));

    se3 bracket = xi1.lieBracket(xi2);
    se3 ad_bracket = adjoint_SE3(g, bracket);

    se3 ad_xi1 = adjoint_SE3(g, xi1);
    se3 ad_xi2 = adjoint_SE3(g, xi2);
    se3 bracket_ad = ad_xi1.lieBracket(ad_xi2);

    REQUIRE(ad_bracket.isApprox(bracket_ad, 1e-10));
}

TEST_CASE("SE3: isApprox", "[SE3]") {
    SE3 g1 = SE3::Translation(Eigen::Vector3d(1, 2, 3));
    SE3 g2 = SE3::Translation(Eigen::Vector3d(1 + 1e-11, 2, 3));

    REQUIRE(g1.isApprox(g2, 1e-9));
    REQUIRE_FALSE(g1.isApprox(g2, 1e-12));
}

TEST_CASE("SE3: Composition order matters", "[SE3]") {
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;

    SE3 g_rot(R, Eigen::Vector3d::Zero());
    SE3 g_trans = SE3::Translation(Eigen::Vector3d(1, 0, 0));

    SE3 g1 = g_trans * g_rot;
    SE3 g2 = g_rot * g_trans;

    REQUIRE_FALSE(g1.isApprox(g2, 1e-6));
}

TEST_CASE("SE3: Exponential map (identity from zero twist)", "[SE3]") {
    se3 xi_zero = se3::Zero();

    SE3 g = exp_se3(xi_zero);

    REQUIRE(g.isApprox(SE3::Identity()));
}

TEST_CASE("SE3: Screw motion along X-axis", "[SE3]") {
    // Rotation around X + translation along X (screw)
    double angle = M_PI / 6;
    Eigen::Vector3d omega(angle, 0, 0);
    Eigen::Vector3d v(1, 0, 0);

    se3 xi(omega, v);
    SE3 g = exp_se3(xi);

    // Should have rotation around X
    Eigen::Vector3d axis;
    double extracted_angle;
    Eigen::AngleAxisd aa(g.rotation());
    axis = aa.axis();
    extracted_angle = aa.angle();

    REQUIRE(axis.isApprox(Eigen::Vector3d(1, 0, 0), 1e-6));
    REQUIRE(std::abs(extracted_angle - angle) < 1e-6);

    // Should have translation along X
    REQUIRE(g.translation()(0) > 0.5);  // Some translation along X
}

// Tests for new convenience methods

TEST_CASE("SE3: RotX", "[SE3]") {
    SE3 T = SE3::RotX(M_PI / 2);
    Eigen::Vector3d v(0, 1, 0);
    Eigen::Vector3d result = T * v;

    REQUIRE(result.isApprox(Eigen::Vector3d(0, 0, 1), 1e-10));
    REQUIRE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("SE3: RotY", "[SE3]") {
    SE3 T = SE3::RotY(M_PI / 2);
    Eigen::Vector3d v(0, 0, 1);
    Eigen::Vector3d result = T * v;

    REQUIRE(result.isApprox(Eigen::Vector3d(1, 0, 0), 1e-10));
    REQUIRE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("SE3: RotZ", "[SE3]") {
    SE3 T = SE3::RotZ(M_PI / 2);
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d result = T * v;

    REQUIRE(result.isApprox(Eigen::Vector3d(0, 1, 0), 1e-10));
    REQUIRE(T.translation().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("SE3: FromRPY", "[SE3]") {
    SE3 T = SE3::FromRPY(0, 0, M_PI / 2, Eigen::Vector3d(1, 2, 3));

    REQUIRE(T.translation().isApprox(Eigen::Vector3d(1, 2, 3)));

    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d result = T.so3() * v;
    REQUIRE(result.isApprox(Eigen::Vector3d(0, 1, 0), 1e-10));
}

TEST_CASE("SE3: FromAxisAngle", "[SE3]") {
    SE3 T = SE3::FromAxisAngle(Eigen::Vector3d::UnitZ(), M_PI / 2, Eigen::Vector3d(1, 0, 0));

    REQUIRE(T.translation().isApprox(Eigen::Vector3d(1, 0, 0)));
    REQUIRE(T.so3().isApprox(SO3::RotZ(M_PI / 2), 1e-10));
}

TEST_CASE("SE3: FromRotationAndTranslation", "[SE3]") {
    SO3 R = SO3::RotZ(M_PI / 4);
    Eigen::Vector3d p(1, 2, 3);
    SE3 T = SE3::FromRotationAndTranslation(R, p);

    REQUIRE(T.so3().isApprox(R, 1e-10));
    REQUIRE(T.translation().isApprox(p));
}

TEST_CASE("SE3: so3 accessor", "[SE3]") {
    SE3 T = SE3::RotZ(M_PI / 3);
    SO3 R = T.so3();

    REQUIRE(R.isApprox(SO3::RotZ(M_PI / 3), 1e-10));
}

TEST_CASE("SE3: rpy accessor", "[SE3]") {
    double roll = 0.1, pitch = 0.2, yaw = 0.3;
    SE3 T = SE3::FromRPY(roll, pitch, yaw);
    Eigen::Vector3d rpy = T.rpy();

    REQUIRE(std::abs(rpy(0) - roll) < 1e-10);
    REQUIRE(std::abs(rpy(1) - pitch) < 1e-10);
    REQUIRE(std::abs(rpy(2) - yaw) < 1e-10);
}
