#include <catch2/catch_test_macros.hpp>
#include <robospace/math/se3_algebra.hpp>

using namespace robospace::math;

TEST_CASE("se3: Zero construction", "[se3]") {
    se3 xi;

    REQUIRE(xi.omega().isApprox(Eigen::Vector3d::Zero()));
    REQUIRE(xi.v().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("se3: Zero factory", "[se3]") {
    se3 xi = se3::Zero();

    REQUIRE(xi.omega().isApprox(Eigen::Vector3d::Zero()));
    REQUIRE(xi.v().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("se3: Construction from omega and v", "[se3]") {
    Eigen::Vector3d omega(1, 2, 3);
    Eigen::Vector3d v(4, 5, 6);

    se3 xi(omega, v);

    REQUIRE(xi.omega().isApprox(omega));
    REQUIRE(xi.v().isApprox(v));
}

TEST_CASE("se3: Construction from 6D vector", "[se3]") {
    Vector6d vec;
    vec << 1, 2, 3, 4, 5, 6;  // [ω; v]

    se3 xi(vec);

    REQUIRE(xi.omega().isApprox(Eigen::Vector3d(1, 2, 3)));
    REQUIRE(xi.v().isApprox(Eigen::Vector3d(4, 5, 6)));
}

TEST_CASE("se3: Construction from 4×4 bracket matrix", "[se3]") {
    Eigen::Matrix4d bracket = Eigen::Matrix4d::Zero();
    // [ω]× in top-left 3×3
    bracket(0, 1) = -3; bracket(0, 2) = 2;
    bracket(1, 0) = 3;  bracket(1, 2) = -1;
    bracket(2, 0) = -2; bracket(2, 1) = 1;
    // v in top-right 3×1
    bracket(0, 3) = 4;
    bracket(1, 3) = 5;
    bracket(2, 3) = 6;

    se3 xi(bracket);

    REQUIRE(xi.omega().isApprox(Eigen::Vector3d(1, 2, 3)));
    REQUIRE(xi.v().isApprox(Eigen::Vector3d(4, 5, 6)));
}

TEST_CASE("se3: Vector accessor", "[se3]") {
    Eigen::Vector3d omega(1, 2, 3);
    Eigen::Vector3d v(4, 5, 6);
    se3 xi(omega, v);

    Vector6d vec = xi.vector();

    REQUIRE(vec.head<3>().isApprox(omega));
    REQUIRE(vec.tail<3>().isApprox(v));
}

TEST_CASE("se3: Bracket matrix generation", "[se3]") {
    Eigen::Vector3d omega(1, 2, 3);
    Eigen::Vector3d v(4, 5, 6);
    se3 xi(omega, v);

    Eigen::Matrix4d bracket = xi.bracket();

    // Check structure
    REQUIRE(bracket(3, 0) == 0);
    REQUIRE(bracket(3, 1) == 0);
    REQUIRE(bracket(3, 2) == 0);
    REQUIRE(bracket(3, 3) == 0);

    // Check omega part (skew-symmetric)
    Eigen::Matrix3d omega_skew = bracket.block<3, 3>(0, 0);
    REQUIRE(omega_skew.isApprox(-omega_skew.transpose()));

    // Check v part
    REQUIRE(bracket.block<3, 1>(0, 3).isApprox(v));
}

TEST_CASE("se3: Lie bracket", "[se3]") {
    se3 xi1(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0));
    se3 xi2(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0));

    se3 bracket = xi1.lieBracket(xi2);

    // For pure rotations: [ξ₁, ξ₂] = [ω₁ × ω₂; ω₁ × v₂ - ω₂ × v₁]
    REQUIRE(bracket.omega().isApprox(Eigen::Vector3d(0, 0, 1)));
    REQUIRE(bracket.v().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("se3: Lie bracket with translations", "[se3]") {
    se3 xi1(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 0, 0));
    se3 xi2(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 1, 0));

    se3 bracket = xi1.lieBracket(xi2);

    // ω₁ × ω₂ = X × Y = Z
    REQUIRE(bracket.omega().isApprox(Eigen::Vector3d(0, 0, 1)));

    // ω₁ × v₂ - ω₂ × v₁ = X × Y - Y × X = 2(X × Y) = 2Z
    Eigen::Vector3d expected_v(0, 0, 2);
    REQUIRE(bracket.v().isApprox(expected_v));
}

TEST_CASE("se3: Lie bracket is anti-symmetric", "[se3]") {
    se3 xi1(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
    se3 xi2(Eigen::Vector3d(7, 8, 9), Eigen::Vector3d(10, 11, 12));

    se3 bracket12 = xi1.lieBracket(xi2);
    se3 bracket21 = xi2.lieBracket(xi1);

    REQUIRE(bracket12.omega().isApprox(-bracket21.omega()));
    REQUIRE(bracket12.v().isApprox(-bracket21.v()));
}

TEST_CASE("se3: Addition", "[se3]") {
    se3 xi1(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
    se3 xi2(Eigen::Vector3d(7, 8, 9), Eigen::Vector3d(10, 11, 12));

    se3 result = xi1 + xi2;

    REQUIRE(result.omega().isApprox(Eigen::Vector3d(8, 10, 12)));
    REQUIRE(result.v().isApprox(Eigen::Vector3d(14, 16, 18)));
}

TEST_CASE("se3: Subtraction", "[se3]") {
    se3 xi1(Eigen::Vector3d(7, 8, 9), Eigen::Vector3d(10, 11, 12));
    se3 xi2(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));

    se3 result = xi1 - xi2;

    REQUIRE(result.omega().isApprox(Eigen::Vector3d(6, 6, 6)));
    REQUIRE(result.v().isApprox(Eigen::Vector3d(6, 6, 6)));
}

TEST_CASE("se3: Scalar multiplication", "[se3]") {
    se3 xi(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));

    se3 result = xi * 2.0;

    REQUIRE(result.omega().isApprox(Eigen::Vector3d(2, 4, 6)));
    REQUIRE(result.v().isApprox(Eigen::Vector3d(8, 10, 12)));
}

TEST_CASE("se3: Scalar division", "[se3]") {
    se3 xi(Eigen::Vector3d(2, 4, 6), Eigen::Vector3d(8, 10, 12));

    se3 result = xi / 2.0;

    REQUIRE(result.omega().isApprox(Eigen::Vector3d(1, 2, 3)));
    REQUIRE(result.v().isApprox(Eigen::Vector3d(4, 5, 6)));
}

TEST_CASE("se3: Negation", "[se3]") {
    se3 xi(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));

    se3 result = -xi;

    REQUIRE(result.omega().isApprox(Eigen::Vector3d(-1, -2, -3)));
    REQUIRE(result.v().isApprox(Eigen::Vector3d(-4, -5, -6)));
}

TEST_CASE("se3: isApprox", "[se3]") {
    se3 xi1(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));
    se3 xi2(Eigen::Vector3d(1 + 1e-11, 2, 3), Eigen::Vector3d(4, 5, 6));

    REQUIRE(xi1.isApprox(xi2, 1e-9));
    REQUIRE_FALSE(xi1.isApprox(xi2, 1e-12));
}

TEST_CASE("se3: Self Lie bracket is zero", "[se3]") {
    se3 xi(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6));

    se3 result = xi.lieBracket(xi);

    REQUIRE(result.omega().isApprox(Eigen::Vector3d::Zero()));
    REQUIRE(result.v().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("se3: Pure translation (zero omega)", "[se3]") {
    se3 xi(Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 2, 3));

    REQUIRE(xi.omega().norm() < 1e-10);
    REQUIRE(xi.v().isApprox(Eigen::Vector3d(1, 2, 3)));
}

TEST_CASE("se3: Pure rotation (zero v)", "[se3]") {
    se3 xi(Eigen::Vector3d(1, 2, 3), Eigen::Vector3d::Zero());

    REQUIRE(xi.omega().isApprox(Eigen::Vector3d(1, 2, 3)));
    REQUIRE(xi.v().norm() < 1e-10);
}
