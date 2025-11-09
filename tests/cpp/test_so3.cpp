#include <catch2/catch_test_macros.hpp>
#include <robospace/math/so3.hpp>

using namespace robospace::math;

TEST_CASE("so3: Zero construction", "[so3]") {
    so3 omega;

    REQUIRE(omega.vector().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("so3: Zero factory", "[so3]") {
    so3 omega = so3::Zero();

    REQUIRE(omega.vector().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("so3: Construction from vector", "[so3]") {
    Eigen::Vector3d v(1, 2, 3);
    so3 omega(v);

    REQUIRE(omega.vector().isApprox(v));
}

TEST_CASE("so3: Construction from skew-symmetric matrix", "[so3]") {
    Eigen::Matrix3d S;
    S <<  0, -3,  2,
          3,  0, -1,
         -2,  1,  0;  // [1,2,3]×

    so3 omega(S);
    Eigen::Vector3d expected(1, 2, 3);

    REQUIRE(omega.vector().isApprox(expected));
}

TEST_CASE("so3: Bracket (skew-symmetric) generation", "[so3]") {
    Eigen::Vector3d v(1, 2, 3);
    so3 omega(v);

    Eigen::Matrix3d bracket = omega.bracket();
    Eigen::Matrix3d expected;
    expected <<  0, -3,  2,
                 3,  0, -1,
                -2,  1,  0;

    REQUIRE(bracket.isApprox(expected));
}

TEST_CASE("so3: Bracket is skew-symmetric", "[so3]") {
    Eigen::Vector3d v(1, 2, 3);
    so3 omega(v);

    Eigen::Matrix3d bracket = omega.bracket();

    REQUIRE(bracket.isApprox(-bracket.transpose()));
}

TEST_CASE("so3: Lie bracket (cross product)", "[so3]") {
    so3 omega1(Eigen::Vector3d(1, 0, 0));
    so3 omega2(Eigen::Vector3d(0, 1, 0));

    so3 result = omega1.lieBracket(omega2);
    Eigen::Vector3d expected(0, 0, 1);  // X × Y = Z

    REQUIRE(result.vector().isApprox(expected));
}

TEST_CASE("so3: Lie bracket properties", "[so3]") {
    so3 omega1(Eigen::Vector3d(1, 2, 3));
    so3 omega2(Eigen::Vector3d(4, 5, 6));

    // Anti-symmetry: [ω1, ω2] = -[ω2, ω1]
    so3 bracket12 = omega1.lieBracket(omega2);
    so3 bracket21 = omega2.lieBracket(omega1);

    REQUIRE(bracket12.vector().isApprox(-bracket21.vector()));
}

TEST_CASE("so3: Addition", "[so3]") {
    so3 omega1(Eigen::Vector3d(1, 2, 3));
    so3 omega2(Eigen::Vector3d(4, 5, 6));

    so3 result = omega1 + omega2;
    Eigen::Vector3d expected(5, 7, 9);

    REQUIRE(result.vector().isApprox(expected));
}

TEST_CASE("so3: Subtraction", "[so3]") {
    so3 omega1(Eigen::Vector3d(4, 5, 6));
    so3 omega2(Eigen::Vector3d(1, 2, 3));

    so3 result = omega1 - omega2;
    Eigen::Vector3d expected(3, 3, 3);

    REQUIRE(result.vector().isApprox(expected));
}

TEST_CASE("so3: Scalar multiplication", "[so3]") {
    so3 omega(Eigen::Vector3d(1, 2, 3));

    so3 result = omega * 2.0;
    Eigen::Vector3d expected(2, 4, 6);

    REQUIRE(result.vector().isApprox(expected));
}

TEST_CASE("so3: Scalar multiplication (commutative)", "[so3]") {
    so3 omega(Eigen::Vector3d(1, 2, 3));

    so3 result1 = omega * 2.0;
    so3 result2 = 2.0 * omega;

    REQUIRE(result1.vector().isApprox(result2.vector()));
}

TEST_CASE("so3: Scalar division", "[so3]") {
    so3 omega(Eigen::Vector3d(2, 4, 6));

    so3 result = omega / 2.0;
    Eigen::Vector3d expected(1, 2, 3);

    REQUIRE(result.vector().isApprox(expected));
}

TEST_CASE("so3: Negation", "[so3]") {
    so3 omega(Eigen::Vector3d(1, 2, 3));

    so3 result = -omega;
    Eigen::Vector3d expected(-1, -2, -3);

    REQUIRE(result.vector().isApprox(expected));
}

TEST_CASE("so3: isApprox", "[so3]") {
    so3 omega1(Eigen::Vector3d(1, 2, 3));
    so3 omega2(Eigen::Vector3d(1 + 1e-11, 2, 3));

    REQUIRE(omega1.isApprox(omega2, 1e-9));
    REQUIRE_FALSE(omega1.isApprox(omega2, 1e-12));
}

TEST_CASE("so3: Zero Lie bracket", "[so3]") {
    so3 omega1(Eigen::Vector3d(1, 2, 3));
    so3 omega_zero = so3::Zero();

    so3 result = omega1.lieBracket(omega_zero);

    REQUIRE(result.vector().isApprox(Eigen::Vector3d::Zero()));
}

TEST_CASE("so3: Self Lie bracket is zero", "[so3]") {
    so3 omega(Eigen::Vector3d(1, 2, 3));

    so3 result = omega.lieBracket(omega);

    REQUIRE(result.vector().isApprox(Eigen::Vector3d::Zero()));
}
