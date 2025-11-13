#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/units.hpp>
#include <robospace/math/SO3.hpp>
#include <cmath>

using namespace robospace::units;
using namespace robospace::math;

TEST_CASE("Units: deg_to_rad scalar", "[units]") {
    REQUIRE_THAT(deg_to_rad(0.0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(deg_to_rad(90.0), Catch::Matchers::WithinAbs(PI/2, 1e-10));
    REQUIRE_THAT(deg_to_rad(180.0), Catch::Matchers::WithinAbs(PI, 1e-10));
    REQUIRE_THAT(deg_to_rad(360.0), Catch::Matchers::WithinAbs(2*PI, 1e-10));
    REQUIRE_THAT(deg_to_rad(-90.0), Catch::Matchers::WithinAbs(-PI/2, 1e-10));
}

TEST_CASE("Units: rad_to_deg scalar", "[units]") {
    REQUIRE_THAT(rad_to_deg(0.0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(rad_to_deg(PI/2), Catch::Matchers::WithinAbs(90.0, 1e-10));
    REQUIRE_THAT(rad_to_deg(PI), Catch::Matchers::WithinAbs(180.0, 1e-10));
    REQUIRE_THAT(rad_to_deg(2*PI), Catch::Matchers::WithinAbs(360.0, 1e-10));
    REQUIRE_THAT(rad_to_deg(-PI/2), Catch::Matchers::WithinAbs(-90.0, 1e-10));
}

TEST_CASE("Units: deg_to_rad vector", "[units]") {
    Eigen::VectorXd deg(3);
    deg << 0.0, 90.0, 180.0;

    Eigen::VectorXd rad = deg_to_rad(deg);

    REQUIRE(rad.size() == 3);
    REQUIRE_THAT(rad(0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(rad(1), Catch::Matchers::WithinAbs(PI/2, 1e-10));
    REQUIRE_THAT(rad(2), Catch::Matchers::WithinAbs(PI, 1e-10));
}

TEST_CASE("Units: rad_to_deg vector", "[units]") {
    Eigen::VectorXd rad(3);
    rad << 0.0, PI/2, PI;

    Eigen::VectorXd deg = rad_to_deg(rad);

    REQUIRE(deg.size() == 3);
    REQUIRE_THAT(deg(0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(deg(1), Catch::Matchers::WithinAbs(90.0, 1e-10));
    REQUIRE_THAT(deg(2), Catch::Matchers::WithinAbs(180.0, 1e-10));
}

TEST_CASE("Units: round-trip deg/rad scalar", "[units]") {
    double original = 45.0;
    double converted = rad_to_deg(deg_to_rad(original));
    REQUIRE_THAT(converted, Catch::Matchers::WithinAbs(original, 1e-10));
}

TEST_CASE("Units: round-trip deg/rad vector", "[units]") {
    Eigen::VectorXd original(4);
    original << 0.0, 45.0, 90.0, 180.0;

    Eigen::VectorXd converted = rad_to_deg(deg_to_rad(original));

    REQUIRE(converted.size() == original.size());
    for (int i = 0; i < original.size(); ++i) {
        REQUIRE_THAT(converted(i), Catch::Matchers::WithinAbs(original(i), 1e-10));
    }
}

TEST_CASE("Units: mm_to_m scalar", "[units]") {
    REQUIRE_THAT(mm_to_m(0.0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(mm_to_m(1000.0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(mm_to_m(500.0), Catch::Matchers::WithinAbs(0.5, 1e-10));
    REQUIRE_THAT(mm_to_m(-250.0), Catch::Matchers::WithinAbs(-0.25, 1e-10));
}

TEST_CASE("Units: m_to_mm scalar", "[units]") {
    REQUIRE_THAT(m_to_mm(0.0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(m_to_mm(1.0), Catch::Matchers::WithinAbs(1000.0, 1e-10));
    REQUIRE_THAT(m_to_mm(0.5), Catch::Matchers::WithinAbs(500.0, 1e-10));
    REQUIRE_THAT(m_to_mm(-0.25), Catch::Matchers::WithinAbs(-250.0, 1e-10));
}

TEST_CASE("Units: mm_to_m Vector3d", "[units]") {
    Eigen::Vector3d mm(1000.0, 500.0, 250.0);
    Eigen::Vector3d m = mm_to_m(mm);

    REQUIRE_THAT(m(0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(m(1), Catch::Matchers::WithinAbs(0.5, 1e-10));
    REQUIRE_THAT(m(2), Catch::Matchers::WithinAbs(0.25, 1e-10));
}

TEST_CASE("Units: m_to_mm Vector3d", "[units]") {
    Eigen::Vector3d m(1.0, 0.5, 0.25);
    Eigen::Vector3d mm = m_to_mm(m);

    REQUIRE_THAT(mm(0), Catch::Matchers::WithinAbs(1000.0, 1e-10));
    REQUIRE_THAT(mm(1), Catch::Matchers::WithinAbs(500.0, 1e-10));
    REQUIRE_THAT(mm(2), Catch::Matchers::WithinAbs(250.0, 1e-10));
}

TEST_CASE("Units: round-trip mm/m scalar", "[units]") {
    double original = 525.75;
    double converted = m_to_mm(mm_to_m(original));
    REQUIRE_THAT(converted, Catch::Matchers::WithinAbs(original, 1e-10));
}

TEST_CASE("Units: round-trip mm/m Vector3d", "[units]") {
    Eigen::Vector3d original(123.456, 789.012, -345.678);
    Eigen::Vector3d converted = m_to_mm(mm_to_m(original));

    REQUIRE_THAT(converted(0), Catch::Matchers::WithinAbs(original(0), 1e-6));
    REQUIRE_THAT(converted(1), Catch::Matchers::WithinAbs(original(1), 1e-6));
    REQUIRE_THAT(converted(2), Catch::Matchers::WithinAbs(original(2), 1e-6));
}

TEST_CASE("Units: inch_to_m", "[units]") {
    REQUIRE_THAT(inch_to_m(0.0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(inch_to_m(1.0), Catch::Matchers::WithinAbs(0.0254, 1e-10));
    REQUIRE_THAT(inch_to_m(10.0), Catch::Matchers::WithinAbs(0.254, 1e-10));
}

TEST_CASE("Units: m_to_inch", "[units]") {
    REQUIRE_THAT(m_to_inch(0.0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(m_to_inch(0.0254), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(m_to_inch(0.254), Catch::Matchers::WithinAbs(10.0, 1e-10));
}

TEST_CASE("Units: SE3 mm_to_m", "[units]") {
    SE3 T_mm = SE3::Translation(Eigen::Vector3d(1000.0, 500.0, 250.0));
    SE3 T_m = mm_to_m(T_mm);

    REQUIRE_THAT(T_m.translation()(0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(T_m.translation()(1), Catch::Matchers::WithinAbs(0.5, 1e-10));
    REQUIRE_THAT(T_m.translation()(2), Catch::Matchers::WithinAbs(0.25, 1e-10));

    REQUIRE(T_m.rotation().isApprox(T_mm.rotation(), 1e-10));
}

TEST_CASE("Units: SE3 m_to_mm", "[units]") {
    SE3 T_m = SE3::Translation(Eigen::Vector3d(1.0, 0.5, 0.25));
    SE3 T_mm = m_to_mm(T_m);

    REQUIRE_THAT(T_mm.translation()(0), Catch::Matchers::WithinAbs(1000.0, 1e-10));
    REQUIRE_THAT(T_mm.translation()(1), Catch::Matchers::WithinAbs(500.0, 1e-10));
    REQUIRE_THAT(T_mm.translation()(2), Catch::Matchers::WithinAbs(250.0, 1e-10));

    REQUIRE(T_mm.rotation().isApprox(T_m.rotation(), 1e-10));
}

TEST_CASE("Units: SE3 conversion preserves rotation", "[units]") {
    SO3 R = SO3::RotZ(PI/4);
    SE3 T_m(R.matrix(), Eigen::Vector3d(1.0, 2.0, 3.0));
    SE3 T_mm = m_to_mm(T_m);
    SE3 T_back = mm_to_m(T_mm);

    REQUIRE(T_back.isApprox(T_m, 1e-10));
}

TEST_CASE("Units: Industrial workflow example", "[units][integration]") {
    // User inputs in mm, deg (industrial standard)
    Eigen::VectorXd q_deg(6);
    q_deg << 0.0, 90.0, -90.0, 0.0, 90.0, 0.0;

    // Convert to SI for robot API
    Eigen::VectorXd q_rad = deg_to_rad(q_deg);

    // Create simple robot
    Eigen::VectorXd q_back_deg = rad_to_deg(q_rad);

    for (int i = 0; i < 6; ++i) {
        REQUIRE_THAT(q_back_deg(i), Catch::Matchers::WithinAbs(q_deg(i), 1e-10));
    }

    // Position conversion
    Eigen::Vector3d pos_mm(500.0, 300.0, 200.0);
    Eigen::Vector3d pos_m = mm_to_m(pos_mm);
    Eigen::Vector3d pos_back_mm = m_to_mm(pos_m);

    for (int i = 0; i < 3; ++i) {
        REQUIRE_THAT(pos_back_mm(i), Catch::Matchers::WithinAbs(pos_mm(i), 1e-10));
    }
}
