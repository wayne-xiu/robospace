#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/robot.hpp>
#include <robospace/math/SO3.hpp>
#include <cmath>

using namespace robospace::model;
using namespace robospace::math;

// Create 2R planar robot for analytical verification
Robot create_2r_planar() {
    Robot robot("2R_planar");
    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));
    robot.add_link(Link("link2"));

    DHParams dh1(0, 1.0, 0, 0, DHConvention::STANDARD);
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);
    robot.add_joint(joint1);

    DHParams dh2(0, 0.8, 0, 0, DHConvention::STANDARD);
    Joint joint2("joint2", JointType::REVOLUTE, 1, 2);
    joint2.set_dh_params(dh2);
    robot.add_joint(joint2);

    return robot;
}

// Analytical Jacobian for 2R planar robot
Eigen::MatrixXd analytical_jacob0_2r(double q1, double q2) {
    const double L1 = 1.0;
    const double L2 = 0.8;

    Eigen::MatrixXd J(6, 2);
    J.setZero();

    double c1 = std::cos(q1);
    double s1 = std::sin(q1);
    double c12 = std::cos(q1 + q2);
    double s12 = std::sin(q1 + q2);

    J(0, 0) = -L1 * s1 - L2 * s12;
    J(0, 1) = -L2 * s12;
    J(1, 0) = L1 * c1 + L2 * c12;
    J(1, 1) = L2 * c12;
    J(5, 0) = 1.0;
    J(5, 1) = 1.0;

    return J;
}

TEST_CASE("Jacobian 2R: Zero configuration", "[jacobian][2r]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << 0.0, 0.0;

    Eigen::MatrixXd J = robot.jacob0(q);
    Eigen::MatrixXd J_expected = analytical_jacob0_2r(0.0, 0.0);

    REQUIRE(J.rows() == 6);
    REQUIRE(J.cols() == 2);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            REQUIRE_THAT(J(i, j), Catch::Matchers::WithinAbs(J_expected(i, j), 1e-10));
        }
    }
}

TEST_CASE("Jacobian 2R: 90° first joint", "[jacobian][2r]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << M_PI/2, 0.0;

    Eigen::MatrixXd J = robot.jacob0(q);
    Eigen::MatrixXd J_expected = analytical_jacob0_2r(M_PI/2, 0.0);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            REQUIRE_THAT(J(i, j), Catch::Matchers::WithinAbs(J_expected(i, j), 1e-10));
        }
    }
}

TEST_CASE("Jacobian 2R: Arbitrary configuration", "[jacobian][2r]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << 0.5, -0.3;

    Eigen::MatrixXd J = robot.jacob0(q);
    Eigen::MatrixXd J_expected = analytical_jacob0_2r(0.5, -0.3);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            REQUIRE_THAT(J(i, j), Catch::Matchers::WithinAbs(J_expected(i, j), 1e-10));
        }
    }
}

TEST_CASE("Jacobian 2R: Singular configuration (stretched)", "[jacobian][2r][singular]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << 0.0, 0.0;

    Eigen::MatrixXd J = robot.jacob0(q);

    double det = (J.block<3, 2>(0, 0).transpose() * J.block<3, 2>(0, 0)).determinant();
    REQUIRE(std::abs(det) < 1e-6);
}

TEST_CASE("Jacobian: Stateful method uses stored configuration", "[jacobian]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << 0.3, 0.7;
    robot.set_joints(q);

    Eigen::MatrixXd J_stateless = robot.jacob0(q);
    Eigen::MatrixXd J_stateful = robot.jacob0();

    REQUIRE(J_stateless.isApprox(J_stateful, 1e-10));
}

TEST_CASE("Jacobian: Configuration size mismatch throws", "[jacobian][error]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q_wrong(3);
    q_wrong << 0.0, 0.0, 0.0;

    REQUIRE_THROWS_AS(robot.jacob0(q_wrong), std::invalid_argument);
}

TEST_CASE("Jacobian: Configuration not set throws for stateful", "[jacobian][error]") {
    Robot robot = create_2r_planar();

    REQUIRE_THROWS_AS(robot.jacob0(), std::runtime_error);
}

TEST_CASE("Jacobian: jacobe dimensions", "[jacobian][jacobe]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << 0.5, -0.3;

    Eigen::MatrixXd Je = robot.jacobe(q);

    REQUIRE(Je.rows() == 6);
    REQUIRE(Je.cols() == 2);
}

TEST_CASE("Jacobian: jacobe at zero configuration", "[jacobian][jacobe]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << 0.0, 0.0;

    Eigen::MatrixXd J0 = robot.jacob0(q);
    Eigen::MatrixXd Je = robot.jacobe(q);

    // At zero config, rotation is identity, so J0 ≈ Je
    REQUIRE(Je.isApprox(J0, 1e-10));
}

TEST_CASE("Jacobian: Empty robot", "[jacobian][edge]") {
    Robot robot("empty");

    Eigen::VectorXd q(0);
    Eigen::MatrixXd J = robot.jacob0(q);

    REQUIRE(J.rows() == 6);
    REQUIRE(J.cols() == 0);
}

TEST_CASE("Jacobian 3DOF: Mixed revolute-prismatic", "[jacobian][3dof]") {
    Robot robot("RPR");
    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));
    robot.add_link(Link("link2"));
    robot.add_link(Link("link3"));

    // Revolute joint
    DHParams dh1(M_PI/2, 0, 0.5, 0, DHConvention::STANDARD);
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_dh_params(dh1);
    robot.add_joint(joint1);

    // Prismatic joint
    DHParams dh2(0, 0, 0, 0, DHConvention::STANDARD);
    Joint joint2("joint2", JointType::PRISMATIC, 1, 2);
    joint2.set_dh_params(dh2);
    robot.add_joint(joint2);

    // Revolute joint
    DHParams dh3(0, 0.3, 0, 0, DHConvention::STANDARD);
    Joint joint3("joint3", JointType::REVOLUTE, 2, 3);
    joint3.set_dh_params(dh3);
    robot.add_joint(joint3);

    Eigen::VectorXd q(3);
    q << 0.0, 0.2, 0.0;

    Eigen::MatrixXd J = robot.jacob0(q);

    REQUIRE(J.rows() == 6);
    REQUIRE(J.cols() == 3);

    // Column 1: revolute - should have angular velocity
    REQUIRE(std::abs(J(5, 0)) > 0.5);

    // Column 2: prismatic - should have zero angular velocity
    REQUIRE_THAT(J(3, 1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(J(4, 1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(J(5, 1), Catch::Matchers::WithinAbs(0.0, 1e-10));

    // Column 3: revolute - should have non-zero angular velocity component
    double omega_norm = std::sqrt(J(3, 2)*J(3, 2) + J(4, 2)*J(4, 2) + J(5, 2)*J(5, 2));
    REQUIRE(omega_norm > 0.5);
}

TEST_CASE("Jacobian Performance: 2R robot", "[jacobian][performance]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << 0.5, -0.3;

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        Eigen::MatrixXd J = robot.jacob0(q);
        (void)J;
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double avg_time = duration.count() / 10000.0;

    REQUIRE(avg_time < 20.0);
}

TEST_CASE("Jacobian: Verify numerical stability", "[jacobian][stability]") {
    Robot robot = create_2r_planar();

    Eigen::VectorXd q(2);
    q << 1.5707963267948966, 0.0;  // π/2 with high precision

    Eigen::MatrixXd J1 = robot.jacob0(q);

    q(0) = M_PI / 2;  // Same angle, different representation
    Eigen::MatrixXd J2 = robot.jacob0(q);

    REQUIRE(J1.isApprox(J2, 1e-9));
}
