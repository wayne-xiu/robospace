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
// Convention: rows 0-2 = angular (ω), rows 3-5 = linear (v)
Eigen::MatrixXd analytical_jacob0_2r(double q1, double q2) {
    const double L1 = 1.0;
    const double L2 = 0.8;

    Eigen::MatrixXd J(6, 2);
    J.setZero();

    double c1 = std::cos(q1);
    double s1 = std::sin(q1);
    double c12 = std::cos(q1 + q2);
    double s12 = std::sin(q1 + q2);

    // Angular velocity (Z-axis for planar robot)
    J(2, 0) = 1.0;
    J(2, 1) = 1.0;

    // Linear velocity (X, Y)
    J(3, 0) = -L1 * s1 - L2 * s12;
    J(3, 1) = -L2 * s12;
    J(4, 0) = L1 * c1 + L2 * c12;
    J(4, 1) = L2 * c12;

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

    // Verify dimensions
    REQUIRE(Je.rows() == 6);
    REQUIRE(Je.cols() == 2);

    // Verify angular parts are same (R = I at zero config)
    REQUIRE(Je.block<3, 2>(0, 0).isApprox(J0.block<3, 2>(0, 0), 1e-10));

    // Linear parts differ due to [p]× term in adjoint (p = [1.8, 0, 0] for 2R)
    // Je = Ad_{T^-1} * J0 where Ad has [p]× in bottom-left block
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
    REQUIRE(std::abs(J(2, 0)) > 0.5);

    // Column 2: prismatic - should have zero angular velocity
    REQUIRE_THAT(J(0, 1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(J(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(J(2, 1), Catch::Matchers::WithinAbs(0.0, 1e-10));

    // Column 3: revolute - should have non-zero angular velocity component
    double omega_norm = std::sqrt(J(0, 2)*J(0, 2) + J(1, 2)*J(1, 2) + J(2, 2)*J(2, 2));
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

TEST_CASE("Jacobian: Non-Z axis joint (URDF-style)", "[jacobian][urdf]") {
    // Create a robot with a Y-axis revolute joint
    Robot robot("Y_axis_robot");
    robot.add_link(Link("base"));
    robot.add_link(Link("link1"));

    // Joint rotating about Y-axis (not Z)
    Joint joint1("joint1", JointType::REVOLUTE, 0, 1);
    joint1.set_origin(SE3::Translation(Eigen::Vector3d(0, 0, 0.5)));
    joint1.set_axis(Eigen::Vector3d::UnitY());  // Y-axis rotation
    robot.add_joint(joint1);

    Eigen::VectorXd q(1);
    q << M_PI / 4;  // 45 degrees

    Eigen::MatrixXd J = robot.jacob0(q);

    REQUIRE(J.rows() == 6);
    REQUIRE(J.cols() == 1);

    // Angular velocity should be along Y-axis in base frame
    // R(Y, q) * [0,1,0] = [0,1,0] (Y-axis is invariant under Y rotations)
    REQUIRE_THAT(J(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(J(1, 0), Catch::Matchers::WithinAbs(1.0, 1e-10));
    REQUIRE_THAT(J(2, 0), Catch::Matchers::WithinAbs(0.0, 1e-10));

    // Verify this is different from Z-axis default (would be [0,0,1])
    // This proves we're using joint.axis() and not hardcoded col(2)
}
