#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/robot.hpp>
#include <robospace/math/SO3.hpp>
#include <robospace/math/SE3.hpp>
#include <cmath>
#include <chrono>

using namespace robospace::model;
using namespace robospace::math;

Robot create_2r_planar_robot() {
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

Eigen::Vector3d analytical_2r_fk(double q1, double q2) {
    const double L1 = 1.0;
    const double L2 = 0.8;
    double x = L1 * std::cos(q1) + L2 * std::cos(q1 + q2);
    double y = L1 * std::sin(q1) + L2 * std::sin(q1 + q2);
    return Eigen::Vector3d(x, y, 0.0);
}

TEST_CASE("FK 2R: Zero configuration", "[fk][2r]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(0.0, 0.0);

    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();
    Eigen::Vector3d pos_expected = analytical_2r_fk(0.0, 0.0);

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(pos_expected(0), 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(pos_expected(1), 1e-10));
    REQUIRE_THAT(pos(2), Catch::Matchers::WithinAbs(pos_expected(2), 1e-10));
    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(1.8, 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("FK 2R: 90° first joint", "[fk][2r]") {
    Robot robot = create_2r_planar_robot();
    double q1 = M_PI / 2.0;
    double q2 = 0.0;
    Eigen::VectorXd q = Eigen::Vector2d(q1, q2);

    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();
    Eigen::Vector3d pos_expected = analytical_2r_fk(q1, q2);

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(pos_expected(0), 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(pos_expected(1), 1e-10));
    REQUIRE_THAT(pos(2), Catch::Matchers::WithinAbs(pos_expected(2), 1e-10));
    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(1.8, 1e-10));
}

TEST_CASE("FK 2R: 90° both joints (folded)", "[fk][2r]") {
    Robot robot = create_2r_planar_robot();
    double q1 = M_PI / 2.0;
    double q2 = M_PI / 2.0;
    Eigen::VectorXd q = Eigen::Vector2d(q1, q2);

    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();
    Eigen::Vector3d pos_expected = analytical_2r_fk(q1, q2);

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(pos_expected(0), 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(pos_expected(1), 1e-10));
    REQUIRE_THAT(pos(2), Catch::Matchers::WithinAbs(pos_expected(2), 1e-10));
    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(-0.8, 1e-9));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(1.0, 1e-9));
}

TEST_CASE("FK 2R: 45° first joint", "[fk][2r]") {
    Robot robot = create_2r_planar_robot();
    double q1 = M_PI / 4.0;
    double q2 = 0.0;
    Eigen::VectorXd q = Eigen::Vector2d(q1, q2);

    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();
    Eigen::Vector3d pos_expected = analytical_2r_fk(q1, q2);

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(pos_expected(0), 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(pos_expected(1), 1e-10));
    REQUIRE_THAT(pos(2), Catch::Matchers::WithinAbs(pos_expected(2), 1e-10));
}

TEST_CASE("FK 2R: Arbitrary configuration", "[fk][2r]") {
    Robot robot = create_2r_planar_robot();
    double q1 = 0.5;
    double q2 = -0.3;
    Eigen::VectorXd q = Eigen::Vector2d(q1, q2);

    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();
    Eigen::Vector3d pos_expected = analytical_2r_fk(q1, q2);

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(pos_expected(0), 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(pos_expected(1), 1e-10));
    REQUIRE_THAT(pos(2), Catch::Matchers::WithinAbs(pos_expected(2), 1e-10));
}

TEST_CASE("FK 2R: Negative angles", "[fk][2r]") {
    Robot robot = create_2r_planar_robot();
    double q1 = -M_PI / 4.0;
    double q2 = -M_PI / 6.0;
    Eigen::VectorXd q = Eigen::Vector2d(q1, q2);

    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();
    Eigen::Vector3d pos_expected = analytical_2r_fk(q1, q2);

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(pos_expected(0), 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(pos_expected(1), 1e-10));
    REQUIRE_THAT(pos(2), Catch::Matchers::WithinAbs(pos_expected(2), 1e-10));
}

TEST_CASE("FK 2R: Full rotation (360°)", "[fk][2r]") {
    Robot robot = create_2r_planar_robot();
    double q1 = 2.0 * M_PI;
    double q2 = 0.0;
    Eigen::VectorXd q = Eigen::Vector2d(q1, q2);

    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(1.8, 1e-9));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("FK API: fk by link name", "[fk][api]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(M_PI / 4.0, 0.0);

    SE3 T_link1 = robot.fk(q, "link1");
    Eigen::Vector3d pos = T_link1.translation();

    double expected_x = 1.0 * std::cos(M_PI / 4.0);
    double expected_y = 1.0 * std::sin(M_PI / 4.0);

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(expected_x, 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(expected_y, 1e-10));
}

TEST_CASE("FK API: fk to base link", "[fk][api]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(0.5, -0.3);

    SE3 T_base = robot.fk(q, "base");
    REQUIRE(T_base.isApprox(SE3::Identity()));
}

TEST_CASE("FK API: fk_all", "[fk][api]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(M_PI / 2.0, 0.0);

    std::vector<SE3> poses = robot.fk_all(q);

    REQUIRE(poses.size() == 3);
    REQUIRE(poses[0].isApprox(SE3::Identity()));

    Eigen::Vector3d pos1 = poses[1].translation();
    REQUIRE_THAT(pos1(0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(pos1(1), Catch::Matchers::WithinAbs(1.0, 1e-10));

    Eigen::Vector3d pos2 = poses[2].translation();
    REQUIRE_THAT(pos2(0), Catch::Matchers::WithinAbs(0.0, 1e-10));
    REQUIRE_THAT(pos2(1), Catch::Matchers::WithinAbs(1.8, 1e-10));
}

TEST_CASE("FK API: fk matches fk_all", "[fk][api]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(0.7, -0.4);

    SE3 T_tcp = robot.fk(q);
    std::vector<SE3> all_poses = robot.fk_all(q);
    SE3 T_last = all_poses.back();

    REQUIRE(T_tcp.isApprox(T_last));
}

TEST_CASE("FK API: fk uses stored config", "[fk][api]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(0.3, 0.6);

    robot.set_joints(q);

    SE3 T_current = robot.fk();
    SE3 T_explicit = robot.fk(q);

    REQUIRE(T_current.isApprox(T_explicit));
}

TEST_CASE("FK API: invalid link name throws", "[fk][api][error]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(0.0, 0.0);

    REQUIRE_THROWS_AS(robot.fk(q, "nonexistent_link"), std::runtime_error);
}

TEST_CASE("FK API: configuration size mismatch throws", "[fk][api][error]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q_wrong = Eigen::Vector3d(0.0, 0.0, 0.0);

    REQUIRE_THROWS_AS(robot.fk(q_wrong), std::invalid_argument);
}

TEST_CASE("FK: Base frame offset applied correctly", "[fk][base()]") {
    Robot robot = create_2r_planar_robot();

    SE3 base_offset = SE3::Translation(Eigen::Vector3d(1.0, 0.0, 0.0)) *
                      SE3(SO3::RotZ(M_PI / 2.0).matrix(), Eigen::Vector3d::Zero());
    robot.set_base(base_offset);

    Eigen::VectorXd q = Eigen::Vector2d(0.0, 0.0);
    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(1.8, 1e-9));
}

TEST_CASE("FK: TCP with tool offset", "[fk][tool]") {
    Robot robot = create_2r_planar_robot();

    Tool tool("gripper");
    tool.set_tcp_pose(SE3::Translation(Eigen::Vector3d(0.2, 0.0, 0.0)));
    int tool_id = robot.add_tool(tool);
    robot.set_active_tool(tool_id);

    Eigen::VectorXd q = Eigen::Vector2d(0.0, 0.0);
    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(2.0, 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("FK: TCP without active tool", "[fk][tool]") {
    Robot robot = create_2r_planar_robot();

    Tool tool("gripper");
    tool.set_tcp_pose(SE3::Translation(Eigen::Vector3d(0.2, 0.0, 0.0)));
    robot.add_tool(tool);

    Eigen::VectorXd q = Eigen::Vector2d(0.0, 0.0);
    SE3 T_tcp = robot.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();

    REQUIRE_THAT(pos(0), Catch::Matchers::WithinAbs(1.8, 1e-10));
    REQUIRE_THAT(pos(1), Catch::Matchers::WithinAbs(0.0, 1e-10));
}

TEST_CASE("FK UR5: Zero configuration", "[fk][ur5]") {
    Robot ur5 = Robot::from_urdf("test_data/ur5_simplified.urdf");
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);

    SE3 T_tcp = ur5.fk(q);
    Eigen::Vector3d pos = T_tcp.translation();

    double reach = pos.norm();
    REQUIRE(reach > 0.5);
    REQUIRE(reach < 2.0);
}

TEST_CASE("FK UR5: All 90° configuration", "[fk][ur5]") {
    Robot ur5 = Robot::from_urdf("test_data/ur5_simplified.urdf");

    Eigen::VectorXd q(6);
    q << M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2;

    SE3 T_tcp = ur5.fk(q);
    REQUIRE(T_tcp.rotation().matrix().determinant() > 0.99);
}

TEST_CASE("FK UR5: Compute FK to all links", "[fk][ur5]") {
    Robot ur5 = Robot::from_urdf("test_data/ur5_simplified.urdf");
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);

    std::vector<SE3> poses = ur5.fk_all(q);

    REQUIRE(poses.size() == 7);
    REQUIRE(poses[0].isApprox(SE3::Identity()));

    for (size_t i = 1; i < poses.size(); ++i) {
        double dist = poses[i].translation().norm();
        REQUIRE(dist >= 0.0);
    }
}

TEST_CASE("FK UR5: Compute FK by link name", "[fk][ur5]") {
    Robot ur5 = Robot::from_urdf("test_data/ur5_simplified.urdf");
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);

    SE3 T_shoulder = ur5.fk(q, "shoulder_link");
    REQUIRE(T_shoulder.rotation().matrix().determinant() > 0.99);
}

TEST_CASE("FK Performance: 2R robot (target < 10 μs)", "[fk][performance]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(0.5, -0.3);

    for (int i = 0; i < 100; ++i) {
        robot.fk(q);
    }

    const int iterations = 10000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i) {
        SE3 T = robot.fk(q);
        (void)T;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    double avg_time_us = duration.count() / (1000.0 * iterations);

    INFO("FK time (2R): " << avg_time_us << " μs");
    REQUIRE(avg_time_us < 10.0);
}

TEST_CASE("FK Performance: UR5 robot (target < 10 μs)", "[fk][performance]") {
    Robot ur5 = Robot::from_urdf("test_data/ur5_simplified.urdf");
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    q << 0.5, -0.3, 0.8, -1.2, 0.4, 0.6;

    for (int i = 0; i < 100; ++i) {
        ur5.fk(q);
    }

    const int iterations = 10000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i) {
        SE3 T = ur5.fk(q);
        (void)T;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    double avg_time_us = duration.count() / (1000.0 * iterations);

    INFO("FK time (UR5): " << avg_time_us << " μs");
    REQUIRE(avg_time_us < 10.0);
}

TEST_CASE("FK Performance: fk_all (target < 15 μs)", "[fk][performance]") {
    Robot ur5 = Robot::from_urdf("test_data/ur5_simplified.urdf");
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    q << 0.5, -0.3, 0.8, -1.2, 0.4, 0.6;

    for (int i = 0; i < 100; ++i) {
        ur5.fk_all(q);
    }

    const int iterations = 10000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i) {
        std::vector<SE3> poses = ur5.fk_all(q);
        (void)poses;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    double avg_time_us = duration.count() / (1000.0 * iterations);

    INFO("FK time (all links): " << avg_time_us << " μs");
    REQUIRE(avg_time_us < 15.0);
}

TEST_CASE("FK: Empty robot (no links)", "[fk][edge]") {
    Robot robot("empty");
    Eigen::VectorXd q(0);
    SE3 T_tcp = robot.fk(q);

    REQUIRE(T_tcp.isApprox(SE3::Identity()));
}

TEST_CASE("FK: Configuration not set throws for fk", "[fk][edge][error]") {
    Robot robot = create_2r_planar_robot();
    REQUIRE_THROWS_AS(robot.fk(), std::runtime_error);
}

TEST_CASE("FK: Stateless methods work without setting configuration", "[fk][edge]") {
    Robot robot = create_2r_planar_robot();
    Eigen::VectorXd q = Eigen::Vector2d(0.0, 0.0);

    REQUIRE_NOTHROW(robot.fk(q));
    REQUIRE_NOTHROW(robot.fk(q, "link1"));
    REQUIRE_NOTHROW(robot.fk_all(q));
}
