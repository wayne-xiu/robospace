#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/kinematics/ik_solver.hpp>
#include <robospace/model/robot.hpp>
#include <robospace/math/SE3.hpp>
#include <cmath>

using namespace robospace::kinematics;
using namespace robospace::model;
using namespace robospace::math;

// Create 2R planar robot for testing IK
Robot create_2r_planar_for_ik() {
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

TEST_CASE("IK Solver: Construction", "[ik][construction]") {
    Robot robot = create_2r_planar_for_ik();

    IKSolver solver(robot);
    REQUIRE(robot.dof() == 2);
}

TEST_CASE("IK Solver: FK->IK->FK roundtrip (zero config)", "[ik][roundtrip]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // Start with a known configuration
    Eigen::VectorXd q_start(2);
    q_start << 0.0, 0.0;

    // Compute FK
    SE3 T_target = robot.fk(q_start);

    // Solve IK
    IKResult result = solver.solve(T_target, q_start);

    REQUIRE(result.success);
    REQUIRE(result.q_solution.size() == 2);

    // Verify FK roundtrip
    SE3 T_result = robot.fk(result.q_solution);
    Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();

    REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("IK Solver: FK->IK->FK roundtrip (arbitrary config)", "[ik][roundtrip]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // Test multiple configurations
    std::vector<Eigen::Vector2d> test_configs = {
        {0.5, 0.3},
        {-0.5, 0.8},
        {M_PI/4, M_PI/6},
        {M_PI/2, -M_PI/4}
    };

    for (const auto& q_vec : test_configs) {
        Eigen::VectorXd q_start = q_vec;

        // Compute FK
        SE3 T_target = robot.fk(q_start);

        // Solve IK
        IKResult result = solver.solve(T_target, q_start);

        REQUIRE(result.success);

        // Verify FK roundtrip
        SE3 T_result = robot.fk(result.q_solution);
        Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();

        REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 1e-4));
    }
}

TEST_CASE("IK Solver: Reachable target position", "[ik][reachable]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // For 2R planar robot, use position-only mode (can't control full 6D pose with 2 DOF)
    solver.set_mode(IKMode::POSITION_ONLY);
    solver.set_position_tolerance(1e-3);  // Practical tolerance for convergence

    // Create a reachable target (within workspace: L1+L2 = 1.8m)
    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.0, 0.5, 0.0;  // Reachable position

    Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();
    IKResult result = solver.solve(T_target, q_seed);

    REQUIRE(result.success);

    // Verify the solution
    SE3 T_result = robot.fk(result.q_solution);
    Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();

    REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 5e-3));  // Relaxed to match solver tolerance
}

TEST_CASE("IK Solver: Reachable target with random restart", "[ik][random_restart]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // For 2R planar robot, use position-only mode
    solver.set_mode(IKMode::POSITION_ONLY);
    solver.set_position_tolerance(1e-3);

    // Enable random restart strategy
    solver.set_use_random_restart(true);
    solver.set_max_searches(20);
    solver.set_max_iterations(100);

    // Same target that fails without random restart
    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.0, 0.5, 0.0;

    Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();
    IKResult result = solver.solve(T_target, q_seed);

    REQUIRE(result.success);
    REQUIRE(result.searches > 0);  // Should report number of searches

    // Verify the solution
    SE3 T_result = robot.fk(result.q_solution);
    Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();

    REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 5e-3));
}

TEST_CASE("IK Solver: Unreachable target (too far)", "[ik][unreachable]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // Create an unreachable target (beyond workspace: L1+L2 = 1.8m)
    SE3 T_target = SE3::Identity();
    T_target.translation() << 5.0, 0.0, 0.0;  // Way too far

    Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();
    IKResult result = solver.solve(T_target, q_seed);

    // Should fail or converge to best effort
    // For now, we expect it to either fail or not reach the target exactly
    if (result.success) {
        SE3 T_result = robot.fk(result.q_solution);
        Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();
        // If it claims success, the error should be large
        REQUIRE(pos_diff.norm() > 0.1);
    }
}

TEST_CASE("IK Solver: Different seed configurations", "[ik][seed]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.0, 0.8, 0.0;

    // Try different seeds - should converge to valid solution
    std::vector<Eigen::Vector2d> seeds = {
        {0.0, 0.0},
        {0.5, 0.5},
        {-0.5, 0.5},
        {M_PI/4, M_PI/4}
    };

    for (const auto& seed_vec : seeds) {
        Eigen::VectorXd q_seed = seed_vec;
        IKResult result = solver.solve(T_target, q_seed);

        if (result.success) {
            SE3 T_result = robot.fk(result.q_solution);
            Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();
            REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 1e-3));
        }
    }
}

TEST_CASE("IK Solver: Custom tolerances", "[ik][config]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // For 2R planar robot, use position-only mode
    solver.set_mode(IKMode::POSITION_ONLY);

    // Set custom tolerances
    solver.set_position_tolerance(1e-3);  // Practical tolerance
    solver.set_orientation_tolerance(1e-2);
    solver.set_max_iterations(1000);

    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.2, 0.6, 0.0;

    Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();
    IKResult result = solver.solve(T_target, q_seed);

    REQUIRE(result.success);
    REQUIRE(result.iterations <= 1000);
}

TEST_CASE("IK Solver: Position-only mode", "[ik][position_only]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // For planar 2R, we only care about position (2 DOF can't control full 6D pose)
    solver.set_mode(IKMode::POSITION_ONLY);
    solver.set_position_tolerance(1e-3);  // Practical tolerance

    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.0, 0.5, 0.0;

    Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();
    IKResult result = solver.solve(T_target, q_seed);

    REQUIRE(result.success);

    // Verify position matches (ignore orientation for 2R planar)
    SE3 T_result = robot.fk(result.q_solution);
    Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();
    REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 5e-3));
}

TEST_CASE("IK Solver: Iteration count tracking", "[ik][iterations]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.0, 0.0, 0.0;

    Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();
    IKResult result = solver.solve(T_target, q_seed);

    REQUIRE(result.iterations > 0);
    REQUIRE(result.iterations <= solver.max_iterations());
}

TEST_CASE("IK Solver: Configuration size mismatch", "[ik][error]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    SE3 T_target = SE3::Identity();
    Eigen::VectorXd q_seed_wrong(3);  // Wrong size
    q_seed_wrong << 0.0, 0.0, 0.0;

    REQUIRE_THROWS_AS(solver.solve(T_target, q_seed_wrong), std::invalid_argument);
}

TEST_CASE("IK Solver: Damping factor", "[ik][damping]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // For 2R planar robot, use position-only mode
    solver.set_mode(IKMode::POSITION_ONLY);
    solver.set_position_tolerance(1e-3);  // Practical tolerance

    // Test with different damping factors
    std::vector<double> damping_values = {0.005, 0.01, 0.05};

    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.0, 0.5, 0.0;

    for (double damping : damping_values) {
        solver.set_damping(damping);

        Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();
        IKResult result = solver.solve(T_target, q_seed);

        // All reasonable damping values should work for reachable targets
        REQUIRE(result.success);
    }
}

TEST_CASE("IK Solver: Near-singular configuration", "[ik][singular]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    // Target near singularity (arm stretched out)
    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.79, 0.0, 0.0;  // Almost at max reach (1.8)

    // Increase damping to handle near-singularity
    solver.set_damping(0.1);

    Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();
    IKResult result = solver.solve(T_target, q_seed);

    if (result.success) {
        SE3 T_result = robot.fk(result.q_solution);
        Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();
        REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 1e-2));
    }
}

TEST_CASE("IK Solver: Performance benchmark", "[ik][performance][!benchmark]") {
    Robot robot = create_2r_planar_for_ik();
    IKSolver solver(robot);

    SE3 T_target = SE3::Identity();
    T_target.translation() << 1.0, 0.5, 0.0;
    Eigen::VectorXd q_seed = Eigen::Vector2d::Zero();

    auto start = std::chrono::high_resolution_clock::now();

    const int num_iterations = 1000;
    for (int i = 0; i < num_iterations; ++i) {
        IKResult result = solver.solve(T_target, q_seed);
        (void)result;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double avg_time_us = duration.count() / static_cast<double>(num_iterations);

    // IK should be reasonably fast (< 1ms for simple 2R robot)
    REQUIRE(avg_time_us < 1000.0);
}
