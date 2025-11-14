#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/kinematics/ik_solver.hpp>
#include <robospace/model/robot.hpp>
#include <robospace/math/SE3.hpp>
#include <robospace/math/SO3.hpp>
#include <cmath>
#include <iostream>

using namespace robospace::kinematics;
using namespace robospace::model;
using namespace robospace::math;

TEST_CASE("IK UR5: Load robot from URDF", "[ik][ur5]") {
    Robot ur5 = Robot::from_urdf("../tests/test_data/ur5_simplified.urdf");

    REQUIRE(ur5.is_valid());
    REQUIRE(ur5.dof() == 6);

    std::cout << "UR5 loaded: " << ur5.dof() << " DOF\n";
}

TEST_CASE("IK UR5: FK->IK->FK roundtrip at home", "[ik][ur5][roundtrip]") {
    Robot ur5 = Robot::from_urdf("../tests/test_data/ur5_simplified.urdf");
    IKSolver solver(ur5);

    // Use position-only mode for better convergence with 6DOF
    solver.set_mode(IKMode::POSITION_ONLY);

    // Start from home configuration
    Eigen::VectorXd q_home = Eigen::VectorXd::Zero(6);

    // Compute FK
    SE3 T_target = ur5.fk(q_home);

    std::cout << "Home position: " << T_target.translation().transpose() << "\n";

    // Solve IK
    IKResult result = solver.solve(T_target, q_home);

    REQUIRE(result.success);
    REQUIRE(result.iterations > 0);

    // Verify FK roundtrip
    SE3 T_result = ur5.fk(result.q_solution);
    Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();

    std::cout << "IK converged in " << result.iterations << " iterations\n";
    std::cout << "Position error: " << pos_diff.norm() << " m\n";

    REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("IK UR5: FK->IK->FK roundtrip at arbitrary config", "[ik][ur5][roundtrip]") {
    Robot ur5 = Robot::from_urdf("../tests/test_data/ur5_simplified.urdf");
    IKSolver solver(ur5);
    solver.set_mode(IKMode::POSITION_ONLY);

    // Test multiple configurations
    std::vector<Eigen::VectorXd> test_configs = {
        (Eigen::VectorXd(6) << 0.0, -M_PI/4, M_PI/4, 0.0, M_PI/2, 0.0).finished(),
        (Eigen::VectorXd(6) << M_PI/6, -M_PI/3, M_PI/3, -M_PI/6, M_PI/4, M_PI/6).finished(),
        (Eigen::VectorXd(6) << -M_PI/4, -M_PI/2, M_PI/2, M_PI/4, -M_PI/4, -M_PI/2).finished()
    };

    int success_count = 0;
    for (size_t i = 0; i < test_configs.size(); ++i) {
        const auto& q_start = test_configs[i];

        // Compute FK
        SE3 T_target = ur5.fk(q_start);

        // Solve IK from different seed
        Eigen::VectorXd q_seed = Eigen::VectorXd::Zero(6);
        IKResult result = solver.solve(T_target, q_seed);

        if (result.success) {
            // Verify FK roundtrip
            SE3 T_result = ur5.fk(result.q_solution);
            Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();

            std::cout << "Config " << i << ": converged in " << result.iterations
                      << " iterations, error = " << pos_diff.norm() << " m\n";

            if (pos_diff.norm() < 1e-4) {
                success_count++;
            }
        }
    }

    // At least 2 out of 3 should converge well
    REQUIRE(success_count >= 2);
}

TEST_CASE("IK UR5: Reachable target position", "[ik][ur5]") {
    Robot ur5 = Robot::from_urdf("../tests/test_data/ur5_simplified.urdf");
    IKSolver solver(ur5);
    solver.set_mode(IKMode::POSITION_ONLY);

    // Create a reachable target within UR5 workspace
    SE3 T_target = SE3::Identity();
    T_target.translation() << 0.4, 0.2, 0.3;  // Typical reachable position for UR5

    Eigen::VectorXd q_seed = Eigen::VectorXd::Zero(6);
    IKResult result = solver.solve(T_target, q_seed);

    std::cout << "Reachable target test:\n";
    std::cout << "  Success: " << result.success << "\n";
    std::cout << "  Iterations: " << result.iterations << "\n";
    std::cout << "  Final error: " << result.final_error << "\n";

    if (result.success) {
        SE3 T_result = ur5.fk(result.q_solution);
        Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();
        std::cout << "  Position error: " << pos_diff.norm() << " m\n";
        std::cout << "  Solution: " << result.q_solution.transpose() << "\n";

        REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 1e-3));
    }
}

TEST_CASE("IK UR5: Full 6D pose (FULL_POSE mode)", "[ik][ur5][full_pose]") {
    Robot ur5 = Robot::from_urdf("../tests/test_data/ur5_simplified.urdf");
    IKSolver solver(ur5);

    // Use FULL_POSE mode (6DOF robot can control full pose)
    solver.set_mode(IKMode::FULL_POSE);
    solver.set_max_iterations(1000);  // May need more iterations for full pose

    // Start configuration
    Eigen::VectorXd q_start(6);
    q_start << 0.0, -M_PI/4, M_PI/3, 0.0, M_PI/2, 0.0;

    // Get target pose
    SE3 T_target = ur5.fk(q_start);

    std::cout << "Full pose IK test:\n";
    std::cout << "Target position: " << T_target.translation().transpose() << "\n";

    // Solve from different seed
    Eigen::VectorXd q_seed = Eigen::VectorXd::Zero(6);
    IKResult result = solver.solve(T_target, q_seed);

    std::cout << "  Success: " << result.success << "\n";
    std::cout << "  Iterations: " << result.iterations << "\n";
    std::cout << "  Final error: " << result.final_error << "\n";

    if (result.success) {
        SE3 T_result = ur5.fk(result.q_solution);
        Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();

        // Check orientation error
        Eigen::Matrix3d R_error = T_result.rotation().transpose() * T_target.rotation();
        SO3 so3_error(R_error);
        so3 ori_error = log_SO3(so3_error);
        double ori_error_norm = ori_error.vector().norm();

        std::cout << "  Position error: " << pos_diff.norm() << " m\n";
        std::cout << "  Orientation error: " << ori_error_norm << " rad\n";

        // For full pose, both should be small
        REQUIRE_THAT(pos_diff.norm(), Catch::Matchers::WithinAbs(0.0, 1e-3));
        REQUIRE_THAT(ori_error_norm, Catch::Matchers::WithinAbs(0.0, 1e-2));
    }
}

TEST_CASE("IK UR5: Multiple solutions exist", "[ik][ur5][solutions]") {
    Robot ur5 = Robot::from_urdf("../tests/test_data/ur5_simplified.urdf");
    IKSolver solver(ur5);
    solver.set_mode(IKMode::POSITION_ONLY);

    // Set a target position
    SE3 T_target = SE3::Identity();
    T_target.translation() << 0.5, 0.2, 0.4;

    // Try different seeds - should converge to potentially different solutions
    std::vector<Eigen::VectorXd> seeds = {
        Eigen::VectorXd::Zero(6),
        (Eigen::VectorXd(6) << M_PI/4, 0, 0, 0, 0, 0).finished(),
        (Eigen::VectorXd(6) << -M_PI/4, 0, 0, 0, 0, 0).finished()
    };

    std::vector<Eigen::VectorXd> solutions;

    for (size_t i = 0; i < seeds.size(); ++i) {
        IKResult result = solver.solve(T_target, seeds[i]);

        if (result.success) {
            SE3 T_result = ur5.fk(result.q_solution);
            Eigen::Vector3d pos_diff = T_result.translation() - T_target.translation();

            if (pos_diff.norm() < 1e-3) {
                solutions.push_back(result.q_solution);
                std::cout << "Seed " << i << " -> solution: "
                          << result.q_solution.transpose() << "\n";
            }
        }
    }

    std::cout << "Found " << solutions.size() << " valid solutions from different seeds\n";

    // At least one solution should be found
    REQUIRE(solutions.size() >= 1);

    // If multiple solutions found, they should be different (not just converged to same)
    if (solutions.size() > 1) {
        double diff = (solutions[0] - solutions[1]).norm();
        std::cout << "Solution difference: " << diff << " rad\n";
        // If they're very different, note it (but don't require it - may converge to same)
        if (diff > 0.1) {
            std::cout << "Multiple distinct solutions found!\n";
        }
    }
}

TEST_CASE("IK UR5: Performance with 6DOF", "[ik][ur5][performance][!benchmark]") {
    Robot ur5 = Robot::from_urdf("../tests/test_data/ur5_simplified.urdf");
    IKSolver solver(ur5);
    solver.set_mode(IKMode::POSITION_ONLY);

    SE3 T_target = SE3::Identity();
    T_target.translation() << 0.4, 0.2, 0.3;
    Eigen::VectorXd q_seed = Eigen::VectorXd::Zero(6);

    auto start = std::chrono::high_resolution_clock::now();

    const int num_iterations = 100;
    int success_count = 0;
    for (int i = 0; i < num_iterations; ++i) {
        IKResult result = solver.solve(T_target, q_seed);
        if (result.success) success_count++;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double avg_time_us = duration.count() / static_cast<double>(num_iterations);

    std::cout << "UR5 IK Performance:\n";
    std::cout << "  Average time: " << avg_time_us << " µs\n";
    std::cout << "  Success rate: " << (100.0 * success_count / num_iterations) << "%\n";

    // 6DOF IK should still be reasonably fast (< 10ms)
    REQUIRE(avg_time_us < 10000.0);
}
