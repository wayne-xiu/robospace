#include <robospace/kinematics/numerical_ik.hpp>
#include <robospace/math/SO3.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace robospace {
namespace kinematics {

NumericalIKSolver::NumericalIKSolver(const model::Robot& robot)
    : robot_(robot) {
}

IKResult NumericalIKSolver::solve(
    const math::SE3& target_pose,
    const Eigen::VectorXd& q_seed) {

    const int dof = robot_.dof();

    // Validate seed configuration
    if (q_seed.size() != dof) {
        throw std::invalid_argument(
            "Seed configuration size mismatch: expected " +
            std::to_string(dof) + " values, got " +
            std::to_string(q_seed.size()));
    }

    // If random restart is disabled, do single attempt
    if (!search_params_.use_random_restart) {
        auto result = solve_single_attempt(
            target_pose, q_seed, search_params_.max_iterations);
        result.searches = 1;
        return result;
    }

    // ========== RANDOM RESTART STRATEGY ==========
    // Based on Peter Corke's Robotics Toolbox approach
    //
    // Try multiple random starting configurations to escape local minima
    // Success probability: 1 - (1-p)^n
    // With p=0.1, n=100: Success = 99.997%

    // First attempt: Use provided seed
    IKResult best_result = solve_single_attempt(
        target_pose, q_seed, search_params_.max_iterations);
    best_result.searches = 1;

    if (best_result.success) {
        return best_result;
    }

    // Initialize random number generator
    std::mt19937 rng;
    if (search_params_.seed != 0) {
        rng.seed(search_params_.seed);
    } else {
        std::random_device rd;
        rng.seed(rd());
    }

    // Random restart loop
    for (int search = 1; search < search_params_.max_searches; ++search) {
        // Generate random configuration
        Eigen::VectorXd q_random = generate_random_config(rng);

        // Solve from random seed
        IKResult result = solve_single_attempt(
            target_pose, q_random, search_params_.max_iterations);

        // Track number of searches
        result.searches = search + 1;

        if (result.success) {
            return result;
        }

        // Keep track of best attempt (lowest error)
        if (result.final_error < best_result.final_error) {
            best_result = result;
            best_result.searches = search + 1;
        }
    }

    // No successful solution found, return best attempt
    best_result.message = "IK failed after " +
        std::to_string(best_result.searches) + " random restart attempts";
    return best_result;
}

IKResult NumericalIKSolver::solve_single_attempt(
    const math::SE3& target,
    const Eigen::VectorXd& q_seed,
    int max_iterations) {

    const int dof = robot_.dof();

    // Initialize solution with seed
    Eigen::VectorXd q = q_seed;

    IKResult result;
    result.q_solution = q;
    result.success = false;
    result.iterations = 0;

    // Iterative damped least squares loop
    for (int iter = 0; iter < max_iterations; ++iter) {
        result.iterations = iter + 1;

        // Compute current end-effector pose
        math::SE3 current_pose = robot_.fk(q);

        // Compute pose error
        Eigen::VectorXd error = compute_pose_error(current_pose, target);

        // Check convergence
        if (check_convergence(error)) {
            result.success = true;
            result.q_solution = q;
            result.final_error = error.norm();
            result.message = "IK converged successfully";
            return result;
        }

        // Compute Jacobian in base frame
        Eigen::MatrixXd J = robot_.jacob0(q);

        // Select relevant rows based on mode
        Eigen::MatrixXd J_reduced;
        Eigen::VectorXd error_reduced;

        switch (mode_) {
            case IKMode::POSITION_ONLY:
                // Use only position rows (0:3)
                J_reduced = J.topRows(3);
                error_reduced = error.head(3);
                break;

            case IKMode::ORIENTATION_ONLY:
                // Use only orientation rows (3:6)
                J_reduced = J.bottomRows(3);
                error_reduced = error.tail(3);
                break;

            case IKMode::FULL_POSE:
            default:
                // Use all 6 rows
                J_reduced = J;
                error_reduced = error;
                break;
        }

        // Scale error by gain before solving
        Eigen::VectorXd error_scaled = error_gain_ * error_reduced;

        // Compute damping (adaptive or constant)
        double damping = search_params_.use_adaptive_damping
            ? compute_adaptive_damping(error_reduced)
            : base_damping_;

        // Compute damped least squares step
        Eigen::VectorXd dq = compute_dls_step(J_reduced, error_scaled, damping);

        // Check for NaN or infinite values
        if (!dq.allFinite()) {
            result.success = false;
            result.q_solution = q;
            result.final_error = error_reduced.norm();
            result.message = "IK solver encountered NaN or infinite values";
            return result;
        }

        // Update joint configuration
        q += step_size_ * dq;

        // Store current error for the reduced problem
        result.final_error = error_reduced.norm();
    }

    // Max iterations reached without convergence
    result.success = false;
    result.q_solution = q;
    result.message = "IK failed to converge within max iterations";

    return result;
}

Eigen::VectorXd NumericalIKSolver::generate_random_config(std::mt19937& rng) {
    const int dof = robot_.dof();
    Eigen::VectorXd q_random(dof);

    // TODO: Check if robot has joint limits and use them
    // For now, sample uniformly from [-π, π] for revolute joints

    std::uniform_real_distribution<double> dist(-M_PI, M_PI);

    for (int i = 0; i < dof; ++i) {
        q_random(i) = dist(rng);
    }

    return q_random;
}

Eigen::VectorXd NumericalIKSolver::compute_pose_error(
    const math::SE3& current,
    const math::SE3& target) const {

    Eigen::VectorXd error(6);

    // Position error: target - current (in base frame)
    Eigen::Vector3d position_error = target.translation() - current.translation();
    error.head(3) = position_error;

    // Orientation error using axis-angle representation
    // We want the error that brings us from current to target rotation
    // R_target = R_current * R_error  =>  R_error = R_current^T * R_target
    Eigen::Matrix3d R_current = current.rotation();
    Eigen::Matrix3d R_target = target.rotation();
    Eigen::Matrix3d R_error = R_current.transpose() * R_target;

    // Convert rotation matrix to axis-angle (using SO3 logarithm)
    // This gives the angular velocity in the current end-effector frame
    math::SO3 so3_error(R_error);
    math::so3 orientation_error_log = math::log_SO3(so3_error);

    // Transform the orientation error from EE frame to base frame
    // ω_base = R_current * ω_ee
    Eigen::Vector3d orientation_error = R_current * orientation_error_log.vector();
    error.tail(3) = orientation_error;

    return error;
}

bool NumericalIKSolver::check_convergence(const Eigen::VectorXd& error) const {
    // Extract position and orientation errors
    const Eigen::Vector3d pos_error = error.head(3);
    const Eigen::Vector3d ori_error = error.tail(3);

    const double pos_norm = pos_error.norm();
    const double ori_norm = ori_error.norm();

    // Check based on mode
    switch (mode_) {
        case IKMode::POSITION_ONLY:
            return pos_norm < position_tolerance_;

        case IKMode::ORIENTATION_ONLY:
            return ori_norm < orientation_tolerance_;

        case IKMode::FULL_POSE:
        default:
            return (pos_norm < position_tolerance_) &&
                   (ori_norm < orientation_tolerance_);
    }
}

Eigen::VectorXd NumericalIKSolver::compute_dls_step(
    const Eigen::MatrixXd& J,
    const Eigen::VectorXd& error,
    double damping) const {

    const int m = J.rows();  // Task space dimension (3 or 6)
    const int n = J.cols();  // Joint space dimension

    // Damped Least Squares (Levenberg-Marquardt):
    // Δq = (J^TJ + λ²I)^(-1) J^T * e
    //
    // This formulation is more numerically stable than J^T(JJ^T + λ²I)^(-1)
    // when m < n (underdetermined systems, common in robotics)

    Eigen::MatrixXd JTJ = J.transpose() * J;
    Eigen::MatrixXd A = JTJ + damping * damping * Eigen::MatrixXd::Identity(n, n);

    // Compute J^T * error
    Eigen::VectorXd JT_error = J.transpose() * error;

    // Solve: (J^TJ + λ²I) * dq = J^T * e
    // Use LDLT decomposition for positive semi-definite matrices
    Eigen::VectorXd dq = A.ldlt().solve(JT_error);

    return dq;
}

double NumericalIKSolver::compute_adaptive_damping(
    const Eigen::VectorXd& error) const {

    // Sugihara's adaptive damping method:
    // λ = sqrt(||e||² + λ_base²)
    //
    // Properties:
    // - When error is large: λ ≈ ||e|| (large damping, stable)
    // - When error is small: λ ≈ λ_base (small damping, fast convergence)
    // - Smoothly transitions between the two regimes
    // - "Solvability-unconcerned" - works even without exact solution

    double error_norm_sq = error.squaredNorm();
    double lambda = std::sqrt(error_norm_sq + base_damping_ * base_damping_);

    return lambda;
}

} // namespace kinematics
} // namespace robospace
