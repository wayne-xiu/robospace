#include <robospace/kinematics/ik_solver.hpp>
#include <robospace/math/SO3.hpp>
#include <cmath>
#include <stdexcept>

namespace robospace {
namespace kinematics {

IKSolver::IKSolver(const model::Robot& robot)
    : robot_(robot) {
}

IKResult IKSolver::solve(const math::SE3& target_pose, const Eigen::VectorXd& q_seed) {
    const int dof = robot_.dof();

    // Validate seed configuration
    if (q_seed.size() != dof) {
        throw std::invalid_argument(
            "Seed configuration size mismatch: expected " +
            std::to_string(dof) + " values, got " +
            std::to_string(q_seed.size()));
    }

    // Initialize solution with seed
    Eigen::VectorXd q = q_seed;

    IKResult result;
    result.q_solution = q;
    result.success = false;
    result.iterations = 0;

    // Iterative IK loop
    for (int iter = 0; iter < max_iterations_; ++iter) {
        result.iterations = iter + 1;

        // Compute current end-effector pose
        math::SE3 current_pose = robot_.fk(q);

        // Compute pose error
        Eigen::VectorXd error = compute_pose_error(current_pose, target_pose);

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

        // Compute damped least squares step
        Eigen::VectorXd dq = compute_dls_step(J_reduced, error_scaled);

        // Check for NaN or infinite values
        if (!dq.allFinite()) {
            result.success = false;
            result.q_solution = q;
            result.message = "IK solver encountered NaN or infinite values";
            return result;
        }

        // Update joint configuration
        q += step_size_ * dq;

        // Store current error for the reduced problem (position-only or full)
        result.final_error = error_reduced.norm();
    }

    // Max iterations reached without convergence
    result.success = false;
    result.q_solution = q;
    result.message = "IK failed to converge within max iterations";

    return result;
}

Eigen::VectorXd IKSolver::compute_pose_error(
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

bool IKSolver::check_convergence(const Eigen::VectorXd& error) const {
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

Eigen::VectorXd IKSolver::compute_dls_step(
    const Eigen::MatrixXd& J,
    const Eigen::VectorXd& error) const {

    const int m = J.rows();  // Task space dimension (3 or 6)
    const int n = J.cols();  // Joint space dimension

    // Damped Least Squares (Levenberg-Marquardt):
    // Two formulations:
    // 1. Δq = J^T(JJ^T + λ²I)^(-1) * e  (used when m < n, i.e., underdetermined)
    // 2. Δq = (J^TJ + λ²I)^(-1) J^T * e (used when m >= n, more numerically stable)
    //
    // For robotics IK, we typically have m=3 or m=6 task dimensions
    // and n=DOF joint dimensions. Use formulation (2) for better stability.

    // Use formulation (2): Δq = (J^TJ + λ²I)^(-1) J^T * e
    Eigen::MatrixXd JTJ = J.transpose() * J;
    Eigen::MatrixXd A = JTJ + damping_ * damping_ * Eigen::MatrixXd::Identity(n, n);

    // Compute J^T * error
    Eigen::VectorXd JT_error = J.transpose() * error;

    // Solve: (J^TJ + λ²I) * dq = J^T * e
    Eigen::VectorXd dq = A.ldlt().solve(JT_error);

    return dq;
}

} // namespace kinematics
} // namespace robospace
