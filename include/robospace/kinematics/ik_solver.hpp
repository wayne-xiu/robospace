#pragma once

#include <robospace/model/robot.hpp>
#include <robospace/math/SE3.hpp>
#include <Eigen/Dense>
#include <string>

namespace robospace {
namespace kinematics {

/**
 * @brief IK solver mode
 *
 * Determines which error components to minimize:
 * - FULL_POSE: Minimize both position and orientation errors (6D)
 * - POSITION_ONLY: Minimize only position error (3D), ignore orientation
 * - ORIENTATION_ONLY: Minimize only orientation error (3D), ignore position
 */
enum class IKMode {
    FULL_POSE,         ///< Minimize full 6D pose error
    POSITION_ONLY,     ///< Minimize position error only (useful for < 6 DOF robots)
    ORIENTATION_ONLY   ///< Minimize orientation error only
};

/**
 * @brief Result of an IK solve attempt
 */
struct IKResult {
    bool success = false;                    ///< Whether IK converged to target
    Eigen::VectorXd q_solution;              ///< Joint configuration solution
    int iterations = 0;                      ///< Number of iterations used
    double final_error = 0.0;                ///< Final pose error (position + orientation)
    std::string message;                     ///< Optional status/error message

    IKResult() = default;
    IKResult(bool success_, const Eigen::VectorXd& q_, int iter_ = 0, double err_ = 0.0)
        : success(success_), q_solution(q_), iterations(iter_), final_error(err_) {}
};

/**
 * @brief Numerical inverse kinematics solver using Jacobian pseudoinverse
 *
 * Implements damped least squares (Levenberg-Marquardt style) IK:
 *   Δq = J^T(JJ^T + λ²I)^(-1) * e
 *
 * Where:
 * - J is the geometric Jacobian (6×n)
 * - e is the pose error vector (6×1) in base frame
 * - λ is the damping factor (helps near singularities)
 *
 * Features:
 * - Damped least squares for singularity robustness
 * - Position-only, orientation-only, or full pose modes
 * - Configurable tolerances and iteration limits
 * - Convergence tracking
 *
 * Limitations (Phase 2):
 * - No joint limit avoidance (Phase 2 focus on core algorithm)
 * - No collision avoidance
 * - No null-space optimization
 * - Single solution (no multiple solution enumeration)
 *
 * Usage:
 * @code
 * IKSolver solver(robot);
 * solver.set_position_tolerance(1e-6);
 * solver.set_damping(0.01);
 *
 * SE3 target_pose = ...;
 * Eigen::VectorXd q_seed = robot.home();
 * IKResult result = solver.solve(target_pose, q_seed);
 *
 * if (result.success) {
 *     std::cout << "IK succeeded in " << result.iterations << " iterations\n";
 *     std::cout << "Solution: " << result.q_solution.transpose() << "\n";
 * }
 * @endcode
 */
class IKSolver {
public:
    /**
     * @brief Construct IK solver for a robot
     * @param robot Robot model
     */
    explicit IKSolver(const model::Robot& robot);

    /**
     * @brief Solve inverse kinematics for target pose
     *
     * Iteratively updates joint configuration to minimize pose error using
     * damped least squares Jacobian pseudoinverse method.
     *
     * @param target_pose Desired end-effector pose in base frame
     * @param q_seed Initial joint configuration (seed)
     * @return IKResult with solution if successful
     *
     * @throws std::invalid_argument if q_seed size doesn't match robot DOF
     */
    IKResult solve(const math::SE3& target_pose, const Eigen::VectorXd& q_seed);

    // Configuration methods

    /**
     * @brief Set position error tolerance (meters)
     * @param tol Position tolerance (default: 1e-6 m)
     */
    void set_position_tolerance(double tol) { position_tolerance_ = tol; }

    /**
     * @brief Set orientation error tolerance (radians)
     * @param tol Orientation tolerance (default: 1e-4 rad)
     */
    void set_orientation_tolerance(double tol) { orientation_tolerance_ = tol; }

    /**
     * @brief Set maximum iterations
     * @param max_iter Maximum iterations (default: 100)
     */
    void set_max_iterations(int max_iter) { max_iterations_ = max_iter; }

    /**
     * @brief Set damping factor for DLS
     * @param damping Damping factor λ (default: 0.01)
     *
     * Larger values: More stable near singularities, slower convergence
     * Smaller values: Faster convergence, less stable near singularities
     */
    void set_damping(double damping) { damping_ = damping; }

    /**
     * @brief Set IK solver mode
     * @param mode Solver mode (FULL_POSE, POSITION_ONLY, ORIENTATION_ONLY)
     */
    void set_mode(IKMode mode) { mode_ = mode; }

    /**
     * @brief Set joint update step size
     * @param alpha Step size (default: 1.0)
     *
     * Smaller values: More conservative, slower but more stable
     * Larger values: Faster but may overshoot
     */
    void set_step_size(double alpha) { step_size_ = alpha; }

    // Getters

    double position_tolerance() const { return position_tolerance_; }
    double orientation_tolerance() const { return orientation_tolerance_; }
    int max_iterations() const { return max_iterations_; }
    double damping() const { return damping_; }
    IKMode mode() const { return mode_; }
    double step_size() const { return step_size_; }

private:
    const model::Robot& robot_;

    // Solver parameters
    double position_tolerance_ = 1e-6;       ///< Position error tolerance (m)
    double orientation_tolerance_ = 1e-4;    ///< Orientation error tolerance (rad)
    int max_iterations_ = 500;               ///< Maximum iterations (increased for better convergence)
    double damping_ = 0.1;                   ///< Damping factor λ for DLS (increased for stability)
    double step_size_ = 0.5;                 ///< Joint update step size (decreased for stability)
    IKMode mode_ = IKMode::FULL_POSE;        ///< Solver mode

    /**
     * @brief Compute pose error vector in base frame
     *
     * Error = [position_error; orientation_error]
     *   position_error = target_pos - current_pos  (3×1)
     *   orientation_error = log(R_current^T * R_target) (3×1, axis-angle)
     *
     * @param current Current end-effector pose
     * @param target Target end-effector pose
     * @return 6×1 error vector
     */
    Eigen::VectorXd compute_pose_error(const math::SE3& current, const math::SE3& target) const;

    /**
     * @brief Check if current pose is within tolerance of target
     * @param error Pose error vector (6×1)
     * @return true if converged
     */
    bool check_convergence(const Eigen::VectorXd& error) const;

    /**
     * @brief Compute damped least squares pseudoinverse
     *
     * Solves: Δq = J^T(JJ^T + λ²I)^(-1) * e
     *
     * @param J Jacobian matrix (6×n or subset based on mode)
     * @param error Pose error vector
     * @return Joint velocity Δq
     */
    Eigen::VectorXd compute_dls_step(const Eigen::MatrixXd& J, const Eigen::VectorXd& error) const;
};

} // namespace kinematics
} // namespace robospace
