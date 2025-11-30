#pragma once

#include <robospace/model/robot.hpp>
#include <robospace/math/SE3.hpp>
#include <robospace/kinematics/ik_solver.hpp>
#include <Eigen/Dense>
#include <random>

namespace robospace {
namespace kinematics {

/**
 * @brief Search parameters for random restart strategy
 *
 * Based on Peter Corke's Robotics Toolbox approach:
 * - Multiple random restart attempts improve success rate dramatically
 * - Success probability: 1 - (1-p)^n where p = single attempt success rate
 * - With p=0.1, n=100: Success rate >99.99%
 */
struct SearchParams {
    int max_searches = 100;        ///< Maximum random restart attempts (slimit)
    int max_iterations = 50;       ///< Iterations per search attempt (ilimit)
    uint32_t seed = 0;             ///< Random seed for reproducibility (0 = random)
    bool use_random_restart = true; ///< Enable/disable random restart
    bool use_adaptive_damping = false; ///< Use Sugihara adaptive damping (experimental)
};

/**
 * @brief Improved numerical IK solver with random restart strategy
 *
 * Key improvements over basic damped least squares:
 * 1. Random restart strategy - Multiple attempts from random seeds
 * 2. Adaptive damping (Sugihara method) - Automatically adjusts based on error
 * 3. Better convergence tracking and diagnostics
 *
 * Algorithm:
 * 1. First attempt: Use provided seed configuration
 * 2. If failed: Try random configurations (max_searches attempts)
 * 3. Each attempt: Run DLS iteration (max_iterations per attempt)
 * 4. Return best solution found across all attempts
 *
 * References:
 * - Peter Corke's Robotics Toolbox: Random restart strategy
 * - Sugihara (2011): Residual-based adaptive damping
 * - Wampler (1986): Damped least squares foundation
 */
class NumericalIKSolver {
public:
    /**
     * @brief Construct numerical IK solver
     * @param robot Robot model
     */
    explicit NumericalIKSolver(const model::Robot& robot);

    /**
     * @brief Solve IK with random restart strategy
     *
     * @param target_pose Desired end-effector pose
     * @param q_seed Initial configuration (first attempt)
     * @return IKResult with best solution found
     */
    IKResult solve(const math::SE3& target_pose, const Eigen::VectorXd& q_seed);

    // ========== CONFIGURATION ==========

    /** @brief Set search parameters */
    void set_search_params(const SearchParams& params) { search_params_ = params; }

    /** @brief Set position tolerance (meters) */
    void set_position_tolerance(double tol) { position_tolerance_ = tol; }

    /** @brief Set orientation tolerance (radians) */
    void set_orientation_tolerance(double tol) { orientation_tolerance_ = tol; }

    /** @brief Set base damping factor */
    void set_damping(double damping) { base_damping_ = damping; }

    /** @brief Set step size (0.0 to 1.0) */
    void set_step_size(double alpha) { step_size_ = alpha; }

    /** @brief Set error gain factor */
    void set_error_gain(double gain) { error_gain_ = gain; }

    /** @brief Set IK mode */
    void set_mode(IKMode mode) { mode_ = mode; }

    // ========== GETTERS ==========

    const SearchParams& search_params() const { return search_params_; }
    double position_tolerance() const { return position_tolerance_; }
    double orientation_tolerance() const { return orientation_tolerance_; }
    double damping() const { return base_damping_; }
    double step_size() const { return step_size_; }
    double error_gain() const { return error_gain_; }
    IKMode mode() const { return mode_; }

private:
    const model::Robot& robot_;

    // Solver parameters
    SearchParams search_params_;
    double position_tolerance_ = 1e-4;       ///< Position convergence tolerance
    double orientation_tolerance_ = 1e-3;    ///< Orientation convergence tolerance
    double base_damping_ = 0.1;              ///< Base damping factor (increased for stability)
    double step_size_ = 1.0;                 ///< Joint update step size
    double error_gain_ = 1.0;                ///< Error scaling gain
    IKMode mode_ = IKMode::FULL_POSE;        ///< Solver mode

    /**
     * @brief Single IK attempt from given seed
     *
     * Runs damped least squares iteration for max_iterations
     *
     * @param target Target pose
     * @param q_seed Initial configuration
     * @param max_iterations Maximum iterations for this attempt
     * @return IKResult for this attempt
     */
    IKResult solve_single_attempt(
        const math::SE3& target,
        const Eigen::VectorXd& q_seed,
        int max_iterations);

    /**
     * @brief Generate random configuration within joint limits
     *
     * If robot has joint limits, samples uniformly within them.
     * Otherwise, samples from [-π, π] for revolute joints.
     *
     * @param rng Random number generator
     * @return Random joint configuration
     */
    Eigen::VectorXd generate_random_config(std::mt19937& rng);

    /**
     * @brief Compute pose error vector
     *
     * Error = [position_error; orientation_error]
     * - position_error = target_pos - current_pos (3×1)
     * - orientation_error = log(R_current^T * R_target) (3×1, axis-angle)
     *
     * @param current Current end-effector pose
     * @param target Target end-effector pose
     * @return 6×1 error vector
     */
    Eigen::VectorXd compute_pose_error(
        const math::SE3& current,
        const math::SE3& target) const;

    /**
     * @brief Check convergence based on error and mode
     * @param error Pose error vector (6×1)
     * @return true if converged
     */
    bool check_convergence(const Eigen::VectorXd& error) const;

    /**
     * @brief Compute damped least squares step
     *
     * Solves: Δq = (J^TJ + λ²I)^(-1) J^T * e
     *
     * @param J Jacobian matrix
     * @param error Pose error vector
     * @param damping Damping factor (may be adaptive)
     * @return Joint velocity Δq
     */
    Eigen::VectorXd compute_dls_step(
        const Eigen::MatrixXd& J,
        const Eigen::VectorXd& error,
        double damping) const;

    /**
     * @brief Compute adaptive damping using Sugihara method
     *
     * λ = sqrt(||e||² + λ_base²)
     *
     * - Large error → large damping (stable, conservative)
     * - Small error → small damping (fast convergence)
     * - "Solvability-unconcerned" - works even without exact solution
     *
     * @param error Current error vector
     * @return Adaptive damping factor
     */
    double compute_adaptive_damping(const Eigen::VectorXd& error) const;
};

} // namespace kinematics
} // namespace robospace
