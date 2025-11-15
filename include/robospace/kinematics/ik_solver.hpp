#pragma once

#include <robospace/model/robot.hpp>
#include <robospace/math/SE3.hpp>
#include <Eigen/Dense>
#include <string>
#include <memory>

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

// Forward declarations
class NumericalIKSolver;
struct SearchParams;

/**
 * @brief Result of an IK solve attempt
 */
struct IKResult {
    bool success = false;                    ///< Whether IK converged to target
    Eigen::VectorXd q_solution;              ///< Joint configuration solution
    int iterations = 0;                      ///< Number of iterations used
    int searches = 0;                        ///< Number of random restart searches (numerical only)
    double final_error = 0.0;                ///< Final pose error (position + orientation)
    std::string message;                     ///< Optional status/error message
    bool from_analytical = false;            ///< True if solution from analytical solver

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
     * @brief Destructor
     *
     * Defined in .cpp to allow forward declaration of NumericalIKSolver
     */
    ~IKSolver();

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
     * @param tol Position tolerance (default: 1e-4 m)
     */
    void set_position_tolerance(double tol);

    /**
     * @brief Set orientation error tolerance (radians)
     * @param tol Orientation tolerance (default: 1e-3 rad)
     */
    void set_orientation_tolerance(double tol);

    /**
     * @brief Set maximum iterations
     * @param max_iter Maximum iterations (default: 30 per search)
     */
    void set_max_iterations(int max_iter);

    /**
     * @brief Set damping factor for DLS
     * @param damping Damping factor λ (default: 0.01)
     *
     * Larger values: More stable near singularities, slower convergence
     * Smaller values: Faster convergence, less stable near singularities
     */
    void set_damping(double damping);

    /**
     * @brief Set IK solver mode
     * @param mode Solver mode (FULL_POSE, POSITION_ONLY, ORIENTATION_ONLY)
     */
    void set_mode(IKMode mode);

    /**
     * @brief Set joint update step size
     * @param alpha Step size (default: 1.0)
     *
     * Smaller values: More conservative, slower but more stable
     * Larger values: Faster but may overshoot
     */
    void set_step_size(double alpha);

    /**
     * @brief Set error gain
     * @param gain Error gain factor (default: 0.5)
     *
     * Scales the error before computing joint update.
     * Larger values: Faster convergence but may overshoot
     * Smaller values: More stable but slower
     */
    void set_error_gain(double gain);

    /**
     * @brief Set maximum number of random restart searches
     * @param max_searches Maximum searches (default: 100)
     *
     * Random restart strategy improves success rate dramatically.
     * With 100 searches, success probability >99.99% (vs ~10% single attempt)
     */
    void set_max_searches(int max_searches);

    /**
     * @brief Set random seed for reproducibility
     * @param seed Random seed (0 = use random device)
     */
    void set_random_seed(uint32_t seed);

    /**
     * @brief Enable/disable random restart strategy
     * @param enable True to enable random restart
     */
    void set_use_random_restart(bool enable);

    /**
     * @brief Enable/disable adaptive damping (Sugihara method)
     * @param enable True to enable adaptive damping
     */
    void set_use_adaptive_damping(bool enable);

    // Getters

    double position_tolerance() const;
    double orientation_tolerance() const;
    int max_iterations() const;
    int max_searches() const;
    double damping() const;
    IKMode mode() const;
    double step_size() const;
    bool use_random_restart() const;
    bool use_adaptive_damping() const;

private:
    const model::Robot& robot_;

    // Numerical solver (uses improved algorithm with random restart)
    std::unique_ptr<NumericalIKSolver> numerical_solver_;
};

} // namespace kinematics
} // namespace robospace
