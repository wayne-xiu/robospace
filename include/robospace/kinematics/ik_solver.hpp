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
class AnalyticalIKSolver;
struct SearchParams;
struct IKConfiguration;

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

    // ========== SOLVER MODE ==========

    /**
     * @brief Solver selection mode
     *
     * Controls which solver type to use:
     * - AUTO: Try analytical first (if available), fallback to numerical
     * - ANALYTICAL: Use only analytical solver (fail if not available)
     * - NUMERICAL: Use only numerical solver
     */
    enum class SolverMode {
        AUTO,           ///< Automatic selection (analytical preferred)
        ANALYTICAL,     ///< Analytical only
        NUMERICAL       ///< Numerical only
    };

    // ========== PRIMARY SOLVE METHODS ==========

    /**
     * @brief Solve inverse kinematics for target pose
     *
     * Behavior depends on solver_mode:
     * - AUTO: Try analytical (if available), fallback to numerical
     * - ANALYTICAL: Use analytical only (fail if unavailable)
     * - NUMERICAL: Use numerical only
     *
     * @param target_pose Desired end-effector pose in base frame
     * @param q_seed Initial joint configuration (for numerical or solution selection)
     * @return IKResult with solution if successful
     *
     * @throws std::invalid_argument if q_seed size doesn't match robot DOF
     */
    IKResult solve(const math::SE3& target_pose, const Eigen::VectorXd& q_seed);

    /**
     * @brief Solve for ALL analytical IK solutions
     *
     * Only works if analytical solver is available.
     * Returns all possible solutions (typically up to 8 for spherical wrist robots).
     *
     * @param target_pose Desired end-effector pose
     * @return Vector of all solutions (empty if no analytical solver or unreachable)
     */
    std::vector<IKResult> solve_all(const math::SE3& target_pose);

    /**
     * @brief Solve with configuration preference
     *
     * Only works if analytical solver is available.
     * Returns solution matching preferred configuration (shoulder/elbow/wrist).
     *
     * @param target_pose Desired end-effector pose
     * @param q_seed Seed configuration (used if analytical solver not available)
     * @param preferred Preferred configuration
     * @return IKResult with preferred configuration
     */
    IKResult solve(const math::SE3& target_pose,
                   const Eigen::VectorXd& q_seed,
                   const IKConfiguration& preferred);

    // ========== CONFIGURATION METHODS ==========

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

    /**
     * @brief Set solver selection mode
     * @param mode Solver mode (AUTO, ANALYTICAL, NUMERICAL)
     */
    void set_solver_mode(SolverMode mode);

    /**
     * @brief Register an analytical IK solver for this robot
     *
     * Allows manual registration of analytical solver.
     * Normally, solvers are auto-detected in constructor.
     *
     * @param solver Analytical solver (takes ownership)
     */
    void register_analytical_solver(std::unique_ptr<AnalyticalIKSolver> solver);

    // ========== QUERY METHODS ==========

    /** @brief Check if analytical solver is available */
    bool has_analytical_solver() const;

    /** @brief Get name of analytical solver (if available) */
    std::string analytical_solver_name() const;

    /** @brief Get maximum number of analytical solutions */
    int max_analytical_solutions() const;

    /** @brief Get current solver mode */
    SolverMode solver_mode() const;

    // ========== NUMERICAL SOLVER GETTERS ==========

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

    // Solvers
    std::unique_ptr<NumericalIKSolver> numerical_solver_;
    std::unique_ptr<AnalyticalIKSolver> analytical_solver_;

    // Solver mode
    SolverMode solver_mode_ = SolverMode::AUTO;

    /**
     * @brief Auto-detect and create analytical solver for robot
     *
     * Tries to match robot against known analytical solver patterns:
     * - Spherical wrist robots
     * - OPW kinematics
     * - Other recognized patterns
     *
     * @return Analytical solver if detected, nullptr otherwise
     */
    std::unique_ptr<AnalyticalIKSolver> create_analytical_solver();
};

} // namespace kinematics
} // namespace robospace
