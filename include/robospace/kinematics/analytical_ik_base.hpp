#pragma once

#include <robospace/math/SE3.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>

namespace robospace {
namespace model {
    class Robot;  // Forward declaration
}

namespace kinematics {

/**
 * @brief Robot configuration for 6-DOF manipulators
 *
 * Many 6-DOF industrial robots can reach the same target pose in multiple
 * configurations (typically 8 for spherical wrist robots).
 *
 * The configuration space is typically parameterized by:
 * - Shoulder: Left (0) or Right (1)
 * - Elbow: Up (0) or Down (1)
 * - Wrist: No-Flip (0) or Flip (1)
 *
 * Total: 2³ = 8 possible configurations
 */
struct IKConfiguration {
    /**
     * @brief Shoulder configuration
     *
     * For robots with a rotational base joint:
     * - LEFT: Base angle in [-π, 0]
     * - RIGHT: Base angle in [0, π]
     * - ANY: No preference
     */
    enum class Shoulder : int {
        LEFT = 0,
        RIGHT = 1,
        ANY = -1
    };

    /**
     * @brief Elbow configuration
     *
     * For robots with an elbow joint:
     * - UP: Elbow bent upward (positive angle)
     * - DOWN: Elbow bent downward (negative angle)
     * - ANY: No preference
     */
    enum class Elbow : int {
        UP = 0,
        DOWN = 1,
        ANY = -1
    };

    /**
     * @brief Wrist configuration
     *
     * For robots with a wrist flip:
     * - NO_FLIP: Wrist in primary orientation
     * - FLIP: Wrist rotated by π around approach axis
     * - ANY: No preference
     */
    enum class Wrist : int {
        NO_FLIP = 0,
        FLIP = 1,
        ANY = -1
    };

    Shoulder shoulder = Shoulder::ANY;
    Elbow elbow = Elbow::ANY;
    Wrist wrist = Wrist::ANY;

    /** @brief Default constructor (all ANY) */
    IKConfiguration() = default;

    /** @brief Construct with specific configuration */
    IKConfiguration(Shoulder s, Elbow e, Wrist w)
        : shoulder(s), elbow(e), wrist(w) {}

    /**
     * @brief Create configuration from array [shoulder, elbow, wrist]
     * @param config Array of 3 integers (0 or 1 for each)
     */
    static IKConfiguration from_array(const std::array<int, 3>& config);

    /**
     * @brief Create "any" configuration (no preferences)
     */
    static IKConfiguration any();

    /**
     * @brief Check if this configuration matches another
     *
     * Matching rules:
     * - ANY matches anything
     * - Otherwise values must be equal
     *
     * @param other Configuration to compare against
     * @return true if configurations match
     */
    bool matches(const IKConfiguration& other) const;

    /**
     * @brief Convert configuration to index (0-7)
     *
     * Maps configuration to unique index:
     * index = shoulder*4 + elbow*2 + wrist
     *
     * @return Index in range [0, 7], or -1 if any component is ANY
     */
    int to_index() const;

    /**
     * @brief Create configuration from index (0-7)
     * @param index Configuration index
     */
    static IKConfiguration from_index(int index);

    /**
     * @brief String representation
     */
    std::string to_string() const;
};

/**
 * @brief Result of a single analytical IK solution
 *
 * Contains joint configuration and associated metadata
 */
struct AnalyticalIKSolution {
    Eigen::VectorXd q;                          ///< Joint configuration
    IKConfiguration configuration;              ///< Robot configuration (shoulder/elbow/wrist)
    bool valid = true;                          ///< Whether solution is valid
    std::string failure_reason;                 ///< Reason for failure (if !valid)

    AnalyticalIKSolution() = default;

    AnalyticalIKSolution(const Eigen::VectorXd& q_,
                        const IKConfiguration& config_ = IKConfiguration())
        : q(q_), configuration(config_), valid(true) {}

    /**
     * @brief Create invalid solution with reason
     */
    static AnalyticalIKSolution invalid(const std::string& reason);
};

/**
 * @brief Base class for analytical IK solvers
 *
 * Analytical IK solvers provide closed-form solutions for specific robot
 * kinematic structures. They are:
 * - Extremely fast (typically < 10 μs)
 * - Deterministic (same input → same output)
 * - Complete (find ALL solutions)
 * - Exact (within numerical precision)
 *
 * Common analytical patterns:
 * - Spherical wrist robots (last 3 axes intersect)
 * - OPW kinematics (ortho-parallel wrist)
 * - Puma-style robots
 * - Custom kinematic structures
 *
 * Each derived solver should:
 * 1. Implement supports_robot() to detect compatible robots
 * 2. Implement solve_all() to return all solutions
 * 3. Optionally override solve() for optimized single-solution case
 */
class AnalyticalIKSolver {
public:
    virtual ~AnalyticalIKSolver() = default;

    /**
     * @brief Solve for ALL possible IK solutions
     *
     * Returns all valid solutions for the given target pose.
     * For a typical 6-DOF spherical wrist robot, this returns up to 8 solutions.
     *
     * @param target Target end-effector pose
     * @return Vector of all valid solutions (may be empty if unreachable)
     */
    virtual std::vector<AnalyticalIKSolution> solve_all(const math::SE3& target) = 0;

    /**
     * @brief Solve for single solution closest to seed configuration
     *
     * Default implementation:
     * 1. Calls solve_all() to get all solutions
     * 2. Returns solution closest to q_seed (by joint space distance)
     *
     * Derived classes may override for more efficient implementation.
     *
     * @param target Target end-effector pose
     * @param q_seed Seed configuration (for selecting closest solution)
     * @return Single best solution
     */
    virtual AnalyticalIKSolution solve(const math::SE3& target,
                                       const Eigen::VectorXd& q_seed);

    /**
     * @brief Solve for single solution with configuration preference
     *
     * Default implementation:
     * 1. Calls solve_all() to get all solutions
     * 2. Filters by configuration preference
     * 3. Returns first matching solution
     *
     * @param target Target end-effector pose
     * @param preferred Preferred configuration (shoulder/elbow/wrist)
     * @return Solution matching preference, or invalid if none match
     */
    virtual AnalyticalIKSolution solve(const math::SE3& target,
                                       const IKConfiguration& preferred);

    /**
     * @brief Maximum number of solutions this solver can return
     *
     * Typical values:
     * - Spherical wrist 6-DOF: 8 solutions
     * - General 6-DOF: up to 16 solutions
     * - Special cases: varies
     *
     * @return Maximum number of solutions
     */
    virtual int max_solutions() const = 0;

    /**
     * @brief Check if this solver supports a given robot
     *
     * Should verify:
     * - DOF matches expected
     * - Kinematic structure matches pattern (e.g., spherical wrist)
     * - DH parameters are compatible
     *
     * @param robot Robot to check
     * @return true if this solver can solve IK for the robot
     */
    virtual bool supports_robot(const model::Robot& robot) const = 0;

    /**
     * @brief Get name/type of this solver (for debugging/logging)
     */
    virtual std::string solver_name() const = 0;

protected:
    /**
     * @brief Compute joint space distance between two configurations
     *
     * Used to find solution closest to seed.
     * Default: Euclidean distance in joint space
     *
     * @param q1 First configuration
     * @param q2 Second configuration
     * @return Distance metric
     */
    static double joint_distance(const Eigen::VectorXd& q1,
                                 const Eigen::VectorXd& q2);
};

} // namespace kinematics
} // namespace robospace
