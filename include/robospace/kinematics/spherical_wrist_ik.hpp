#pragma once

#include <robospace/kinematics/analytical_ik_base.hpp>
#include <robospace/model/robot.hpp>
#include <Eigen/Dense>

namespace robospace {
namespace kinematics {

/**
 * @brief Generic analytical IK solver for 6-DOF spherical wrist robots
 *
 * A spherical wrist robot has the last 3 joint axes intersecting at a single point.
 * This allows geometric decoupling:
 * - Joints 1-3: Position IK (wrist center)
 * - Joints 4-6: Orientation IK (end-effector rotation)
 *
 * This is the most common industrial robot configuration, including:
 * - Universal Robots (UR3, UR5, UR10, UR16)
 * - ABB IRB series
 * - KUKA KR series
 * - Fanuc R series
 * - Yaskawa Motoman
 * - Many others
 *
 * Algorithm:
 * 1. Compute wrist center position from target pose
 * 2. Solve position IK for joints 1-3 (up to 4 solutions)
 * 3. For each position solution:
 *    - Compute required wrist orientation
 *    - Solve orientation IK for joints 4-6 (up to 2 solutions)
 * 4. Return all valid combinations (up to 8 total)
 *
 * References:
 * - Paul & Zhang: "Computationally Efficient Kinematics"
 * - Siciliano et al: "Robotics: Modelling, Planning and Control"
 * - Lynch & Park: "Modern Robotics"
 */
class SphericalWristIK : public AnalyticalIKSolver {
public:
    /**
     * @brief Construct spherical wrist solver
     * @param robot Robot model (must be 6-DOF with spherical wrist)
     * @throws std::invalid_argument if robot is not compatible
     */
    explicit SphericalWristIK(const model::Robot& robot);

    /**
     * @brief Solve for all IK solutions (up to 8)
     * @param target Target end-effector pose
     * @return Vector of all valid solutions
     */
    std::vector<AnalyticalIKSolution> solve_all(const math::SE3& target) override;

    /**
     * @brief Maximum solutions for spherical wrist
     * @return 8 (2^3 configurations: shoulder, elbow, wrist)
     */
    int max_solutions() const override { return 8; }

    /**
     * @brief Check if robot has spherical wrist structure
     * @param robot Robot to check
     * @return true if compatible
     */
    bool supports_robot(const model::Robot& robot) const override;

    /**
     * @brief Solver name for debugging
     */
    std::string solver_name() const override { return "SphericalWristIK"; }

private:
    const model::Robot& robot_;

    /**
     * @brief Wrist offset distance (d6 in DH notation)
     *
     * Distance from wrist center to end-effector along last joint axis
     */
    double wrist_offset_;

    /**
     * @brief Check if robot has spherical wrist geometry
     *
     * Verifies that axes 4, 5, 6 intersect at a point.
     * Uses joint transforms to check intersection within tolerance.
     *
     * @param robot Robot to check
     * @param wrist_offset Output: wrist offset if spherical
     * @return true if spherical wrist detected
     */
    static bool detect_spherical_wrist(const model::Robot& robot, double& wrist_offset);

    /**
     * @brief Compute wrist center position from target pose
     *
     * P_wrist = P_ee - d6 * approach_vector
     * where approach_vector is the z-axis of end-effector frame
     *
     * @param target Target end-effector pose
     * @return Wrist center position in base frame
     */
    Eigen::Vector3d compute_wrist_center(const math::SE3& target) const;

    /**
     * @brief Solve position IK for first 3 joints
     *
     * Finds joint angles q1, q2, q3 that place wrist center at target position.
     * Returns up to 4 solutions (shoulder left/right × elbow up/down).
     *
     * @param wrist_pos Target wrist center position
     * @return Vector of (q1, q2, q3) triplets
     */
    std::vector<Eigen::Vector3d> solve_position_ik(const Eigen::Vector3d& wrist_pos);

    /**
     * @brief Solve orientation IK for last 3 joints
     *
     * Given first 3 joint angles, finds q4, q5, q6 to achieve target orientation.
     * Returns up to 2 solutions (wrist flip/no-flip).
     *
     * @param R_target Target end-effector orientation
     * @param q_123 First 3 joint angles
     * @return Vector of (q4, q5, q6) triplets
     */
    std::vector<Eigen::Vector3d> solve_orientation_ik(
        const Eigen::Matrix3d& R_target,
        const Eigen::Vector3d& q_123);

    /**
     * @brief Determine configuration from joint angles
     *
     * Classifies solution into shoulder/elbow/wrist configuration
     *
     * @param q Full 6-joint configuration
     * @return Configuration classification
     */
    IKConfiguration determine_configuration(const Eigen::VectorXd& q) const;

    /**
     * @brief Normalize angle to [-π, π]
     */
    static double normalize_angle(double angle);
};

} // namespace kinematics
} // namespace robospace
