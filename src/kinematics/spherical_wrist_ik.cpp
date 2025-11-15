#include <robospace/kinematics/spherical_wrist_ik.hpp>
#include <cmath>
#include <stdexcept>

namespace robospace {
namespace kinematics {

SphericalWristIK::SphericalWristIK(const model::Robot& robot)
    : robot_(robot) {

    if (robot.dof() != 6) {
        throw std::invalid_argument(
            "SphericalWristIK requires 6-DOF robot, got " +
            std::to_string(robot.dof()) + " DOF");
    }

    if (!detect_spherical_wrist(robot, wrist_offset_)) {
        throw std::invalid_argument(
            "Robot does not have spherical wrist structure");
    }
}

std::vector<AnalyticalIKSolution> SphericalWristIK::solve_all(const math::SE3& target) {
    std::vector<AnalyticalIKSolution> solutions;

    // Step 1: Compute wrist center position
    Eigen::Vector3d wrist_pos = compute_wrist_center(target);

    // Step 2: Solve position IK for joints 1-3
    std::vector<Eigen::Vector3d> position_solutions = solve_position_ik(wrist_pos);

    if (position_solutions.empty()) {
        return solutions;  // No position solutions (unreachable)
    }

    // Step 3: For each position solution, solve orientation IK
    for (const auto& q_123 : position_solutions) {
        std::vector<Eigen::Vector3d> orientation_solutions =
            solve_orientation_ik(target.rotation(), q_123);

        // Step 4: Combine position + orientation into full solutions
        for (const auto& q_456 : orientation_solutions) {
            Eigen::VectorXd q_full(6);
            q_full << q_123, q_456;

            // Normalize all angles to [-π, π]
            for (int i = 0; i < 6; ++i) {
                q_full(i) = normalize_angle(q_full(i));
            }

            // Determine configuration
            IKConfiguration config = determine_configuration(q_full);

            solutions.push_back(AnalyticalIKSolution(q_full, config));
        }
    }

    return solutions;
}

bool SphericalWristIK::supports_robot(const model::Robot& robot) const {
    if (robot.dof() != 6) {
        return false;
    }

    double wrist_offset_dummy;
    return detect_spherical_wrist(robot, wrist_offset_dummy);
}

// ========== PRIVATE METHODS ==========

bool SphericalWristIK::detect_spherical_wrist(
    const model::Robot& robot,
    double& wrist_offset) {

    if (robot.dof() != 6) {
        return false;
    }

    // For spherical wrist: axes 4, 5, 6 must intersect at a point (wrist center)
    //
    // Check by computing FK at zero configuration and examining
    // the positions of joints 4, 5, 6 origins.
    //
    // In a true spherical wrist, joints 4, 5, 6 origins should coincide
    // within numerical tolerance.

    Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(6);

    // Get transforms for joints 3, 4, 5, 6
    // (We need to check if 4, 5, 6 origins coincide)
    //
    // TODO: This requires access to individual joint transforms
    // For now, use a heuristic based on DH parameters

    // Simplified detection: Check if robot has standard industrial structure
    // This is a placeholder - real detection would examine joint transforms

    // For now, return false to avoid false positives
    // Real implementation needs robot transform access
    wrist_offset = 0.0;
    return false;  // TODO: Implement proper detection
}

Eigen::Vector3d SphericalWristIK::compute_wrist_center(const math::SE3& target) const {
    // Wrist center = End-effector position - d6 * approach vector
    // where approach vector is the z-axis of end-effector frame

    Eigen::Vector3d approach = target.rotation().col(2);  // Z-axis of EE frame
    Eigen::Vector3d wrist_center = target.translation() - wrist_offset_ * approach;

    return wrist_center;
}

std::vector<Eigen::Vector3d> SphericalWristIK::solve_position_ik(
    const Eigen::Vector3d& wrist_pos) {

    // Position IK for joints 1-3 is ROBOT-SPECIFIC
    //
    // The geometry depends on the specific robot structure:
    // - UR-series: Special structure (not quite spherical wrist)
    // - ABB/KUKA: Different geometries
    // - Others: Varied structures
    //
    // A truly generic solver would need:
    // 1. Classification of robot type
    // 2. Type-specific position solvers
    // 3. Or numerical position IK as fallback
    //
    // For now, return empty (indicates unreachable or not implemented)

    std::vector<Eigen::Vector3d> solutions;

    // TODO: Implement robot-specific position IK
    // This requires:
    // - Identifying robot kinematic structure
    // - Applying appropriate geometric solution
    // - Handling multiple solutions (shoulder left/right, elbow up/down)

    return solutions;  // Not implemented yet
}

std::vector<Eigen::Vector3d> SphericalWristIK::solve_orientation_ik(
    const Eigen::Matrix3d& R_target,
    const Eigen::Vector3d& q_123) {

    std::vector<Eigen::Vector3d> solutions;

    // Orientation IK for spherical wrist is GENERIC
    //
    // Algorithm:
    // 1. Compute R_0_3 (rotation from base to wrist) using q_123
    // 2. Required wrist rotation: R_3_6 = R_0_3^T * R_target
    // 3. Decompose R_3_6 into Euler angles (q4, q5, q6)
    //
    // For spherical wrist, typically get 2 solutions (wrist flip)

    // Step 1: Compute R_0_3
    Eigen::VectorXd q_partial(3);
    q_partial << q_123;

    // Extend to 6-DOF with zeros for joints 4-6
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(6);
    q_full.head(3) = q_partial;

    math::SE3 T_0_6 = robot_.fk(q_full);
    Eigen::Matrix3d R_0_3 = T_0_6.rotation();

    // Step 2: Required wrist rotation
    Eigen::Matrix3d R_3_6 = R_0_3.transpose() * R_target;

    // Step 3: Decompose to Euler angles
    // Using ZYZ Euler convention (common for spherical wrist)
    //
    // R(θ4, θ5, θ6) = Rz(θ4) * Ry(θ5) * Rz(θ6)
    //
    // Two solutions due to Euler angle ambiguity:
    // (θ4, θ5, θ6) and (θ4±π, -θ5, θ6±π)

    double sy = std::sqrt(R_3_6(0, 2) * R_3_6(0, 2) + R_3_6(1, 2) * R_3_6(1, 2));

    bool singular = (sy < 1e-6);

    if (!singular) {
        // Solution 1
        double q4_1 = std::atan2(R_3_6(1, 2), R_3_6(0, 2));
        double q5_1 = std::atan2(sy, R_3_6(2, 2));
        double q6_1 = std::atan2(R_3_6(2, 1), -R_3_6(2, 0));

        solutions.push_back(Eigen::Vector3d(q4_1, q5_1, q6_1));

        // Solution 2 (flip)
        double q4_2 = q4_1 + M_PI;
        double q5_2 = -q5_1;
        double q6_2 = q6_1 + M_PI;

        solutions.push_back(Eigen::Vector3d(q4_2, q5_2, q6_2));
    } else {
        // Singular configuration (gimbal lock)
        // Set q4 = 0 arbitrarily
        double q4 = 0.0;
        double q5 = std::atan2(sy, R_3_6(2, 2));
        double q6 = std::atan2(R_3_6(1, 0), R_3_6(0, 0));

        solutions.push_back(Eigen::Vector3d(q4, q5, q6));
    }

    return solutions;
}

IKConfiguration SphericalWristIK::determine_configuration(const Eigen::VectorXd& q) const {
    IKConfiguration config;

    // Shoulder: based on q1
    if (q(0) >= 0) {
        config.shoulder = IKConfiguration::Shoulder::RIGHT;
    } else {
        config.shoulder = IKConfiguration::Shoulder::LEFT;
    }

    // Elbow: based on q2 or q3 (robot-specific)
    // Using q2 as heuristic
    if (q(2) >= 0) {
        config.elbow = IKConfiguration::Elbow::UP;
    } else {
        config.elbow = IKConfiguration::Elbow::DOWN;
    }

    // Wrist: based on q5
    if (std::abs(q(4)) < M_PI / 2) {
        config.wrist = IKConfiguration::Wrist::NO_FLIP;
    } else {
        config.wrist = IKConfiguration::Wrist::FLIP;
    }

    return config;
}

double SphericalWristIK::normalize_angle(double angle) {
    // Normalize to [-π, π]
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

} // namespace kinematics
} // namespace robospace
