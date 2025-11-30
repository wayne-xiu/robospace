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
    // Strategy: Use FK to compute wrist joint positions at zero config
    // Check if joints 4, 5, 6 origins are approximately coincident

    Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(6);

    // Get transforms to each joint at zero configuration
    std::vector<math::SE3> transforms = robot.fk_all(q_zero);

    // For a 6-DOF robot, we have 6 joint transforms
    if (transforms.size() < 6) {
        return false;
    }

    // Extract positions of joints 4, 5, 6 (indices 3, 4, 5)
    Eigen::Vector3d pos_j4 = transforms[3].translation();
    Eigen::Vector3d pos_j5 = transforms[4].translation();
    Eigen::Vector3d pos_j6 = transforms[5].translation();

    // Check if wrist joints are approximately at same position
    const double tolerance = 0.01;  // 10mm tolerance for "same point"

    double dist_45 = (pos_j5 - pos_j4).norm();
    double dist_46 = (pos_j6 - pos_j4).norm();

    // If joints 4, 5, 6 are close together, it's a spherical wrist
    // (d45 small and d46 small means they intersect)
    if (dist_45 > tolerance || dist_46 > tolerance) {
        return false;  // Not a spherical wrist
    }

    // Compute wrist offset as distance from wrist center to end-effector
    // End-effector is at index 5 (6th transform)
    Eigen::Vector3d ee_pos = transforms[5].translation();
    Eigen::Vector3d wrist_center = pos_j4;  // Use j4 position as wrist center

    // Wrist offset is the distance along the end-effector's approach axis
    Eigen::Vector3d approach = transforms[5].rotation().col(2);  // Z-axis
    Eigen::Vector3d wrist_to_ee = ee_pos - wrist_center;

    // Project onto approach axis to get wrist offset
    wrist_offset = std::abs(wrist_to_ee.dot(approach));

    // Minimum wrist offset (some robots have tool flange at wrist center)
    if (wrist_offset < 0.01) {
        wrist_offset = 0.01;  // 10mm minimum to avoid numerical issues
    }

    return true;  // Detected spherical wrist structure
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

    std::vector<Eigen::Vector3d> solutions;

    // Extract geometric parameters from robot using FK
    // For position IK, we need:
    // - d1: Base height offset
    // - a1: Base radial offset
    // - a2: Upper arm length (shoulder to elbow)
    // - a3: Forearm length (elbow to wrist)

    Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(6);
    std::vector<math::SE3> transforms = robot_.fk_all(q_zero);

    // Extract joint positions
    Eigen::Vector3d pos_j1 = transforms[0].translation();  // Shoulder pan
    Eigen::Vector3d pos_j2 = transforms[1].translation();  // Shoulder lift
    Eigen::Vector3d pos_j3 = transforms[2].translation();  // Elbow
    Eigen::Vector3d pos_wrist = transforms[3].translation();  // Wrist (j4)

    // Compute geometric parameters
    double d1 = pos_j1.z();  // Base height (Z offset of first joint)

    // a1: Radial offset from base to shoulder axis
    double a1 = std::sqrt(pos_j2.x() * pos_j2.x() + pos_j2.y() * pos_j2.y());

    // a2: Distance from shoulder to elbow
    double a2 = (pos_j3 - pos_j2).norm();

    // a3: Distance from elbow to wrist
    double a3 = (pos_wrist - pos_j3).norm();

    double wx = wrist_pos.x();
    double wy = wrist_pos.y();
    double wz = wrist_pos.z();

    // Step 1: Solve for q1 (base rotation) - 2 solutions
    std::vector<double> q1_candidates;

    double r_xy = std::sqrt(wx * wx + wy * wy);

    if (r_xy < 1e-6) {
        // Singularity: wrist directly above base
        q1_candidates.push_back(0.0);  // Arbitrary choice
    } else {
        // Solution 1: Front configuration
        q1_candidates.push_back(std::atan2(wy, wx));
        // Solution 2: Back configuration (rotated by π)
        q1_candidates.push_back(normalize_angle(std::atan2(wy, wx) + M_PI));
    }

    // Step 2: For each q1, solve for q2 and q3
    for (double q1 : q1_candidates) {
        // Project wrist position into arm plane
        double r = r_xy - a1;  // Account for base offset
        double s = wz - d1;    // Account for base height

        double D_squared = r * r + s * s;
        double D = std::sqrt(D_squared);

        // Check reachability using triangle inequality
        double reach_max = a2 + a3;
        double reach_min = std::abs(a2 - a3);

        if (D > reach_max + 1e-3 || D < reach_min - 1e-3) {
            // Out of reach for this q1
            continue;
        }

        // Step 3: Solve for q3 (elbow angle) using law of cosines
        // cos(q3) = (D² - a2² - a3²) / (2*a2*a3)
        double cos_q3 = (D_squared - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);

        // Clamp to valid range to handle numerical errors
        cos_q3 = std::max(-1.0, std::min(1.0, cos_q3));

        // Two elbow solutions
        std::vector<double> q3_candidates;
        double sin_q3 = std::sqrt(1.0 - cos_q3 * cos_q3);

        if (sin_q3 > 1e-6) {
            q3_candidates.push_back(std::atan2(sin_q3, cos_q3));   // Elbow up
            q3_candidates.push_back(std::atan2(-sin_q3, cos_q3));  // Elbow down
        } else {
            // Fully extended or folded - only one solution
            q3_candidates.push_back(std::atan2(0.0, cos_q3));
        }

        // Step 4: For each q3, solve for q2 (shoulder angle)
        for (double q3 : q3_candidates) {
            // Using geometric relations:
            // beta = atan2(s, r)  - angle to wrist in arm plane
            // psi = atan2(a3*sin(q3), a2 + a3*cos(q3))  - angle from link 2 to wrist
            // q2 = beta - psi

            double beta = std::atan2(s, r);
            double psi = std::atan2(a3 * std::sin(q3), a2 + a3 * std::cos(q3));
            double q2 = beta - psi;

            // Store solution (q1, q2, q3)
            Eigen::Vector3d solution;
            solution << q1, q2, q3;
            solutions.push_back(solution);
        }
    }

    return solutions;
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
