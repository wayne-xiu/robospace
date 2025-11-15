#include <robospace/kinematics/analytical_ik_base.hpp>
#include <sstream>
#include <limits>
#include <cmath>

namespace robospace {
namespace kinematics {

// ========== IKConfiguration Implementation ==========

IKConfiguration IKConfiguration::from_array(const std::array<int, 3>& config) {
    return IKConfiguration(
        static_cast<Shoulder>(config[0]),
        static_cast<Elbow>(config[1]),
        static_cast<Wrist>(config[2])
    );
}

IKConfiguration IKConfiguration::any() {
    return IKConfiguration(Shoulder::ANY, Elbow::ANY, Wrist::ANY);
}

bool IKConfiguration::matches(const IKConfiguration& other) const {
    // Check shoulder
    if (shoulder != Shoulder::ANY && other.shoulder != Shoulder::ANY) {
        if (shoulder != other.shoulder) return false;
    }

    // Check elbow
    if (elbow != Elbow::ANY && other.elbow != Elbow::ANY) {
        if (elbow != other.elbow) return false;
    }

    // Check wrist
    if (wrist != Wrist::ANY && other.wrist != Wrist::ANY) {
        if (wrist != other.wrist) return false;
    }

    return true;
}

int IKConfiguration::to_index() const {
    // Return -1 if any component is ANY
    if (shoulder == Shoulder::ANY ||
        elbow == Elbow::ANY ||
        wrist == Wrist::ANY) {
        return -1;
    }

    // Map to index: shoulder*4 + elbow*2 + wrist
    int s = static_cast<int>(shoulder);
    int e = static_cast<int>(elbow);
    int w = static_cast<int>(wrist);

    return s * 4 + e * 2 + w;
}

IKConfiguration IKConfiguration::from_index(int index) {
    if (index < 0 || index >= 8) {
        return IKConfiguration::any();
    }

    // Reverse mapping: index = shoulder*4 + elbow*2 + wrist
    int s = index / 4;
    int e = (index % 4) / 2;
    int w = index % 2;

    return IKConfiguration(
        static_cast<Shoulder>(s),
        static_cast<Elbow>(e),
        static_cast<Wrist>(w)
    );
}

std::string IKConfiguration::to_string() const {
    std::ostringstream oss;
    oss << "Config(";

    // Shoulder
    if (shoulder == Shoulder::LEFT) oss << "LEFT,";
    else if (shoulder == Shoulder::RIGHT) oss << "RIGHT,";
    else oss << "ANY,";

    // Elbow
    if (elbow == Elbow::UP) oss << "UP,";
    else if (elbow == Elbow::DOWN) oss << "DOWN,";
    else oss << "ANY,";

    // Wrist
    if (wrist == Wrist::NO_FLIP) oss << "NO_FLIP";
    else if (wrist == Wrist::FLIP) oss << "FLIP";
    else oss << "ANY";

    oss << ")";
    return oss.str();
}

// ========== AnalyticalIKSolution Implementation ==========

AnalyticalIKSolution AnalyticalIKSolution::invalid(const std::string& reason) {
    AnalyticalIKSolution sol;
    sol.valid = false;
    sol.failure_reason = reason;
    return sol;
}

// ========== AnalyticalIKSolver Implementation ==========

AnalyticalIKSolution AnalyticalIKSolver::solve(
    const math::SE3& target,
    const Eigen::VectorXd& q_seed) {

    // Get all solutions
    std::vector<AnalyticalIKSolution> all_solutions = solve_all(target);

    // Return invalid if no solutions
    if (all_solutions.empty()) {
        return AnalyticalIKSolution::invalid("No IK solutions found");
    }

    // Filter out invalid solutions
    std::vector<AnalyticalIKSolution> valid_solutions;
    for (const auto& sol : all_solutions) {
        if (sol.valid) {
            valid_solutions.push_back(sol);
        }
    }

    if (valid_solutions.empty()) {
        return AnalyticalIKSolution::invalid("All solutions are invalid");
    }

    // Find solution closest to seed
    double min_distance = std::numeric_limits<double>::infinity();
    size_t best_idx = 0;

    for (size_t i = 0; i < valid_solutions.size(); ++i) {
        double dist = joint_distance(valid_solutions[i].q, q_seed);
        if (dist < min_distance) {
            min_distance = dist;
            best_idx = i;
        }
    }

    return valid_solutions[best_idx];
}

AnalyticalIKSolution AnalyticalIKSolver::solve(
    const math::SE3& target,
    const IKConfiguration& preferred) {

    // Get all solutions
    std::vector<AnalyticalIKSolution> all_solutions = solve_all(target);

    // Return invalid if no solutions
    if (all_solutions.empty()) {
        return AnalyticalIKSolution::invalid("No IK solutions found");
    }

    // Find first solution matching configuration preference
    for (const auto& sol : all_solutions) {
        if (sol.valid && sol.configuration.matches(preferred)) {
            return sol;
        }
    }

    // No matching configuration found
    return AnalyticalIKSolution::invalid(
        "No solution found matching configuration: " + preferred.to_string());
}

double AnalyticalIKSolver::joint_distance(
    const Eigen::VectorXd& q1,
    const Eigen::VectorXd& q2) {

    if (q1.size() != q2.size()) {
        return std::numeric_limits<double>::infinity();
    }

    // Euclidean distance in joint space
    return (q1 - q2).norm();
}

} // namespace kinematics
} // namespace robospace
