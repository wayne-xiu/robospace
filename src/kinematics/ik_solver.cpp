#include <robospace/kinematics/ik_solver.hpp>
#include <robospace/kinematics/numerical_ik.hpp>
#include <robospace/kinematics/analytical_ik_base.hpp>
#include <robospace/kinematics/spherical_wrist_ik.hpp>

namespace robospace {
namespace kinematics {

IKSolver::IKSolver(const model::Robot& robot)
    : robot_(robot),
      numerical_solver_(std::make_unique<NumericalIKSolver>(robot)),
      analytical_solver_(create_analytical_solver()) {
}

IKSolver::~IKSolver() = default;

// ========== PRIMARY SOLVE METHODS ==========

IKResult IKSolver::solve(const math::SE3& target_pose, const Eigen::VectorXd& q_seed) {
    // AUTO mode: try analytical first, fallback to numerical
    if (solver_mode_ == SolverMode::AUTO) {
        if (analytical_solver_) {
            // Use analytical solver
            AnalyticalIKSolution analytical_sol = analytical_solver_->solve(target_pose, q_seed);

            if (analytical_sol.valid) {
                IKResult result;
                result.success = true;
                result.q_solution = analytical_sol.q;
                result.from_analytical = true;
                result.iterations = 0;
                result.message = "Analytical IK solution";
                return result;
            }
            // Analytical failed, fall through to numerical
        }

        // No analytical solver or it failed - use numerical
        return numerical_solver_->solve(target_pose, q_seed);
    }

    // ANALYTICAL mode: analytical only
    if (solver_mode_ == SolverMode::ANALYTICAL) {
        if (!analytical_solver_) {
            IKResult result;
            result.success = false;
            result.message = "Analytical solver not available for this robot";
            return result;
        }

        AnalyticalIKSolution analytical_sol = analytical_solver_->solve(target_pose, q_seed);

        IKResult result;
        result.success = analytical_sol.valid;
        result.q_solution = analytical_sol.q;
        result.from_analytical = true;
        result.iterations = 0;
        result.message = analytical_sol.valid ? "Analytical IK solution" : analytical_sol.failure_reason;
        return result;
    }

    // NUMERICAL mode: numerical only
    return numerical_solver_->solve(target_pose, q_seed);
}

std::vector<IKResult> IKSolver::solve_all(const math::SE3& target_pose) {
    std::vector<IKResult> results;

    if (!analytical_solver_) {
        // No analytical solver available
        return results;
    }

    // Get all analytical solutions
    std::vector<AnalyticalIKSolution> analytical_solutions = analytical_solver_->solve_all(target_pose);

    // Convert to IKResult format
    for (const auto& sol : analytical_solutions) {
        if (sol.valid) {
            IKResult result;
            result.success = true;
            result.q_solution = sol.q;
            result.from_analytical = true;
            result.iterations = 0;
            result.message = "Analytical IK solution";
            results.push_back(result);
        }
    }

    return results;
}

IKResult IKSolver::solve(const math::SE3& target_pose,
                         const Eigen::VectorXd& q_seed,
                         const IKConfiguration& preferred) {

    if (!analytical_solver_) {
        // No analytical solver - use numerical
        return numerical_solver_->solve(target_pose, q_seed);
    }

    // Use analytical solver with configuration preference
    AnalyticalIKSolution analytical_sol = analytical_solver_->solve(target_pose, preferred);

    IKResult result;
    result.success = analytical_sol.valid;
    result.q_solution = analytical_sol.q;
    result.from_analytical = true;
    result.iterations = 0;
    result.message = analytical_sol.valid ? "Analytical IK solution" : analytical_sol.failure_reason;

    return result;
}

// ========== CONFIGURATION METHODS ==========

void IKSolver::set_position_tolerance(double tol) {
    numerical_solver_->set_position_tolerance(tol);
}

void IKSolver::set_orientation_tolerance(double tol) {
    numerical_solver_->set_orientation_tolerance(tol);
}

void IKSolver::set_max_iterations(int max_iter) {
    auto params = numerical_solver_->search_params();
    params.max_iterations = max_iter;
    numerical_solver_->set_search_params(params);
}

void IKSolver::set_damping(double damping) {
    numerical_solver_->set_damping(damping);
}

void IKSolver::set_mode(IKMode mode) {
    numerical_solver_->set_mode(mode);
}

void IKSolver::set_step_size(double alpha) {
    numerical_solver_->set_step_size(alpha);
}

void IKSolver::set_error_gain(double gain) {
    numerical_solver_->set_error_gain(gain);
}

void IKSolver::set_max_searches(int max_searches) {
    auto params = numerical_solver_->search_params();
    params.max_searches = max_searches;
    numerical_solver_->set_search_params(params);
}

void IKSolver::set_random_seed(uint32_t seed) {
    auto params = numerical_solver_->search_params();
    params.seed = seed;
    numerical_solver_->set_search_params(params);
}

void IKSolver::set_use_random_restart(bool enable) {
    auto params = numerical_solver_->search_params();
    params.use_random_restart = enable;
    numerical_solver_->set_search_params(params);
}

void IKSolver::set_use_adaptive_damping(bool enable) {
    auto params = numerical_solver_->search_params();
    params.use_adaptive_damping = enable;
    numerical_solver_->set_search_params(params);
}

void IKSolver::set_solver_mode(SolverMode mode) {
    solver_mode_ = mode;
}

void IKSolver::register_analytical_solver(std::unique_ptr<AnalyticalIKSolver> solver) {
    analytical_solver_ = std::move(solver);
}

// ========== QUERY METHODS ==========

bool IKSolver::has_analytical_solver() const {
    return analytical_solver_ != nullptr;
}

std::string IKSolver::analytical_solver_name() const {
    if (analytical_solver_) {
        return analytical_solver_->solver_name();
    }
    return "none";
}

int IKSolver::max_analytical_solutions() const {
    if (analytical_solver_) {
        return analytical_solver_->max_solutions();
    }
    return 0;
}

IKSolver::SolverMode IKSolver::solver_mode() const {
    return solver_mode_;
}

// ========== NUMERICAL SOLVER GETTERS ==========

double IKSolver::position_tolerance() const {
    return numerical_solver_->position_tolerance();
}

double IKSolver::orientation_tolerance() const {
    return numerical_solver_->orientation_tolerance();
}

int IKSolver::max_iterations() const {
    return numerical_solver_->search_params().max_iterations;
}

int IKSolver::max_searches() const {
    return numerical_solver_->search_params().max_searches;
}

double IKSolver::damping() const {
    return numerical_solver_->damping();
}

IKMode IKSolver::mode() const {
    return numerical_solver_->mode();
}

double IKSolver::step_size() const {
    return numerical_solver_->step_size();
}

bool IKSolver::use_random_restart() const {
    return numerical_solver_->search_params().use_random_restart;
}

bool IKSolver::use_adaptive_damping() const {
    return numerical_solver_->search_params().use_adaptive_damping;
}

// ========== PRIVATE METHODS ==========

std::unique_ptr<AnalyticalIKSolver> IKSolver::create_analytical_solver() {
    // Auto-detect analytical solver for robot

    // Try spherical wrist solver (most common industrial robot pattern)
    try {
        auto spherical_solver = std::make_unique<SphericalWristIK>(robot_);
        if (spherical_solver->supports_robot(robot_)) {
            return spherical_solver;
        }
    } catch (const std::exception&) {
        // Not a spherical wrist robot, continue checking
    }

    // TODO: Try other patterns:
    // - OPW kinematics
    // - Other specialized solvers

    // No analytical solver available
    return nullptr;
}

} // namespace kinematics
} // namespace robospace
