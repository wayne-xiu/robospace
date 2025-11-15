#include <robospace/kinematics/ik_solver.hpp>
#include <robospace/kinematics/numerical_ik.hpp>

namespace robospace {
namespace kinematics {

IKSolver::IKSolver(const model::Robot& robot)
    : robot_(robot),
      numerical_solver_(std::make_unique<NumericalIKSolver>(robot)) {
}

IKSolver::~IKSolver() = default;

IKResult IKSolver::solve(const math::SE3& target_pose, const Eigen::VectorXd& q_seed) {
    // Delegate to numerical solver
    return numerical_solver_->solve(target_pose, q_seed);
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

// ========== GETTERS ==========

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

} // namespace kinematics
} // namespace robospace
