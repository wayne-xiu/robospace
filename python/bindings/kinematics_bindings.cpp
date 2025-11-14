#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "robospace/kinematics/ik_solver.hpp"

namespace py = pybind11;
using namespace robospace::kinematics;

void bind_kinematics(py::module_& m) {
    // IKMode enum
    py::enum_<IKMode>(m, "IKMode", "Inverse kinematics solver mode")
        .value("FULL_POSE", IKMode::FULL_POSE,
               "Minimize both position and orientation errors (6D)")
        .value("POSITION_ONLY", IKMode::POSITION_ONLY,
               "Minimize only position error (3D), ignore orientation")
        .value("ORIENTATION_ONLY", IKMode::ORIENTATION_ONLY,
               "Minimize only orientation error (3D), ignore position")
        .export_values();

    // IKResult struct
    py::class_<IKResult>(m, "IKResult", "Result of an IK solve attempt")
        .def(py::init<>(), "Create empty IK result")
        .def(py::init<bool, const Eigen::VectorXd&, int, double>(),
             py::arg("success"), py::arg("q_solution"),
             py::arg("iterations") = 0, py::arg("final_error") = 0.0,
             "Create IK result with values")
        .def_readwrite("success", &IKResult::success,
                      "Whether IK converged to target")
        .def_readwrite("q_solution", &IKResult::q_solution,
                      "Joint configuration solution")
        .def_readwrite("iterations", &IKResult::iterations,
                      "Number of iterations used")
        .def_readwrite("final_error", &IKResult::final_error,
                      "Final pose error (position + orientation)")
        .def_readwrite("message", &IKResult::message,
                      "Optional status/error message")
        .def("__repr__", [](const IKResult& self) {
            std::string status = self.success ? "SUCCESS" : "FAILED";
            return "<IKResult " + status +
                   ", iterations=" + std::to_string(self.iterations) +
                   ", error=" + std::to_string(self.final_error) + ">";
        });

    // IKSolver class
    py::class_<IKSolver>(m, "IKSolver",
        "Numerical inverse kinematics solver using Jacobian pseudoinverse\n\n"
        "Implements damped least squares (Levenberg-Marquardt style) IK:\n"
        "  Δq = J^T(JJ^T + λ²I)^(-1) * e\n\n"
        "Features:\n"
        "- Damped least squares for singularity robustness\n"
        "- Position-only, orientation-only, or full pose modes\n"
        "- Configurable tolerances and iteration limits\n"
        "- Convergence tracking\n\n"
        "Example:\n"
        "  >>> import robospace as rs\n"
        "  >>> import numpy as np\n"
        "  >>> robot = rs.Robot.from_urdf('ur5.urdf')\n"
        "  >>> solver = rs.IKSolver(robot)\n"
        "  >>> solver.set_mode(rs.IKMode.POSITION_ONLY)\n"
        "  >>> target = rs.SE3.Identity()\n"
        "  >>> target.set_translation(np.array([0.4, 0.2, 0.3]))\n"
        "  >>> result = solver.solve(target, robot.home())\n"
        "  >>> if result.success:\n"
        "  ...     print('IK solution:', result.q_solution)\n")
        .def(py::init<const robospace::model::Robot&>(),
             py::arg("robot"),
             "Construct IK solver for a robot",
             py::keep_alive<1, 2>())  // Keep robot alive as long as solver exists

        // Main solve method
        .def("solve",
             [](IKSolver& self,
                const robospace::math::SE3& target_pose,
                Eigen::Ref<const Eigen::VectorXd> q_seed) -> IKResult {
                 return self.solve(target_pose, q_seed);
             },
             py::arg("target_pose"), py::arg("q_seed"),
             "Solve inverse kinematics for target pose\n\n"
             "Args:\n"
             "    target_pose: Desired end-effector pose in base frame\n"
             "    q_seed: Initial joint configuration (seed)\n\n"
             "Returns:\n"
             "    IKResult with solution if successful\n\n"
             "Raises:\n"
             "    ValueError: if q_seed size doesn't match robot DOF")

        // Configuration methods
        .def("set_position_tolerance", &IKSolver::set_position_tolerance,
             py::arg("tolerance"),
             "Set position error tolerance in meters (default: 1e-6)")
        .def("set_orientation_tolerance", &IKSolver::set_orientation_tolerance,
             py::arg("tolerance"),
             "Set orientation error tolerance in radians (default: 1e-4)")
        .def("set_max_iterations", &IKSolver::set_max_iterations,
             py::arg("max_iterations"),
             "Set maximum iterations (default: 500)")
        .def("set_damping", &IKSolver::set_damping,
             py::arg("damping"),
             "Set damping factor λ for DLS (default: 0.1)\n\n"
             "Larger values: More stable near singularities, slower convergence\n"
             "Smaller values: Faster convergence, less stable near singularities")
        .def("set_mode", &IKSolver::set_mode,
             py::arg("mode"),
             "Set IK solver mode (FULL_POSE, POSITION_ONLY, ORIENTATION_ONLY)")
        .def("set_step_size", &IKSolver::set_step_size,
             py::arg("step_size"),
             "Set joint update step size (default: 0.5)\n\n"
             "Smaller values: More conservative, slower but more stable\n"
             "Larger values: Faster but may overshoot")

        // Getters
        .def("position_tolerance", &IKSolver::position_tolerance,
             "Get position tolerance")
        .def("orientation_tolerance", &IKSolver::orientation_tolerance,
             "Get orientation tolerance")
        .def("max_iterations", &IKSolver::max_iterations,
             "Get maximum iterations")
        .def("damping", &IKSolver::damping,
             "Get damping factor")
        .def("mode", &IKSolver::mode,
             "Get solver mode")
        .def("step_size", &IKSolver::step_size,
             "Get step size")

        .def("__repr__", [](const IKSolver& self) {
            return "<IKSolver max_iter=" + std::to_string(self.max_iterations()) +
                   ", damping=" + std::to_string(self.damping()) + ">";
        });
}
