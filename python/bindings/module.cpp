#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Forward declarations
void bind_math(py::module_& m);
void bind_model(py::module_& m);
void bind_kinematics(py::module_& m);

PYBIND11_MODULE(_robospace_core, m) {
    m.doc() = "RoboSpace - Robot kinematics and dynamics library";

    // Bind math types (SE3, SO3, Rotation, Transform)
    bind_math(m);

    // Bind model types (Robot, Link, Joint, Tool, Frame)
    bind_model(m);

    // Bind kinematics (IKSolver, IKResult, IKMode)
    bind_kinematics(m);
}
