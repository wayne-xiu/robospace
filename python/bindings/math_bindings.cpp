#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include "robospace/math/SE3.hpp"
#include "robospace/math/SO3.hpp"
#include "robospace/math/rotation.hpp"
#include "robospace/math/transform.hpp"

namespace py = pybind11;
using namespace robospace::math;

void bind_math(py::module_& m) {
    // EulerConvention enum (declare first)
    py::enum_<EulerConvention>(m, "EulerConvention")
        .value("XYZ", EulerConvention::XYZ)
        .value("ZYX", EulerConvention::ZYX)
        .value("ZYZ", EulerConvention::ZYZ)
        .value("XYX", EulerConvention::XYX)
        .export_values();

    // SE3 - Use shared_ptr for Eigen alignment
    py::class_<SE3>(m, "SE3", "SE(3) rigid transformation")
        .def(py::init<>())
        .def_static("Identity", &SE3::Identity)
        .def_static("FromMatrix", [](const Eigen::Matrix4d& mat) { return SE3(mat); }, py::arg("matrix"), "Construct SE3 from 4x4 matrix")
        .def("matrix", &SE3::matrix)
        .def("rotation", &SE3::rotation)
        .def("translation", &SE3::translation)
        .def("inverse", &SE3::inverse)
        .def("__mul__", [](const SE3& self, const SE3& other) {
            return self * other;
        });

    // SO3
    py::class_<SO3>(m, "SO3", "SO(3) rotation")
        .def(py::init<>())
        .def_static("Identity", &SO3::Identity)
        .def_static("FromAxisAngle", &SO3::FromAxisAngle,
                   py::arg("axis"), py::arg("angle"))
        .def_static("RotX", &SO3::RotX, py::arg("angle"))
        .def_static("RotY", &SO3::RotY, py::arg("angle"))
        .def_static("RotZ", &SO3::RotZ, py::arg("angle"))
        .def("matrix", &SO3::matrix)
        .def("inverse", &SO3::inverse)
        .def("__mul__", [](const SO3& self, const SO3& other) {
            return self * other;
        });

    // Rotation
    py::class_<Rotation>(m, "Rotation", "Classical rotation")
        .def(py::init<>())
        .def_static("Identity", &Rotation::Identity)
        .def_static("FromEuler",
                   py::overload_cast<double, double, double, EulerConvention>(&Rotation::FromEuler),
                   py::arg("roll"), py::arg("pitch"), py::arg("yaw"),
                   py::arg("convention") = EulerConvention::XYZ)
        .def_static("FromAxisAngle", &Rotation::FromAxisAngle,
                   py::arg("axis"), py::arg("angle"))
        .def("matrix", &Rotation::matrix)
        .def("quaternion", &Rotation::quaternion);

    // Transform
    py::class_<Transform>(m, "Transform", "4x4 homogeneous transformation")
        .def(py::init<>())
        .def_static("Identity", &Transform::Identity)
        .def_static("Translation",
                   py::overload_cast<double, double, double>(&Transform::Translation),
                   py::arg("x"), py::arg("y"), py::arg("z"))
        .def_static("RotX", &Transform::RotX, py::arg("angle"))
        .def_static("RotY", &Transform::RotY, py::arg("angle"))
        .def_static("RotZ", &Transform::RotZ, py::arg("angle"))
        .def("matrix", &Transform::matrix)
        .def("rotation", &Transform::rotation)
        .def("translation", &Transform::translation)
        .def("inverse", &Transform::inverse)
        .def("__mul__", [](const Transform& self, const Transform& other) {
            return self * other;
        });
}
