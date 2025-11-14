#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include "robospace/math/SE3.hpp"
#include "robospace/math/SO3.hpp"
#include "robospace/math/se3.hpp"
#include "robospace/math/so3.hpp"
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

    // se3 - Lie algebra element (twist)
    py::class_<se3>(m, "se3", "se(3) Lie algebra element (twist)")
        .def(py::init<>(), "Default constructor (zero twist)")
        .def_static("Zero", &se3::Zero, "Create zero twist")
        .def("vector", &se3::vector, "Get 6D vector representation [omega; v]")
        .def("omega", &se3::omega, "Get angular velocity (3D)")
        .def("v", &se3::v, "Get linear velocity (3D)")
        .def("bracket", &se3::bracket, "Get 4×4 bracket matrix representation")
        .def("__add__", [](const se3& self, const se3& other) {
            return self + other;
        })
        .def("__sub__", [](const se3& self, const se3& other) {
            return self - other;
        })
        .def("__mul__", [](const se3& self, double scalar) {
            return self * scalar;
        })
        .def("__rmul__", [](const se3& self, double scalar) {
            return scalar * self;
        })
        .def("__truediv__", [](const se3& self, double scalar) {
            return self / scalar;
        })
        .def("__neg__", [](const se3& self) {
            return -self;
        });

    // so3 - Lie algebra element (angular velocity)
    py::class_<so3>(m, "so3", "so(3) Lie algebra element (angular velocity)")
        .def(py::init<>(), "Default constructor (zero angular velocity)")
        .def_static("Zero", &so3::Zero, "Create zero angular velocity")
        .def("vector", &so3::vector, "Get 3D vector representation")
        .def("bracket", &so3::bracket, "Get 3×3 skew-symmetric matrix")
        .def("__add__", [](const so3& self, const so3& other) {
            return self + other;
        })
        .def("__sub__", [](const so3& self, const so3& other) {
            return self - other;
        })
        .def("__mul__", [](const so3& self, double scalar) {
            return self * scalar;
        })
        .def("__rmul__", [](const so3& self, double scalar) {
            return scalar * self;
        })
        .def("__truediv__", [](const so3& self, double scalar) {
            return self / scalar;
        })
        .def("__neg__", [](const so3& self) {
            return -self;
        });

    // Factory functions for creating se3 and so3 from numpy arrays
    m.def("make_se3", [](Eigen::Ref<const Eigen::Vector3d> omega,
                          Eigen::Ref<const Eigen::Vector3d> v) -> se3 {
        return se3(Eigen::Vector3d(omega), Eigen::Vector3d(v));
    }, py::arg("omega"), py::arg("v"),
       "Create se3 twist from angular and linear velocity");

    m.def("make_so3", [](Eigen::Ref<const Eigen::Vector3d> omega) -> so3 {
        return so3(Eigen::Vector3d(omega));
    }, py::arg("omega"),
       "Create so3 from angular velocity vector");

    // Exponential and logarithm maps
    m.def("exp_se3", &exp_se3, py::arg("xi"),
          "Exponential map: se(3) → SE(3)");
    m.def("log_SE3", &log_SE3, py::arg("g"),
          "Logarithm map: SE(3) → se(3)");
    m.def("exp_so3", &exp_so3, py::arg("omega"),
          "Exponential map: so(3) → SO(3)");
    m.def("log_SO3", &log_SO3, py::arg("R"),
          "Logarithm map: SO(3) → so(3)");
}
