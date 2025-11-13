#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "robospace/model/robot.hpp"
#include "robospace/model/link.hpp"
#include "robospace/model/joint.hpp"
#include "robospace/model/tool.hpp"
#include "robospace/model/frame.hpp"

namespace py = pybind11;
using namespace robospace::model;

void bind_model(py::module_& m) {
    // JointType enum
    py::enum_<JointType>(m, "JointType", "Type of robot joint")
        .value("REVOLUTE", JointType::REVOLUTE, "Revolute (rotational) joint")
        .value("PRISMATIC", JointType::PRISMATIC, "Prismatic (linear) joint")
        .value("CONTINUOUS", JointType::CONTINUOUS, "Continuous rotation joint")
        .value("FIXED", JointType::FIXED, "Fixed joint (no motion)")
        .export_values();

    // Link class
    py::class_<Link>(m, "Link", "Robot link with physical properties")
        .def(py::init<const std::string&>(),
             py::arg("name"),
             "Create link with name")
        .def("name", &Link::name, "Get link name")
        .def("mass", &Link::mass, "Get link mass")
        .def("set_mass", &Link::set_mass, py::arg("mass"), "Set link mass")
        .def("com", &Link::com, "Get center of mass")
        .def("set_com", &Link::set_com, py::arg("com"), "Set center of mass")
        .def("inertia", &Link::inertia, "Get inertia tensor")
        .def("set_inertia", &Link::set_inertia, py::arg("inertia"), "Set inertia tensor")
        .def("__repr__", [](const Link& self) {
            return "<Link '" + self.name() + "'>";
        });

    // Joint class
    py::class_<Joint>(m, "Joint", "Robot joint connecting links")
        .def(py::init<const std::string&, JointType, int, int>(),
             py::arg("name"), py::arg("type"),
             py::arg("parent_link_id"), py::arg("child_link_id"),
             "Create joint with name, type, and link IDs")
        .def("name", &Joint::name, "Get joint name")
        .def("type", &Joint::type, "Get joint type")
        .def("is_revolute", &Joint::is_revolute, "Check if revolute joint")
        .def("is_prismatic", &Joint::is_prismatic, "Check if prismatic joint")
        .def("is_fixed", &Joint::is_fixed, "Check if fixed joint")
        .def("has_position_limits", &Joint::has_position_limits, "Check if joint has position limits")
        .def("lower_limit", &Joint::lower_limit, "Get lower position limit")
        .def("upper_limit", &Joint::upper_limit, "Get upper position limit")
        .def("set_limits", &Joint::set_limits,
             py::arg("lower"), py::arg("upper"),
             "Set position limits")
        .def("axis", &Joint::axis, "Get joint axis")
        .def("set_axis", &Joint::set_axis, py::arg("axis"), "Set joint axis")
        .def("__repr__", [](const Joint& self) {
            std::string type_str;
            switch(self.type()) {
                case JointType::REVOLUTE: type_str = "REVOLUTE"; break;
                case JointType::PRISMATIC: type_str = "PRISMATIC"; break;
                case JointType::CONTINUOUS: type_str = "CONTINUOUS"; break;
                case JointType::FIXED: type_str = "FIXED"; break;
            }
            return "<Joint '" + self.name() + "' type=" + type_str + ">";
        });

    // Tool class
    py::class_<Tool>(m, "Tool", "End-effector tool with TCP")
        .def(py::init<const std::string&>(),
             py::arg("name"),
             "Create tool with name (TCP at origin)")
        .def(py::init<const std::string&, const robospace::math::SE3&>(),
             py::arg("name"), py::arg("tcp_pose"),
             "Create tool with name and TCP pose")
        .def("name", &Tool::name, "Get tool name")
        .def("tcp_pose", &Tool::tcp_pose, "Get TCP pose transformation")
        .def("set_tcp_pose", &Tool::set_tcp_pose,
             py::arg("tcp"),
             "Set TCP pose")
        .def("__repr__", [](const Tool& self) {
            return "<Tool '" + self.name() + "'>";
        });

    // Frame class
    py::class_<Frame>(m, "Frame", "Reference frame in robot scene")
        .def(py::init<const std::string&>(),
             py::arg("name"),
             "Create frame with name")
        .def("name", &Frame::name, "Get frame name")
        .def("__repr__", [](const Frame& self) {
            return "<Frame '" + self.name() + "'>";
        });

    // Robot class - Main API
    py::class_<Robot>(m, "Robot", "Robot manipulator with kinematics")
        .def(py::init<const std::string&>(),
             py::arg("name"),
             "Create robot with name")

        // Factory methods
        .def_static("from_urdf",
             py::overload_cast<const std::string&>(&Robot::from_urdf),
             py::arg("path"),
             "Load robot from URDF file")
        .def_static("from_urdf_string",
             py::overload_cast<const std::string&>(&Robot::from_urdf_string),
             py::arg("urdf_string"),
             "Load robot from URDF string")

        // Basic properties
        .def("name", &Robot::name, "Get robot name")
        .def("dof", &Robot::dof, "Get degrees of freedom")
        .def("num_links", &Robot::num_links, "Get number of links")
        .def("num_joints", &Robot::num_joints, "Get number of joints")

        // Access links/joints
        .def("link", py::overload_cast<const std::string&>(&Robot::link, py::const_),
             py::arg("name"),
             "Get link by name")
        .def("joint", py::overload_cast<const std::string&>(&Robot::joint, py::const_),
             py::arg("name"),
             "Get joint by name")
        .def("has_link", &Robot::has_link, py::arg("name"), "Check if link exists")
        .def("has_joint", &Robot::has_joint, py::arg("name"), "Check if joint exists")

        // Forward kinematics - Return matrix to avoid alignment issues
        .def("fk", [](const Robot& self, Eigen::Ref<const Eigen::VectorXd> q, const std::string& link_name) -> robospace::math::SE3 {
            return self.fk(q, link_name);
        }, py::arg("q"), py::arg("link_name"),
           "Compute forward kinematics for a specific link ()")
        .def("fk_all", [](const Robot& self, Eigen::Ref<const Eigen::VectorXd> q) {
            return self.fk_all(q);
        }, py::arg("q"),
           "Compute poses for all links")

        // Jacobian - also use lambdas for consistency
        .def("jacob0", [](const Robot& self, Eigen::Ref<const Eigen::VectorXd> q) {
            return self.jacob0(q);
        }, py::arg("q"),
           "Compute Jacobian in base frame")
        .def("jacobe", [](const Robot& self, Eigen::Ref<const Eigen::VectorXd> q) {
            return self.jacobe(q);
        }, py::arg("q"),
           "Compute Jacobian in end-effector frame")
        .def("manipulability", [](const Robot& self, Eigen::Ref<const Eigen::VectorXd> q) {
            return self.manipulability(q);
        }, py::arg("q"),
           "Compute manipulability measure (singularity detection)")

        // Tool management
        .def("add_tool", &Robot::add_tool,
             py::arg("tool"),
             "Add tool to robot")
        .def("set_active_tool",
             py::overload_cast<const std::string&>(&Robot::set_active_tool),
             py::arg("name"),
             "Set active tool by name")
        .def("active_tool", &Robot::active_tool, "Get active tool")
        .def("has_active_tool", &Robot::has_active_tool, "Check if active tool is set")

        .def("__repr__", [](const Robot& self) {
            return "<Robot '" + self.name() + "' dof=" + std::to_string(self.dof()) + ">";
        });
}
