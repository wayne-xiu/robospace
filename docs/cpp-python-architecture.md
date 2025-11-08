# Robospace: C++ Core + Python Bindings Architecture

**Date:** 2025-11-08
**Purpose:** Clarify the C++ core implementation with Python bindings architecture

---

## Architecture Overview

**Core Principle:** Performance-critical code in C++, accessible via Python bindings

```
┌─────────────────────────────────────────────────────────────┐
│                    User Applications                        │
│                                                             │
│  Python Scripts  │  C++ Programs  │  Web App (Python/JS)   │
└──────────┬─────────────────┬─────────────────┬────────────┘
           │                 │                 │
           │                 │                 │
┌──────────▼─────────────────┼─────────────────▼────────────┐
│          Python API Layer                                  │
│          (Python wrappers, type hints, NumPy integration)  │
│                            │                                │
│  import robospace as rs    │                                │
│  robot = rs.Robot(...)     │                                │
└────────────────────────────┼────────────────────────────────┘
                             │
                    ┌────────▼────────┐
                    │   pybind11      │  (C++ ↔ Python)
                    └────────┬────────┘
                             │
┌────────────────────────────▼────────────────────────────────┐
│              C++ Core Implementation                        │
│              (All performance-critical algorithms)          │
│                                                             │
│  namespace robospace {                                     │
│      class Robot { ... };                                  │
│      class ForwardKinematics { ... };                      │
│      SE3 exp_se3(const se3& xi);                           │
│  }                                                         │
└─────────────────────────────────────────────────────────────┘
```

---

## Example 1: Math Library (SE3/Lie Algebra)

### C++ Implementation (Core)

**File:** `core/math/se3.h` and `core/math/se3.cpp`

```cpp
// core/math/se3.h
#ifndef ROBOSPACE_MATH_SE3_H
#define ROBOSPACE_MATH_SE3_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace robospace {
namespace math {

// SE(3) Lie group element (special Euclidean group)
class SE3 {
public:
    // Constructors
    SE3();  // Identity
    SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);
    SE3(const Eigen::Matrix4d& T);

    // Accessors
    Eigen::Matrix3d rotation() const { return R_; }
    Eigen::Vector3d translation() const { return p_; }
    Eigen::Matrix4d matrix() const;

    // Group operations
    SE3 operator*(const SE3& other) const;
    SE3 inverse() const;
    Eigen::Vector3d operator*(const Eigen::Vector3d& v) const;

    // Static constructors
    static SE3 identity();
    static SE3 from_translation(const Eigen::Vector3d& p);
    static SE3 from_rotation(const Eigen::Matrix3d& R);

private:
    Eigen::Matrix3d R_;  // Rotation matrix (SO(3))
    Eigen::Vector3d p_;  // Translation vector
};

// se(3) Lie algebra element (twist)
class se3 {
public:
    // Constructors
    se3();
    se3(const Eigen::Vector3d& omega, const Eigen::Vector3d& v);
    se3(const Eigen::Matrix<double, 6, 1>& xi);  // [omega; v]

    // Accessors
    Eigen::Vector3d omega() const { return omega_; }  // Angular velocity
    Eigen::Vector3d v() const { return v_; }          // Linear velocity
    Eigen::Matrix<double, 6, 1> vector() const;       // As 6D vector
    Eigen::Matrix4d bracket() const;                  // [xi] 4×4 matrix

    // Lie algebra operations
    se3 operator+(const se3& other) const;
    se3 operator*(double scalar) const;

private:
    Eigen::Vector3d omega_;  // Angular velocity
    Eigen::Vector3d v_;      // Linear velocity
};

// Exponential map: se(3) → SE(3)
SE3 exp_se3(const se3& xi);
SE3 exp_se3(const Eigen::Matrix<double, 6, 1>& xi);

// Logarithm map: SE(3) → se(3)
se3 log_SE3(const SE3& g);

// Adjoint transformations
Eigen::Matrix<double, 6, 6> adjoint_SE3(const SE3& g);
Eigen::Matrix<double, 6, 6> adjoint_se3(const se3& xi);

}  // namespace math
}  // namespace robospace

#endif  // ROBOSPACE_MATH_SE3_H
```

**Implementation:**

```cpp
// core/math/se3.cpp
#include "se3.h"
#include <unsupported/Eigen/MatrixFunctions>

namespace robospace {
namespace math {

// SE3 implementation
SE3::SE3() : R_(Eigen::Matrix3d::Identity()), p_(Eigen::Vector3d::Zero()) {}

SE3::SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& p)
    : R_(R), p_(p) {}

Eigen::Matrix4d SE3::matrix() const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R_;
    T.block<3,1>(0,3) = p_;
    return T;
}

SE3 SE3::operator*(const SE3& other) const {
    return SE3(R_ * other.R_, R_ * other.p_ + p_);
}

SE3 SE3::inverse() const {
    Eigen::Matrix3d R_inv = R_.transpose();
    return SE3(R_inv, -R_inv * p_);
}

// Exponential map implementation (Rodrigues' formula for SE(3))
SE3 exp_se3(const se3& xi) {
    Eigen::Vector3d omega = xi.omega();
    Eigen::Vector3d v = xi.v();

    double theta = omega.norm();

    if (theta < 1e-10) {
        // Small angle approximation
        return SE3(Eigen::Matrix3d::Identity(), v);
    }

    Eigen::Vector3d omega_hat = omega / theta;
    Eigen::Matrix3d omega_cross;
    omega_cross << 0, -omega_hat(2), omega_hat(1),
                   omega_hat(2), 0, -omega_hat(0),
                   -omega_hat(1), omega_hat(0), 0;

    // Rodrigues' formula for SO(3)
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
                       + std::sin(theta) * omega_cross
                       + (1 - std::cos(theta)) * omega_cross * omega_cross;

    // Translation part
    Eigen::Matrix3d V = Eigen::Matrix3d::Identity()
                      + ((1 - std::cos(theta)) / theta) * omega_cross
                      + ((theta - std::sin(theta)) / theta) * omega_cross * omega_cross;

    Eigen::Vector3d p = V * v;

    return SE3(R, p);
}

// Logarithm map implementation
se3 log_SE3(const SE3& g) {
    Eigen::Matrix3d R = g.rotation();
    Eigen::Vector3d p = g.translation();

    // Extract rotation angle
    double trace = R.trace();
    double theta = std::acos((trace - 1.0) / 2.0);

    if (theta < 1e-10) {
        // Small angle
        return se3(Eigen::Vector3d::Zero(), p);
    }

    // Extract omega (logarithm of rotation)
    Eigen::Matrix3d omega_cross = (R - R.transpose()) / (2.0 * std::sin(theta));
    Eigen::Vector3d omega;
    omega << omega_cross(2,1), omega_cross(0,2), omega_cross(1,0);
    omega *= theta;

    // Compute V inverse
    Eigen::Matrix3d omega_cross_norm = omega_cross / theta;
    Eigen::Matrix3d V_inv = Eigen::Matrix3d::Identity()
                          - 0.5 * omega_cross_norm * theta
                          + ((1.0 / theta - 0.5 / std::tan(theta/2)) / theta)
                            * omega_cross_norm * omega_cross_norm * theta * theta;

    Eigen::Vector3d v = V_inv * p;

    return se3(omega, v);
}

}  // namespace math
}  // namespace robospace
```

### Python Bindings (pybind11)

**File:** `python/bindings/math_bindings.cpp`

```cpp
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include "core/math/se3.h"

namespace py = pybind11;
using namespace robospace::math;

void bind_math(py::module& m) {
    // SE3 class
    py::class_<SE3>(m, "SE3", "SE(3) Lie group element")
        .def(py::init<>(), "Construct identity SE(3)")
        .def(py::init<const Eigen::Matrix3d&, const Eigen::Vector3d&>(),
             "Construct from rotation matrix and translation vector",
             py::arg("R"), py::arg("p"))
        .def(py::init<const Eigen::Matrix4d&>(),
             "Construct from 4x4 homogeneous matrix",
             py::arg("T"))
        .def_property_readonly("rotation", &SE3::rotation,
                              "Get rotation matrix")
        .def_property_readonly("translation", &SE3::translation,
                              "Get translation vector")
        .def("matrix", &SE3::matrix,
             "Get 4x4 homogeneous matrix representation")
        .def("inverse", &SE3::inverse,
             "Compute inverse transformation")
        .def(py::self * py::self)  // Operator overloading
        .def("__mul__", [](const SE3& self, const Eigen::Vector3d& v) {
            return self * v;
        })
        .def("__repr__", [](const SE3& g) {
            std::ostringstream oss;
            oss << "SE3(R=\n" << g.rotation() << ",\np=" << g.translation().transpose() << ")";
            return oss.str();
        })
        .def_static("identity", &SE3::identity, "Identity transformation");

    // se3 class
    py::class_<se3>(m, "se3", "se(3) Lie algebra element (twist)")
        .def(py::init<>(), "Construct zero twist")
        .def(py::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
             "Construct from angular and linear velocity",
             py::arg("omega"), py::arg("v"))
        .def(py::init<const Eigen::Matrix<double, 6, 1>&>(),
             "Construct from 6D vector [omega; v]",
             py::arg("xi"))
        .def_property_readonly("omega", &se3::omega,
                              "Get angular velocity")
        .def_property_readonly("v", &se3::v,
                              "Get linear velocity")
        .def("vector", &se3::vector,
             "Get 6D vector representation")
        .def("bracket", &se3::bracket,
             "Get 4x4 bracket (hat) matrix [xi]")
        .def(py::self + py::self)
        .def(py::self * double())
        .def("__repr__", [](const se3& xi) {
            std::ostringstream oss;
            oss << "se3(omega=" << xi.omega().transpose()
                << ", v=" << xi.v().transpose() << ")";
            return oss.str();
        });

    // Exponential and logarithm maps
    m.def("exp_se3",
          py::overload_cast<const se3&>(&exp_se3),
          "Exponential map: se(3) -> SE(3)",
          py::arg("xi"));

    m.def("log_SE3",
          &log_SE3,
          "Logarithm map: SE(3) -> se(3)",
          py::arg("g"));

    m.def("adjoint_SE3",
          &adjoint_SE3,
          "Adjoint transformation for SE(3)",
          py::arg("g"));
}
```

### Python API Usage

**User Code:**

```python
import robospace as rs
import numpy as np

# Create SE(3) transformation
R = np.eye(3)  # Rotation matrix
p = np.array([1.0, 2.0, 3.0])  # Translation
g = rs.SE3(R, p)

print(f"SE(3) transformation:\n{g}")
print(f"As matrix:\n{g.matrix()}")

# Create twist (se(3) element)
omega = np.array([0, 0, 1.0])  # Rotate around Z
v = np.array([1.0, 0, 0])      # Move in X
xi = rs.se3(omega, v)

print(f"Twist: {xi}")

# Exponential map (integrate twist over unit time)
g_exp = rs.exp_se3(xi)
print(f"Exponential of twist:\n{g_exp.matrix()}")

# Logarithm map (extract twist from transformation)
xi_log = rs.log_SE3(g_exp)
print(f"Logarithm of transformation: {xi_log}")

# Compose transformations
g1 = rs.SE3(R, np.array([1, 0, 0]))
g2 = rs.SE3(R, np.array([0, 1, 0]))
g_composed = g1 * g2
print(f"Composition: {g_composed.translation}")  # [1, 1, 0]

# Transform a point
point = np.array([1, 2, 3])
transformed = g1 * point
print(f"Transformed point: {transformed}")
```

---

## Example 2: Robot Item Class (Hybrid Pattern)

### C++ Core Implementation

**File:** `core/item/robot.h` and `core/item/robot.cpp`

```cpp
// core/item/robot.h
#ifndef ROBOSPACE_ITEM_ROBOT_H
#define ROBOSPACE_ITEM_ROBOT_H

#include "core/item/item.h"
#include "core/kinematics/forward_kinematics.h"
#include "core/kinematics/inverse_kinematics.h"
#include <Eigen/Dense>
#include <memory>

namespace robospace {

enum class ItemType {
    STATION = 1,
    ROBOT = 2,
    FRAME = 3,
    TOOL = 4,
    OBJECT = 5,
    TARGET = 6,
    MACHINE_TOOL = 10,
    LASER_TRACKER = 11
};

// Base Item class (like RoboDK)
class Item {
public:
    Item(const std::string& id, ItemType type);
    virtual ~Item() = default;

    // Common interface for ALL items
    std::string id() const { return id_; }
    ItemType type() const { return type_; }
    std::string name() const { return name_; }
    void set_name(const std::string& name) { name_ = name; }

    math::Transform pose() const { return pose_; }
    void set_pose(const math::Transform& pose) { pose_ = pose; }

    Item* parent() const { return parent_; }
    void set_parent(Item* parent);
    std::vector<Item*> children() const { return children_; }

    bool visible() const { return visible_; }
    void set_visible(bool visible) { visible_ = visible; }

protected:
    std::string id_;
    ItemType type_;
    std::string name_;
    math::Transform pose_;
    Item* parent_;
    std::vector<Item*> children_;
    bool visible_;
};

// Robot-specific class (inherits from Item)
class Robot : public Item {
public:
    Robot(const std::string& id, const std::string& urdf_path);

    // Robot-specific methods
    Eigen::VectorXd joints() const { return joints_; }
    void set_joints(const Eigen::VectorXd& q);

    math::Transform compute_fk(const Eigen::VectorXd& q);
    std::optional<Eigen::VectorXd> compute_ik(
        const math::Transform& target,
        const Eigen::VectorXd& seed);

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q);

    int dof() const { return dof_; }

private:
    std::unique_ptr<kinematics::ForwardKinematics> fk_;
    std::unique_ptr<kinematics::InverseKinematics> ik_;
    Eigen::VectorXd joints_;
    int dof_;
};

}  // namespace robospace

#endif  // ROBOSPACE_ITEM_ROBOT_H
```

**Implementation:**

```cpp
// core/item/robot.cpp
#include "robot.h"
#include "core/model/urdf_parser.h"

namespace robospace {

Robot::Robot(const std::string& id, const std::string& urdf_path)
    : Item(id, ItemType::ROBOT) {

    // Parse URDF
    model::URDFParser parser;
    auto robot_model = parser.parse(urdf_path);

    dof_ = robot_model->dof();
    joints_ = Eigen::VectorXd::Zero(dof_);

    // Create FK and IK solvers
    fk_ = std::make_unique<kinematics::ForwardKinematics>(robot_model);
    ik_ = std::make_unique<kinematics::NumericalIK>(robot_model);
}

void Robot::set_joints(const Eigen::VectorXd& q) {
    joints_ = q;
    // Update end-effector pose
    pose_ = compute_fk(q);
}

math::Transform Robot::compute_fk(const Eigen::VectorXd& q) {
    return fk_->compute(q);
}

std::optional<Eigen::VectorXd> Robot::compute_ik(
    const math::Transform& target,
    const Eigen::VectorXd& seed) {
    return ik_->solve(target, seed);
}

Eigen::MatrixXd Robot::jacobian(const Eigen::VectorXd& q) {
    kinematics::JacobianCalculator jac_calc(fk_->model());
    return jac_calc.compute(q);
}

}  // namespace robospace
```

### Python Bindings

**File:** `python/bindings/item_bindings.cpp`

```cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include "core/item/robot.h"

namespace py = pybind11;
using namespace robospace;

void bind_items(py::module& m) {
    // ItemType enum
    py::enum_<ItemType>(m, "ItemType")
        .value("STATION", ItemType::STATION)
        .value("ROBOT", ItemType::ROBOT)
        .value("FRAME", ItemType::FRAME)
        .value("TOOL", ItemType::TOOL)
        .value("OBJECT", ItemType::OBJECT)
        .value("TARGET", ItemType::TARGET)
        .value("MACHINE_TOOL", ItemType::MACHINE_TOOL)
        .value("LASER_TRACKER", ItemType::LASER_TRACKER);

    // Base Item class
    py::class_<Item>(m, "Item", "Base class for all workspace items")
        .def_property_readonly("id", &Item::id)
        .def_property_readonly("type", &Item::type)
        .def_property("name", &Item::name, &Item::set_name)
        .def_property("pose", &Item::pose, &Item::set_pose)
        .def_property("visible", &Item::visible, &Item::set_visible)
        .def("parent", &Item::parent, py::return_value_policy::reference)
        .def("set_parent", &Item::set_parent)
        .def("children", &Item::children, py::return_value_policy::reference);

    // Robot class (inherits from Item)
    py::class_<Robot, Item>(m, "Robot", "Robot item with kinematics")
        .def(py::init<const std::string&, const std::string&>(),
             "Create robot from URDF file",
             py::arg("id"), py::arg("urdf_path"))
        .def_property("joints", &Robot::joints, &Robot::set_joints,
                     "Get/set joint positions")
        .def("compute_fk", &Robot::compute_fk,
             "Compute forward kinematics",
             py::arg("q"))
        .def("compute_ik", &Robot::compute_ik,
             "Compute inverse kinematics",
             py::arg("target"), py::arg("seed"))
        .def("jacobian", &Robot::jacobian,
             "Compute Jacobian matrix",
             py::arg("q"))
        .def_property_readonly("dof", &Robot::dof,
                              "Number of degrees of freedom");
}
```

### Python Usage

```python
import robospace as rs
import numpy as np

# Create robot (C++ object wrapped in Python)
robot = rs.Robot("ur10e_1", "models/ur10e.urdf")

print(f"Robot: {robot.name}")
print(f"Type: {robot.type}")
print(f"DOF: {robot.dof}")

# Set joint positions (calls C++ implementation)
q = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
robot.joints = q

# Compute forward kinematics (C++ algorithm)
ee_pose = robot.compute_fk(q)
print(f"End-effector pose:\n{ee_pose.matrix()}")

# Compute inverse kinematics (C++ solver)
target = rs.Transform(translation=[0.5, 0.2, 0.3])
ik_solution = robot.compute_ik(target, seed=q)
if ik_solution is not None:
    print(f"IK solution: {ik_solution}")

# Compute Jacobian (C++ linear algebra)
J = robot.jacobian(q)
print(f"Jacobian shape: {J.shape}")

# Robot is also an Item (polymorphism works!)
def print_item_info(item: rs.Item):
    print(f"Item name: {item.name}, type: {item.type}")

print_item_info(robot)  # Works! Robot is an Item
```

---

## Summary

| Aspect | C++ Core | Python Bindings | Python API Layer |
|--------|----------|-----------------|------------------|
| **Purpose** | Performance-critical algorithms | Expose C++ to Python | User-friendly interface |
| **Language** | C++17/20 | pybind11 (C++) | Python |
| **Examples** | SE3, exp_se3, Robot::compute_fk | py::class_<SE3>, py::def("exp_se3") | rs.SE3(), rs.exp_se3() |
| **Dependencies** | Eigen, FCL, OMPL | pybind11, pybind11_eigen | NumPy, type hints |
| **Performance** | Maximum (native C++) | Near-native (minimal overhead) | Same as C++ (calls C++) |
| **Who Uses** | C++ developers, internal | Binding developers | Python users, web backend |

**Key Points:**

1. ✅ **All algorithms in C++** (kinematics, dynamics, planning, Lie algebra, etc.)
2. ✅ **pybind11 exposes C++ to Python** (automatic conversion between Eigen and NumPy)
3. ✅ **Python API** provides Pythonic interface (properties, type hints, operator overloading)
4. ✅ **Same hybrid Item pattern** in both languages (base Item + specialized classes)
5. ✅ **Zero performance loss** - Python calls go directly to C++ implementation

**You get:**
- **Speed of C++** for computation
- **Ease of Python** for scripting and web backend
- **Type safety** in both languages
- **Best of both worlds!** 🚀
