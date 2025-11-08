# Robospace Phase 1: Foundation - Detailed Task Breakdown

**Timeline:** Months 1-2
**Goal:** Establish core infrastructure with math library and basic Item system

---

## Task Categories

### 1. Project Setup & Build System

#### 1.1 Directory Structure
- [ ] Create complete directory structure (core/, python/, web/, tests/, docs/, etc.)
- [ ] Add .gitignore for C++, Python, build artifacts
- [ ] Create placeholder README files in each module

#### 1.2 CMake Build System
- [ ] Root CMakeLists.txt with project configuration
- [ ] core/CMakeLists.txt for C++ libraries
- [ ] Find/configure Eigen dependency
- [ ] Set up compiler flags (C++17/20, optimization levels)
- [ ] Configure installation rules
- [ ] Add CMake options (BUILD_TESTS, BUILD_PYTHON, etc.)

#### 1.3 Dependency Management
- [ ] Choose dependency manager (Conan or vcpkg)
- [ ] Create conanfile.txt or vcpkg.json
- [ ] Document dependency installation process
- [ ] Set up Eigen 3.4+
- [ ] (Optional) Add Sophus for Lie algebra validation

---

### 2. Math Library (C++) - Core Module

#### 2.1 Classical Representations
- [ ] **Transform class** (SE(3) as 4×4 homogeneous matrix)
  - Constructors (identity, from matrix, from R and p)
  - Accessors (rotation, translation, matrix)
  - Operators (multiplication, inverse)
  - Static factory methods
- [ ] **Rotation class** (SO(3) representations)
  - Rotation matrix representation
  - Quaternion representation
  - Euler angles (XYZ, ZYX, etc.)
  - Axis-angle representation
  - Conversions between representations
- [ ] **Vector3 class** (thin wrapper around Eigen::Vector3d)
- [ ] **Quaternion class** (unit quaternion operations)

#### 2.2 Modern Robotics / Lie Algebra
- [ ] **SE3 class** (SE(3) Lie group element)
  - Constructors (identity, from R and p, from matrix)
  - Group operations (composition, inverse)
  - Accessors
- [ ] **SO3 class** (SO(3) Lie group element)
  - Constructors (identity, from matrix, from axis-angle)
  - Group operations
- [ ] **se3 class** (se(3) Lie algebra - twist)
  - Constructors (zero, from omega and v, from 6D vector)
  - Bracket operator [xi] (4×4 matrix)
  - Accessors (omega, v, as vector)
- [ ] **so3 class** (so(3) Lie algebra - angular velocity)
  - Constructors
  - Bracket operator [omega] (3×3 skew-symmetric)
- [ ] **Exponential maps**
  - exp_se3: se(3) → SE(3) (Rodrigues' formula)
  - exp_so3: so(3) → SO(3)
- [ ] **Logarithm maps**
  - log_SE3: SE(3) → se(3)
  - log_SO3: SO(3) → so(3)
- [ ] **Adjoint transformations**
  - adjoint_SE3: 6×6 adjoint matrix
  - adjoint_se3: 6×6 adjoint for Lie algebra

#### 2.3 Utility Functions
- [ ] Angle normalization (wrap to [-π, π])
- [ ] Skew-symmetric matrix from vector
- [ ] Vector from skew-symmetric matrix
- [ ] Conversion utilities (Transform ↔ SE3, etc.)

#### 2.4 Math Library Tests (C++)
- [ ] Transform class unit tests
- [ ] Rotation class unit tests (all representations)
- [ ] SE3/se3 unit tests
- [ ] SO3/so3 unit tests
- [ ] Exponential map tests (verify exp(log(g)) = g)
- [ ] Logarithm map tests
- [ ] Adjoint tests
- [ ] Conversion tests
- [ ] Benchmark tests (performance targets)

---

### 3. Item Pattern - Base Classes (C++)

#### 3.1 Item Base Class
- [ ] **Item class** (base for all workspace items)
  - ID management (UUID generation)
  - ItemType enum (ROBOT, TOOL, FRAME, etc.)
  - Name property
  - Pose property (Transform)
  - Parent/child tree structure
  - Visibility property
  - to_dict() serialization method

#### 3.2 Core Item Types (Phase 1 subset)
- [ ] **Frame class** (reference frame, inherits Item)
  - Just uses base Item functionality
  - No additional methods in Phase 1
- [ ] **Target class** (target pose, inherits Item)
  - Target pose storage
  - Optional joint configuration

#### 3.3 Item Tests (C++)
- [ ] Item construction and properties
- [ ] Parent-child relationships
- [ ] Tree traversal
- [ ] Serialization/deserialization

---

### 4. Robot Model (Basic - C++)

#### 4.1 Joint and Link Classes
- [ ] **Joint class**
  - Joint types (revolute, prismatic, fixed)
  - Joint limits (position, velocity, acceleration)
  - Axis of motion
  - Parent/child link references
- [ ] **Link class**
  - Name and ID
  - Visual geometry (placeholder for Phase 1)
  - Collision geometry (placeholder)
  - Inertial properties (mass, inertia tensor, COM)

#### 4.2 Kinematic Chain
- [ ] **KinematicChain class**
  - Store joints and links
  - Tree structure representation
  - DOF calculation
  - Joint limits enforcement

#### 4.3 URDF Parser (Basic)
- [ ] **URDFParser class**
  - Parse robot tag
  - Parse link tags (basic)
  - Parse joint tags (basic)
  - Build KinematicChain
  - Handle URDF file loading
  - Error handling

#### 4.4 Robot Class (Simple)
- [ ] **Robot class** (inherits Item)
  - Load from URDF
  - Store KinematicChain
  - Store current joint state
  - DOF accessor
  - Joint limits accessors

#### 4.5 Robot Model Tests
- [ ] Joint class tests
- [ ] Link class tests
- [ ] URDF parsing tests (simple robot)
- [ ] Robot class basic tests

---

### 5. Forward Kinematics (Basic - C++)

#### 5.1 DH-based Forward Kinematics
- [ ] **ForwardKinematics interface** (base class)
- [ ] **DHForwardKinematics class**
  - Compute FK using DH parameters
  - Return end-effector Transform
  - Support for arbitrary link chains

#### 5.2 FK Tests
- [ ] Test with simple 2-DOF robot
- [ ] Test with 6-DOF industrial robot (UR5 model)
- [ ] Verify against known poses
- [ ] Performance benchmarks (target: < 10 μs)

---

### 6. Python Bindings (pybind11)

#### 6.1 Build System
- [ ] Add pybind11 as dependency
- [ ] Create python/CMakeLists.txt
- [ ] Set up scikit-build-core for Python packaging
- [ ] Create setup.py or pyproject.toml
- [ ] Configure module installation

#### 6.2 Math Bindings
- [ ] Bind Transform class
- [ ] Bind Rotation class
- [ ] Bind SE3 class
- [ ] Bind se3 class
- [ ] Bind SO3 class
- [ ] Bind so3 class
- [ ] Bind exp_se3, log_SE3, exp_so3, log_SO3
- [ ] Bind adjoint functions
- [ ] Enable Eigen ↔ NumPy conversions (pybind11/eigen.h)
- [ ] Add operator overloading (*, +, etc.)
- [ ] Add __repr__ for nice printing

#### 6.3 Item Bindings
- [ ] Bind ItemType enum
- [ ] Bind Item base class
- [ ] Bind Frame class
- [ ] Bind Target class
- [ ] Bind Robot class (basic)

#### 6.4 Kinematics Bindings
- [ ] Bind ForwardKinematics class
- [ ] Bind FK compute method

#### 6.5 Python Module Structure
- [ ] Create robospace/__init__.py
- [ ] Import and expose classes
- [ ] Add type hints (stub files .pyi)
- [ ] Organize submodules (robospace.math, robospace.kinematics)

---

### 7. Python API Layer (Optional Wrappers)

#### 7.1 Math Module
- [ ] Create robospace/math.py
- [ ] Add convenient factory functions
- [ ] Add NumPy integration helpers

#### 7.2 Item Module
- [ ] Create robospace/items.py (if needed)
- [ ] Add convenience wrappers

---

### 8. Testing Infrastructure

#### 8.1 C++ Testing
- [ ] Set up Google Test
- [ ] Create tests/CMakeLists.txt
- [ ] Add test discovery
- [ ] Create test data directory (test robots, URDFs)
- [ ] Add benchmark framework (Google Benchmark)

#### 8.2 Python Testing
- [ ] Set up pytest
- [ ] Create python/tests/ directory
- [ ] Add test fixtures
- [ ] Add conftest.py for shared fixtures

#### 8.3 Test Data
- [ ] Create test URDF files (simple 2R, 6-DOF, UR5)
- [ ] Create expected FK results (golden data)
- [ ] Add test meshes (if needed)

---

### 9. Documentation

#### 9.1 C++ Documentation
- [ ] Set up Doxygen
- [ ] Create Doxyfile
- [ ] Add documentation comments to all public APIs
- [ ] Generate HTML docs

#### 9.2 Python Documentation
- [ ] Set up Sphinx
- [ ] Create docs/source/ structure
- [ ] Add autodoc for API reference
- [ ] Write getting started guide
- [ ] Add examples

#### 9.3 User Documentation
- [ ] README.md with installation instructions
- [ ] Build instructions (CMake)
- [ ] Python installation guide (pip install)
- [ ] Basic usage examples

---

### 10. CI/CD Pipeline

#### 10.1 GitHub Actions
- [ ] Create .github/workflows/ci.yml
- [ ] C++ build and test job (Ubuntu)
- [ ] Python build and test job
- [ ] Code coverage job (lcov for C++, pytest-cov for Python)
- [ ] Documentation build job

#### 10.2 Code Quality
- [ ] Set up clang-format (C++)
- [ ] Add .clang-format config
- [ ] Set up ruff (Python linter)
- [ ] Add pre-commit hooks (optional)

---

## Deliverables Checklist

- [ ] C++ library: `librobospace-core.so` (Linux) / `librobospace-core.dylib` (macOS)
- [ ] Python package: `pip install robospace` works
- [ ] Can load URDF and create Robot object
- [ ] Can compute forward kinematics (DH-based)
- [ ] Can create SE3/se3 objects and use exp/log maps
- [ ] All unit tests pass (C++ and Python)
- [ ] Code coverage > 80%
- [ ] Documentation builds successfully
- [ ] CI/CD pipeline runs on every commit

---

## Example Milestones

### Milestone 1 (Week 1-2): Math Library
**Goal:** Complete math library with tests

**Definition of Done:**
- All Transform, Rotation, SE3, se3 classes implemented
- Exponential/logarithm maps working
- 100+ unit tests passing
- Can create and manipulate transformations in both C++ and Python

**Example Code Works:**
```python
import robospace as rs
import numpy as np

# Classical approach
T = rs.Transform(translation=[1, 2, 3])
print(T.matrix())

# Modern Robotics approach
omega = np.array([0, 0, 1.57])  # 90° around Z
v = np.array([1, 0, 0])
xi = rs.se3(omega, v)
g = rs.exp_se3(xi)
print(g.matrix())
```

### Milestone 2 (Week 3-4): Robot Model + FK
**Goal:** Load robot, compute FK

**Definition of Done:**
- Can parse URDF files
- Robot class stores kinematic chain
- Forward kinematics computes correct poses
- Python bindings work

**Example Code Works:**
```python
import robospace as rs

robot = rs.Robot.from_urdf("models/ur5.urdf")
print(f"Robot DOF: {robot.dof}")

q = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
robot.joints = q

ee_pose = robot.compute_fk(q)
print(f"End-effector position: {ee_pose.translation}")
```

### Milestone 3 (Week 5-6): Item Pattern + Integration
**Goal:** Unified Item system works

**Definition of Done:**
- Item base class with tree structure
- Robot inherits from Item
- Can create scenes with multiple items
- Serialization works

**Example Code Works:**
```python
import robospace as rs

# Create items
frame = rs.Frame("world")
robot = rs.Robot.from_urdf("ur5.urdf")
target = rs.Target("pick_point")

# Build hierarchy
robot.set_parent(frame)
target.set_parent(frame)
target.pose = rs.Transform(translation=[0.5, 0.2, 0.3])

# All are Items!
def print_item(item: rs.Item):
    print(f"{item.name}: {item.type}")

print_item(robot)   # ✓
print_item(target)  # ✓
```

### Milestone 4 (Week 7-8): Testing & Documentation
**Goal:** Production-ready Phase 1

**Definition of Done:**
- CI/CD pipeline passes
- Documentation complete
- All deliverables ready
- Performance targets met (FK < 10μs)

---

## Priority Order (Recommended)

1. **Week 1-2:** Math library (Transform, SE3, exp/log) + tests
2. **Week 2-3:** Python bindings for math + tests
3. **Week 3-4:** Robot model (URDF, joints, links) + FK
4. **Week 4-5:** Python bindings for robot + FK
5. **Week 5-6:** Item pattern (base Item, Robot as Item)
6. **Week 6-7:** Python bindings for Item pattern
7. **Week 7-8:** Documentation, CI/CD, polish

---

## Notes

- **Parallel Work:** Math library and build system can be done in parallel
- **Dependencies:** Robot model depends on math library being done
- **Python Bindings:** Can be done incrementally after each C++ component
- **Testing:** Write tests alongside implementation (TDD recommended)
- **Documentation:** Write docs as you code (easier than retroactive)

---

## Phase 1 Success Criteria

✅ Math library complete with dual representation (classical + Lie algebra)
✅ Can load robots from URDF
✅ Forward kinematics works (DH-based)
✅ Item pattern base classes implemented
✅ Python bindings functional
✅ All tests passing (80%+ coverage)
✅ Documentation complete
✅ CI/CD pipeline working

**Ready for Phase 2:** Inverse kinematics and planning
