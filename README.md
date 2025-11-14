# Robospace

Modern robotics framework with dual mathematical representations (classical + Lie algebra), machine tool support, and metrology integration.

## Features

- **Dual Mathematical Framework**: Classical (DH, homogeneous matrices) + Modern (Lie algebra, Product of Exponentials)
- **C++ Core + Python API**: High-performance C++ with convenient Python bindings via pybind11
- **Industrial Robot Support**: Forward/inverse kinematics, dynamics, motion planning
- **Machine Tool Integration**: CNC mills, lathes with G-code parsing
- **Metrology Support**: Laser trackers, scanners for robot calibration and inspection
- **Item Pattern**: Unified interface for all workspace objects (robots, tools, frames, targets)

## Project Structure

```
robospace/
├── core/           # C++ core library
│   ├── math/       # Transform, SE(3), Lie algebra
│   ├── model/      # Robot model, URDF parsing
│   └── kinematics/ # Forward/inverse kinematics
├── python/         # Python bindings
│   ├── bindings/   # pybind11 bindings
│   └── robospace/  # Python API
├── tests/          # Unit tests
│   ├── cpp/        # C++ tests (Google Test)
│   └── python/     # Python tests (pytest)
└── docs/           # Documentation

```

## Installation

### Prerequisites

- CMake >= 3.18
- C++17 compiler (GCC, Clang, MSVC)
- Eigen >= 3.4
- Python >= 3.8 (for Python bindings)

### C++ Library

```bash
mkdir build && cd build
cmake ..
make
sudo make install
```

### Python Package

```bash
pip install -e .
```

## Quick Start

### Python

```python
import robospace as rs
import numpy as np

# Math types - SE3 transformations
T = rs.SE3.Identity()
pos = T.translation()  # NumPy array [0, 0, 0]

# Classical transformations
T_classic = rs.Transform.Translation(1.0, 2.0, 3.0)
mat = T_classic.matrix()  # 4×4 NumPy array

# Load robot from URDF
robot = rs.Robot.from_urdf("models/ur5.urdf")
print(f"Robot DOF: {robot.dof()}")

# Forward kinematics
q = np.zeros(6)
T_ee = robot.fk(q, "ee_link")  # Returns SE3 object
position = T_ee.translation()   # Extract [x, y, z]
rotation = T_ee.rotation()      # Extract 3×3 rotation matrix

# Jacobian computation
J_base = robot.jacob0(q)        # 6×6 Jacobian in base frame
J_ee = robot.jacobe(q)          # 6×6 Jacobian in EE frame
m = robot.manipulability(q)     # Singularity measure
```

## Development Status

**Phase 1 ✅ COMPLETE**: Foundation - Math library, robot model, kinematics, Python bindings
- All 304 tests passing (298 C++ + 6 Python)
- Production-ready for FK, Jacobians, URDF loading
- Full Python/NumPy integration

**Phase 2 (Next)**: Dynamics, inverse kinematics, trajectory planning

See [PROGRESS.md](docs/PROGRESS.md) for detailed status and [DESIGN.md](DESIGN.md) for architecture.

## Documentation

- [Architecture Design](DESIGN.md)
- [Phase 1 Tasks](docs/phase1-detailed-tasks.md)
- [C++/Python Implementation Guide](docs/cpp-python-architecture.md)
- [Git Workflow](docs/git-workflow-best-practices.md)

## References

- *Modern Robotics* by Kevin Lynch and Frank Park
- *Robotics, Vision & Control* by Peter Corke
- RoboDK Item pattern
- Spatial Math Toolbox (MATLAB/Python)

## License

MIT

## Author

Wayne Xiu
