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

# Classical approach: homogeneous transformation
T = rs.Transform(translation=[1, 2, 3])
print(T.matrix())

# Modern approach: Lie algebra
omega = np.array([0, 0, 1.57])  # 90° rotation around Z
v = np.array([1, 0, 0])
xi = rs.se3(omega, v)
g = rs.exp_se3(xi)  # Exponential map
print(g.matrix())

# Robot kinematics
robot = rs.Robot.from_urdf("models/ur5.urdf")
q = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
ee_pose = robot.compute_fk(q)
```

## Development Status

**Phase 1 (In Progress)**: Foundation - Math library, robot model, basic kinematics

See [DESIGN.md](DESIGN.md) for complete architecture and roadmap.

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
