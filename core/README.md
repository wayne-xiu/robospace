# Robospace Core (C++)

This directory contains the core C++ implementation of the robospace robotics framework.

## Modules

- **math/** - Mathematical foundations (Transform, SE(3), Lie algebra)
- **model/** - Robot model representation (URDF, joints, links)
- **kinematics/** - Forward and inverse kinematics

## Building

The core library is header-only for the math module and compiled for model/kinematics modules.

See the root CMakeLists.txt for build instructions.
