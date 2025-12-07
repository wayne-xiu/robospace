"""
RoboSpace - Robot kinematics and dynamics library

Simple, elegant Python API for robot modeling and control.
"""

from ._robospace_core import (
    # Math types - Lie groups
    SE3,
    SO3,

    # Math types - Lie algebras
    se3,
    so3,

    # Euler convention enum
    EulerConvention,

    # Factory functions
    make_se3,
    make_so3,

    # Exponential and logarithm maps
    exp_se3,
    log_SE3,
    exp_so3,
    log_SO3,

    # Model types
    Robot,
    Link,
    Joint,
    Tool,
    Frame,

    # Joint types
    JointType,

    # Kinematics
    IKSolver,
    IKResult,
    IKMode,
)

__version__ = "0.1.0"

__all__ = [
    # Math - Lie groups
    "SE3",
    "SO3",

    # Math - Lie algebras
    "se3",
    "so3",

    # Euler convention enum
    "EulerConvention",

    # Factory functions
    "make_se3",
    "make_so3",

    # Exponential/logarithm maps
    "exp_se3",
    "log_SE3",
    "exp_so3",
    "log_SO3",

    # Model
    "Robot",
    "Link",
    "Joint",
    "Tool",
    "Frame",
    "JointType",

    # Kinematics
    "IKSolver",
    "IKResult",
    "IKMode",
]
