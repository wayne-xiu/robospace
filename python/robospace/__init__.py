"""
RoboSpace - Robot kinematics and dynamics library

Simple, elegant Python API for robot modeling and control.
"""

from ._robospace_core import (
    # Math types
    SE3,
    SO3,
    Rotation,
    Transform,

    # Model types
    Robot,
    Link,
    Joint,
    Tool,
    Frame,

    # Joint types
    JointType,
)

__version__ = "0.1.0"

__all__ = [
    # Math
    "SE3",
    "SO3",
    "Rotation",
    "Transform",

    # Model
    "Robot",
    "Link",
    "Joint",
    "Tool",
    "Frame",
    "JointType",
]
