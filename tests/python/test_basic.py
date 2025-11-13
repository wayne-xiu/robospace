"""
Basic tests for Python bindings
"""
import sys
import os

# Add python module to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../python'))

import robospace as rs
import numpy as np


def test_import():
    """Test that module imports successfully"""
    assert rs is not None
    assert hasattr(rs, 'Robot')
    assert hasattr(rs, 'SE3')


def test_se3_basic():
    """Test SE3 basic functionality"""
    # Identity
    T = rs.SE3()
    assert T is not None

    # Translation
    t = T.translation()
    assert len(t) == 3
    assert np.allclose(t, [0, 0, 0])

    # Rotation
    R = T.rotation()
    assert R.shape == (3, 3)
    assert np.allclose(R, np.eye(3))


def test_transform_basic():
    """Test Transform basic functionality"""
    # Identity
    T = rs.Transform()
    assert T is not None

    M = T.matrix()
    assert M.shape == (4, 4)
    assert np.allclose(M, np.eye(4))

    # Translation
    T2 = rs.Transform.Translation(1.0, 2.0, 3.0)
    t = T2.translation()
    assert np.allclose(t, [1, 2, 3])


def test_robot_from_urdf():
    """Test loading robot from URDF"""
    urdf_path = os.path.join(os.path.dirname(__file__), '../test_data/simple_2r.urdf')

    robot = rs.Robot.from_urdf(urdf_path)
    assert robot is not None
    assert robot.dof() == 2
    assert robot.num_links() > 0
    assert robot.num_joints() >= 2


def test_robot_fk():
    """Test robot forward kinematics"""
    urdf_path = os.path.join(os.path.dirname(__file__), '../test_data/simple_2r.urdf')
    robot = rs.Robot.from_urdf(urdf_path)

    # FK at zero configuration
    q = np.array([0.0, 0.0])
    
    # FK returns SE3 object
    T = robot.fk(q, "link2")
    assert T is not None

    # Check translation
    t = T.translation()
    assert len(t) == 3

    # Check rotation
    R = T.rotation()
    assert R.shape == (3, 3)


def test_robot_jacobian():
    """Test robot Jacobian computation"""
    urdf_path = os.path.join(os.path.dirname(__file__), '../test_data/simple_2r.urdf')
    robot = rs.Robot.from_urdf(urdf_path)

    q = np.array([0.0, 0.0])

    # Jacobian in base frame
    J0 = robot.jacob0(q)
    assert J0.shape == (6, 2)

    # Jacobian in EE frame
    Je = robot.jacobe(q)
    assert Je.shape == (6, 2)

    # Manipulability
    m = robot.manipulability(q)
    assert m >= 0


if __name__ == '__main__':
    print("Running basic Python binding tests...")

    test_import()
    print("✓ Import test passed")

    test_se3_basic()
    print("✓ SE3 test passed")

    test_transform_basic()
    print("✓ Transform test passed")

    test_robot_from_urdf()
    print("✓ Robot URDF loading test passed")

    test_robot_fk()
    print("✓ Robot FK test passed")

    test_robot_jacobian()
    print("✓ Robot Jacobian test passed")

    print("\n✅ All tests passed!")
