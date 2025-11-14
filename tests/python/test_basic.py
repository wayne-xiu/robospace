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


def test_lie_algebra():
    """Test Lie algebra types and exponential maps"""
    # se3 - twist
    omega = np.array([0, 0, 1.57])  # 90° rotation around Z
    v = np.array([1, 0, 0])
    xi = rs.make_se3(omega, v)
    assert xi is not None

    # Check vector representation
    xi_vec = xi.vector()
    assert xi_vec.shape == (6,)
    assert np.allclose(xi_vec[:3], omega)
    assert np.allclose(xi_vec[3:], v)

    # Exponential map: se3 -> SE3
    g = rs.exp_se3(xi)
    assert g is not None
    assert isinstance(g, rs.SE3)

    # Check we can get the matrix
    mat = g.matrix()
    assert mat.shape == (4, 4)

    # Logarithm map: SE3 -> se3 (roundtrip)
    xi_back = rs.log_SE3(g)
    assert xi_back is not None
    xi_back_vec = xi_back.vector()
    assert np.allclose(xi_back_vec, xi_vec, atol=1e-6)

    # so3 - angular velocity
    omega2 = np.array([0, 0, np.pi/2])
    w = rs.make_so3(omega2)
    assert w is not None

    omega_vec = w.vector()
    assert omega_vec.shape == (3,)
    assert np.allclose(omega_vec, omega2)

    # Exponential map: so3 -> SO3
    R = rs.exp_so3(w)
    assert R is not None
    assert isinstance(R, rs.SO3)

    # Logarithm map: SO3 -> so3 (roundtrip)
    w_back = rs.log_SO3(R)
    w_back_vec = w_back.vector()
    assert np.allclose(w_back_vec, omega_vec, atol=1e-6)


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

    test_lie_algebra()
    print("✓ Lie algebra test passed")

    test_robot_from_urdf()
    print("✓ Robot URDF loading test passed")

    test_robot_fk()
    print("✓ Robot FK test passed")

    test_robot_jacobian()
    print("✓ Robot Jacobian test passed")

    print("\n✅ All tests passed!")
