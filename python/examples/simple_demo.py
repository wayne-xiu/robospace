"""
Simple demo of RoboSpace Python API
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import robospace as rs
import numpy as np

print("RoboSpace Python Bindings Demo")
print("=" * 40)

# SE3 transformations
print("\n1. SE3 Transformations:")
T = rs.SE3()
print(f"   Identity SE3: {T.translation()}")

# Transform
print("\n2. Transform (classical 4x4 matrices):")
T = rs.Transform.Translation(1.0, 2.0, 3.0)
print(f"   Translation: {T.translation()}")

# Rotation
print("\n3. SO3 Rotations:")
R = rs.SO3.RotZ(np.pi/2)  # 90 degrees around Z
print(f"   Rotation matrix shape: {R.matrix().shape}")

print("\n✅ Python bindings working correctly!")
