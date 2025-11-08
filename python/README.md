# Robospace Python Package

This directory contains the Python bindings and API for robospace.

## Structure

- **bindings/** - pybind11 C++ binding code
- **robospace/** - Pure Python API layer

## Installation

```bash
pip install -e .
```

## Usage

```python
import robospace as rs

# Create SE(3) transformation
T = rs.Transform(translation=[1, 2, 3])
print(T.matrix())

# Use Lie algebra
import numpy as np
xi = rs.se3(omega=[0, 0, 1.57], v=[1, 0, 0])
g = rs.exp_se3(xi)
```
