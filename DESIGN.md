# Robospace: Architecture & Design Document

**Version:** 1.0
**Date:** 2025-11-08
**Status:** Initial Design

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Vision & Goals](#vision--goals)
3. [System Architecture](#system-architecture)
4. [Technology Stack](#technology-stack)
5. [Core Modules](#core-modules)
6. [API Design](#api-design)
7. [Web Application](#web-application)
8. [Development Phases](#development-phases)
9. [Testing Strategy](#testing-strategy)
10. [Deployment & DevOps](#deployment--devops)
11. [Performance Considerations](#performance-considerations)
12. [Security & Safety](#security--safety)
13. [Future Roadmap](#future-roadmap)

---

## Executive Summary

**Robospace** is a modern, modular robotics framework designed for industrial robot applications with a focus on arm and collaborative robots. It combines high-performance C++ core libraries with an accessible Python API and a contemporary web-based user interface.

### Key Differentiators

- **Modern Web UI**: Browser-based 3D visualization and control (no Qt dependencies)
- **Computer Vision First-Class**: Built-in machine vision for robotics applications
- **Dual-Language Core**: C++ for performance-critical operations, Python for accessibility
- **Modular Architecture**: Composable libraries that can be used independently or together
- **Cloud-Ready**: Deployable locally or in cloud environments
- **Industry-Focused**: Targeting industrial automation and collaborative robotics

---

## Vision & Goals

### Primary Vision

Create a comprehensive, modern robotics framework that bridges the gap between research flexibility and industrial robustness, while providing an exceptional user experience through web-based interfaces.

### Goals

1. **Performance**: Match or exceed C++-based libraries (robotics-library, MoveIt) in computation speed
2. **Accessibility**: Provide Python API that's intuitive for researchers and students
3. **Modularity**: Allow users to use individual components without the full framework
4. **Vision Integration**: First-class support for computer/machine vision workflows
5. **Modern UX**: Web-based interface accessible from any device
6. **Production-Ready**: Suitable for industrial deployments with safety and reliability
7. **Extensibility**: Plugin architecture for custom robots, algorithms, and sensors

### Non-Goals (Initial Scope)

- Mobile robot navigation (future phase)
- Humanoid robotics
- Soft robotics
- Micro-controller embedded systems (focus on PC-level control)

---

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        User Layer                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Web Browser  │  │ Python Apps  │  │   ROS Nodes  │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
└─────────┼──────────────────┼──────────────────┼─────────────┘
          │                  │                  │
┌─────────┼──────────────────┼──────────────────┼─────────────┐
│         │    Application Layer                │             │
│  ┌──────▼───────┐    ┌────▼──────────┐  ┌────▼────────┐    │
│  │   FastAPI    │    │  Python API   │  │  ROS Bridge │    │
│  │   Backend    │    │  (robospace)  │  │             │    │
│  └──────┬───────┘    └────┬──────────┘  └────┬────────┘    │
│         │                 │                   │             │
│         │          ┌──────▼──────────┐        │             │
│         └──────────►  Python Bindings ◄────────┘             │
│                    │   (pybind11)    │                      │
│                    └──────┬──────────┘                      │
└───────────────────────────┼─────────────────────────────────┘
                            │
┌───────────────────────────┼─────────────────────────────────┐
│                     Core Layer (C++)                        │
│  ┌──────────────┬──────────────┬──────────────┬──────────┐  │
│  │  Kinematics  │   Dynamics   │   Planning   │  Vision  │  │
│  ├──────────────┼──────────────┼──────────────┼──────────┤  │
│  │  Collision   │  Simulation  │     Math     │    I/O   │  │
│  └──────────────┴──────────────┴──────────────┴──────────┘  │
└───────────────────────────────┬─────────────────────────────┘
                                │
┌───────────────────────────────┼─────────────────────────────┐
│                      Hardware Layer                         │
│  ┌──────────────┬──────────────┬──────────────┬──────────┐  │
│  │ Robot Drivers│   Cameras    │   Sensors    │  Grippers│  │
│  └──────────────┴──────────────┴──────────────┴──────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### Component Interactions

```
Web Frontend (React + Three.js)
    ↕ [WebSocket/REST]
FastAPI Backend
    ↕ [Python Function Calls]
Robospace Python API
    ↕ [pybind11 Bindings]
C++ Core Libraries
    ↕ [Driver APIs]
Hardware (Robots, Cameras, Sensors)
```

---

## Technology Stack

### Core Libraries (C++)

| Component | Technology | Purpose |
|-----------|------------|---------|
| Language | C++17/20 | Modern features, performance |
| Build System | CMake 3.15+ | Cross-platform builds |
| Linear Algebra | Eigen 3.4+ | Matrix operations, transforms |
| Collision Detection | FCL (Flexible Collision Library) | Fast collision checking |
| Motion Planning | OMPL (Open Motion Planning Library) | Sampling-based planners |
| Physics Engine | Bullet3 / MuJoCo | Rigid body dynamics simulation |
| Computer Vision | OpenCV 4.x | Image processing, calibration |
| Point Clouds | PCL 1.12+ / Open3D | 3D perception |
| Logging | spdlog | Fast, structured logging |
| Testing | Google Test / Catch2 | Unit testing |
| Benchmarking | Google Benchmark | Performance testing |

### Python Layer

| Component | Technology | Purpose |
|-----------|------------|---------|
| Bindings | pybind11 | C++ ↔ Python interface |
| Build | scikit-build-core | Python package with C++ |
| Numerics | NumPy | Array operations |
| Scientific | SciPy | Optimization, signal processing |
| Vision | OpenCV-Python | Image processing in Python |
| Testing | pytest | Python unit testing |
| Type Checking | mypy | Static type analysis |
| Linting | ruff | Fast Python linter |

### Web Application

| Component | Technology | Purpose |
|-----------|------------|---------|
| Frontend Framework | React 18+ or Vue 3+ | UI components, state management |
| 3D Visualization | Three.js | WebGL-based 3D rendering |
| Robot Rendering | urdf-loader (Three.js) | URDF model visualization |
| UI Components | Material-UI / Ant Design | Consistent UI design |
| State Management | Redux / Zustand | Application state |
| API Client | Axios / Fetch API | HTTP requests |
| WebSocket | Socket.io / native WebSocket | Real-time communication |
| Build Tool | Vite | Fast dev server and bundling |
| Type Safety | TypeScript | Static typing for JavaScript |

### Backend (Web Server)

| Component | Technology | Purpose |
|-----------|------------|---------|
| Framework | FastAPI | High-performance async API |
| WebSocket | FastAPI WebSocket / Socket.io | Real-time updates |
| Validation | Pydantic | Request/response validation |
| Authentication | JWT / OAuth2 | User authentication |
| Database | PostgreSQL / SQLite | Persistent data storage |
| ORM | SQLAlchemy | Database abstraction |
| Task Queue | Celery (optional) | Background jobs |

### DevOps & Tooling

| Component | Technology | Purpose |
|-----------|------------|---------|
| Version Control | Git | Source control |
| CI/CD | GitHub Actions | Automated testing, builds |
| Containerization | Docker | Deployment packaging |
| Documentation | Sphinx (Python), Doxygen (C++) | API documentation |
| Package Manager (C++) | Conan / vcpkg | Dependency management |
| Package Manager (Python) | pip / Poetry | Python dependencies |
| Code Quality | clang-format, clang-tidy | C++ code style, analysis |

### Optional Integrations

| Component | Technology | Purpose |
|-----------|------------|---------|
| ROS | ROS2 Humble+ | Robotics middleware |
| Cloud | AWS / Azure / GCP | Cloud deployment |
| Monitoring | Prometheus + Grafana | Performance monitoring |
| ML/AI | PyTorch / TensorFlow | Vision, learning-based control |

---

## Core Modules

### 1. Math Library (`core/math`)

**Purpose**: Foundational mathematical operations for robotics

**Components**:
- **Transform**: SE(3) transformations, homogeneous matrices
- **Rotation**: SO(3) rotations (rotation matrices, quaternions, Euler angles, axis-angle)
- **Vector/Matrix**: Thin wrappers around Eigen with robotics-specific operations
- **Interpolation**: Linear, cubic spline, SLERP for rotations
- **Utilities**: Angle normalization, numeric differentiation

**Key Classes**:
```cpp
namespace robospace::math {
    class Transform;     // SE(3) rigid transformation
    class Rotation;      // SO(3) rotation representation
    class Vector3;       // 3D vector
    class Quaternion;    // Unit quaternion
    class Twist;         // Velocity in SE(3)
    class Wrench;        // Force-torque in SE(3)
}
```

**Dependencies**: Eigen

---

### 2. Robot Model (`core/model`)

**Purpose**: Robot representation, kinematics chains, link/joint modeling

**Components**:
- **Joint**: Revolute, prismatic, fixed joints
- **Link**: Physical properties (mass, inertia, geometry)
- **KinematicChain**: Serial/tree structure of joints and links
- **URDF Parser**: Load robot models from URDF/XACRO
- **Robot**: High-level robot representation

**Key Classes**:
```cpp
namespace robospace::model {
    class Joint;
    class Link;
    class KinematicChain;
    class Robot;
    class URDFParser;
}
```

**File Formats**: URDF, custom JSON/YAML

**Dependencies**: TinyXML2 (URDF parsing), Eigen

---

### 3. Kinematics (`core/kinematics`)

**Purpose**: Forward and inverse kinematics calculations

**Components**:
- **Forward Kinematics (FK)**: Joint positions → end-effector pose
- **Inverse Kinematics (IK)**: End-effector pose → joint positions
  - Analytical IK (6-DOF industrial robots)
  - Numerical IK (Jacobian-based, TRAC-IK)
- **Jacobian**: Velocity kinematics, singularity analysis
- **Differential IK**: Velocity-level IK

**Key Classes**:
```cpp
namespace robospace::kinematics {
    class ForwardKinematics;
    class InverseKinematics;
    class JacobianCalculator;
    class IKSolver;           // Base class
    class AnalyticalIK;       // Robot-specific
    class NumericalIK;        // General-purpose
}
```

**Algorithms**:
- DH parameter-based FK
- Analytical IK for common 6-DOF manipulators (UR, ABB, KUKA)
- Damped least squares
- TRAC-IK algorithm

**Dependencies**: Eigen, model module

---

### 4. Dynamics (`core/dynamics`)

**Purpose**: Robot dynamics calculations

**Components**:
- **Forward Dynamics**: Torques → accelerations
- **Inverse Dynamics**: Accelerations → torques (for control)
- **Mass Matrix**: Configuration-dependent inertia
- **Gravity/Coriolis**: Compensation terms
- **Lagrangian/Newton-Euler**: Different formulations

**Key Classes**:
```cpp
namespace robospace::dynamics {
    class DynamicsCalculator;
    class MassMatrix;
    class GravityCompensation;
    class CoriolisMatrix;
}
```

**Algorithms**:
- Recursive Newton-Euler Algorithm (RNEA)
- Composite Rigid Body Algorithm (CRBA)
- Articulated Body Algorithm (ABA)

**Dependencies**: Eigen, model, kinematics

---

### 5. Collision Detection (`core/collision`)

**Purpose**: Collision checking for motion planning and safety

**Components**:
- **Collision Objects**: Robot links, obstacles, environments
- **Collision Checker**: Pairwise and global collision detection
- **Distance Queries**: Minimum distance between objects
- **Self-Collision**: Robot self-collision checking

**Key Classes**:
```cpp
namespace robospace::collision {
    class CollisionObject;
    class CollisionChecker;
    class CollisionWorld;
    class DistanceQuery;
}
```

**Backend**: FCL (Flexible Collision Library)

**Dependencies**: FCL, model

---

### 6. Motion Planning (`core/planning`)

**Purpose**: Path and trajectory planning

**Components**:
- **Path Planning**: Collision-free paths in configuration space
  - RRT, RRT*, PRM, KPIECE
  - Cartesian planning
- **Trajectory Generation**: Time-optimal, jerk-limited trajectories
  - Cubic/quintic polynomials
  - B-splines
  - Time parameterization (TOPP-RA)
- **Local Planning**: Interpolation, smoothing
- **Global Planning**: Multi-query planning

**Key Classes**:
```cpp
namespace robospace::planning {
    class MotionPlanner;
    class PathPlanner;         // OMPL integration
    class TrajectoryGenerator;
    class Trajectory;
    class Waypoint;
    class PlanningScene;
}
```

**Algorithms**:
- Sampling-based: RRT, RRT-Connect, RRT*, PRM
- Optimization-based: CHOMP, TrajOpt (future)
- Interpolation: Linear, cubic spline, B-spline

**Dependencies**: OMPL, collision, kinematics

---

### 7. Simulation (`core/simulation`)

**Purpose**: Physics-based robot simulation

**Components**:
- **Physics Engine**: Rigid body dynamics (Bullet/MuJoCo wrapper)
- **Simulated Robot**: Virtual robot with actuators and sensors
- **World**: Simulation environment with objects
- **Sensor Models**: Cameras, force sensors, encoders
- **Controller Interface**: Apply commands, read feedback

**Key Classes**:
```cpp
namespace robospace::simulation {
    class Simulator;
    class SimulatedRobot;
    class SimulationWorld;
    class PhysicsEngine;      // Abstract
    class BulletEngine;       // Bullet implementation
    class MuJoCoEngine;       // MuJoCo implementation
}
```

**Dependencies**: Bullet3 or MuJoCo, model, dynamics

---

### 8. Computer Vision (`vision/`)

**Purpose**: Machine vision for robotic applications

**Components**:

#### 8.1 Calibration (`vision/calibration`)
- **Camera Calibration**: Intrinsic parameters (focal length, distortion)
- **Hand-Eye Calibration**: Camera-to-robot transformation
- **Stereo Calibration**: Stereo camera pairs

#### 8.2 Detection (`vision/detection`)
- **Marker Detection**: ArUco, AprilTag
- **Object Detection**: Template matching, feature-based
- **Pose Estimation**: PnP (Perspective-n-Point)

#### 8.3 Perception (`vision/perception`)
- **Point Cloud Processing**: Segmentation, filtering
- **Surface Reconstruction**: Mesh generation
- **3D Object Recognition**: PCL-based methods

#### 8.4 Tracking (`vision/tracking`)
- **Object Tracking**: KCF, CSRT, optical flow
- **Visual Servoing**: PBVS (position-based), IBVS (image-based)

**Key Classes**:
```cpp
namespace robospace::vision {
    // Calibration
    class CameraCalibrator;
    class HandEyeCalibrator;
    class CameraIntrinsics;

    // Detection
    class MarkerDetector;
    class ObjectDetector;
    class PoseEstimator;

    // Perception
    class PointCloudProcessor;
    class ObjectRecognizer;

    // Tracking
    class ObjectTracker;
    class VisualServo;
}
```

**Dependencies**: OpenCV, PCL or Open3D, Eigen

---

### 9. Hardware Abstraction (`hardware/`)

**Purpose**: Unified interface for robot hardware

**Components**:

#### 9.1 Robot Drivers (`hardware/drivers`)
- **Driver Interface**: Abstract base class for robot control
- **Manufacturer Drivers**: UR, ABB, KUKA, Fanuc plugins
- **Communication**: TCP/IP, UDP, EtherCAT, Modbus

#### 9.2 I/O (`hardware/io`)
- **Digital I/O**: GPIO control
- **Analog I/O**: Sensor reading
- **Network I/O**: Socket communication

#### 9.3 Controllers (`hardware/controllers`)
- **Position Control**: Joint/Cartesian position
- **Velocity Control**: Joint/Cartesian velocity
- **Force Control**: Impedance, admittance control
- **Safety Monitoring**: Limits, e-stops, collision detection

**Key Classes**:
```cpp
namespace robospace::hardware {
    class RobotDriver;         // Abstract interface
    class URDriver;            // Universal Robots
    class ABBDriver;           // ABB robots
    class KUKADriver;          // KUKA robots

    class ControlInterface;
    class JointController;
    class CartesianController;
    class ForceController;
}
```

**Communication Protocols**:
- Universal Robots: UR RTDE (Real-Time Data Exchange)
- ABB: Externally Guided Motion (EGM)
- KUKA: RSI (Robot Sensor Interface)

**Dependencies**: Asio (networking), model

---

## API Design

### C++ API Philosophy

**Principles**:
1. **RAII**: Resource management via constructors/destructors
2. **Move Semantics**: Efficient transfers, no unnecessary copies
3. **Const Correctness**: Clear mutation semantics
4. **Exception Safety**: Strong exception guarantee where possible
5. **Modern C++**: Use C++17/20 features (std::optional, std::variant, concepts)

**Example: Kinematics API**

```cpp
#include <robospace/kinematics.h>
#include <robospace/model.h>

using namespace robospace;

// Load robot model
auto robot = model::Robot::from_urdf("ur5e.urdf");

// Create FK solver
kinematics::ForwardKinematics fk(robot);

// Joint configuration
Eigen::VectorXd joint_positions(6);
joint_positions << 0, -M_PI/2, M_PI/2, 0, M_PI/2, 0;

// Compute end-effector pose
math::Transform ee_pose = fk.compute(joint_positions);
std::cout << "Position: " << ee_pose.translation().transpose() << std::endl;

// Create IK solver
auto ik = kinematics::NumericalIK::create(robot, "base_link", "tool0");

// Target pose
math::Transform target;
target.translation() << 0.4, 0.2, 0.3;
target.linear() = math::Rotation::from_euler(0, M_PI, 0).matrix();

// Solve IK
auto result = ik->solve(target, joint_positions);  // seed with current config
if (result.has_value()) {
    std::cout << "IK Solution: " << result->transpose() << std::endl;
} else {
    std::cerr << "IK failed" << std::endl;
}
```

**Example: Motion Planning API**

```cpp
#include <robospace/planning.h>

using namespace robospace;

// Create planning scene
planning::PlanningScene scene(robot);
scene.add_box_obstacle("table", {0.5, 0.3, 0.0}, {1.0, 0.6, 0.02});

// Create planner
planning::MotionPlanner planner(robot, scene);
planner.set_planner_type(planning::PlannerType::RRTConnect);
planner.set_planning_time(5.0);  // 5 seconds max

// Plan from current to target
Eigen::VectorXd start = joint_positions;
Eigen::VectorXd goal(6);
goal << M_PI/4, -M_PI/3, M_PI/2, 0, M_PI/2, M_PI/4;

auto trajectory = planner.plan(start, goal);
if (trajectory) {
    std::cout << "Path found with " << trajectory->waypoints.size()
              << " waypoints" << std::endl;

    // Time parameterize
    auto timed_traj = planning::time_parameterize(*trajectory,
                                                   max_vel, max_acc);
}
```

---

### Python API Philosophy

**Principles**:
1. **Pythonic**: Follow PEP 8, Python naming conventions
2. **NumPy Integration**: Seamless conversion between C++ Eigen and NumPy
3. **Type Hints**: Full type annotations for IDE support
4. **Documentation**: Comprehensive docstrings (Google style)
5. **Error Handling**: Translate C++ exceptions to Python exceptions

**Example: Python Kinematics API**

```python
import robospace as rs
import numpy as np

# Load robot
robot = rs.Robot.from_urdf("ur5e.urdf")

# Forward kinematics
fk = rs.ForwardKinematics(robot)
joint_pos = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
ee_pose = fk.compute(joint_pos)
print(f"Position: {ee_pose.translation}")
print(f"Rotation: {ee_pose.rotation.as_matrix()}")

# Inverse kinematics
ik = rs.InverseKinematics(robot, base_link="base_link", tip_link="tool0")
target_pose = rs.Transform(
    translation=[0.4, 0.2, 0.3],
    rotation=rs.Rotation.from_euler([0, np.pi, 0])
)

solution = ik.solve(target_pose, seed=joint_pos)
if solution is not None:
    print(f"IK solution: {solution}")
else:
    print("IK failed")
```

**Example: Python Planning API**

```python
import robospace as rs

# Create planning scene
scene = rs.PlanningScene(robot)
scene.add_box("table", position=[0.5, 0.3, 0.0], size=[1.0, 0.6, 0.02])

# Motion planner
planner = rs.MotionPlanner(robot, scene)
planner.planner_type = "RRTConnect"
planner.planning_time = 5.0

# Plan motion
start = np.array([0, 0, 0, 0, 0, 0])
goal = np.array([np.pi/4, -np.pi/3, np.pi/2, 0, np.pi/2, np.pi/4])

trajectory = planner.plan(start, goal)
if trajectory:
    print(f"Found path with {len(trajectory.waypoints)} waypoints")

    # Time parameterization
    timed_traj = rs.time_parameterize(trajectory, max_vel, max_acc)

    # Execute in simulation
    sim = rs.Simulator(robot)
    sim.execute(timed_traj)
```

**Example: Vision API**

```python
import robospace.vision as rsv
import cv2

# Camera calibration
calibrator = rsv.CameraCalibrator()
for image in calibration_images:
    calibrator.add_image(image, chessboard_size=(9, 6))
intrinsics = calibrator.calibrate()

# Hand-eye calibration
hand_eye = rsv.HandEyeCalibrator()
for robot_pose, marker_pose in calibration_data:
    hand_eye.add_sample(robot_pose, marker_pose)
camera_to_robot = hand_eye.calibrate()

# Object detection and pose estimation
detector = rsv.MarkerDetector(marker_type="aruco", dictionary="4x4_50")
camera = rsv.Camera(intrinsics)

frame = camera.capture()
markers = detector.detect(frame)
for marker in markers:
    pose = marker.estimate_pose(intrinsics)
    print(f"Marker {marker.id} at {pose.translation}")

    # Transform to robot frame
    object_in_robot = camera_to_robot * pose
```

---

### REST API (FastAPI Backend)

**Base URL**: `http://localhost:8000/api/v1`

**Authentication**: JWT tokens (optional for local, required for cloud)

#### Robot Management

```
GET    /robots                    # List available robots
POST   /robots                    # Load robot from URDF
GET    /robots/{robot_id}         # Get robot info
DELETE /robots/{robot_id}         # Unload robot
GET    /robots/{robot_id}/state   # Get current joint state
POST   /robots/{robot_id}/state   # Set joint state (simulation)
```

#### Kinematics

```
POST   /robots/{robot_id}/fk      # Compute forward kinematics
POST   /robots/{robot_id}/ik      # Compute inverse kinematics
POST   /robots/{robot_id}/jacobian # Compute Jacobian
```

**Example Request**:
```json
POST /api/v1/robots/ur5e/ik
{
  "target_pose": {
    "position": [0.4, 0.2, 0.3],
    "orientation": [0, 3.14159, 0]  // Euler angles
  },
  "seed": [0, -1.57, 1.57, 0, 1.57, 0]
}
```

**Example Response**:
```json
{
  "success": true,
  "solution": [0.785, -1.047, 1.57, 0, 1.57, 0.785],
  "computation_time_ms": 15.3
}
```

#### Motion Planning

```
POST   /robots/{robot_id}/plan    # Plan motion
GET    /plans/{plan_id}           # Get plan details
POST   /plans/{plan_id}/execute   # Execute plan
```

#### Scene Management

```
GET    /scenes                    # List scenes
POST   /scenes                    # Create scene
POST   /scenes/{scene_id}/objects # Add obstacle
DELETE /scenes/{scene_id}/objects/{object_id}
```

#### Hardware Control

```
GET    /hardware/robots           # List connected robots
POST   /hardware/robots/connect   # Connect to robot
POST   /hardware/robots/{id}/execute  # Execute trajectory
GET    /hardware/robots/{id}/state    # Real robot state
```

---

### WebSocket API

**Connection**: `ws://localhost:8000/ws`

**Real-time Updates**:

```javascript
// Client subscribes to robot state
{
  "type": "subscribe",
  "topic": "robot_state",
  "robot_id": "ur5e"
}

// Server sends updates at 10-100 Hz
{
  "type": "robot_state",
  "robot_id": "ur5e",
  "timestamp": 1699459200.123,
  "joint_positions": [0, -1.57, 1.57, 0, 1.57, 0],
  "joint_velocities": [0, 0, 0, 0, 0, 0],
  "ee_pose": {
    "position": [0.4, 0.2, 0.5],
    "orientation": [0, 3.14159, 0]
  }
}
```

**Topics**:
- `robot_state`: Joint and Cartesian state
- `trajectory_progress`: Execution progress
- `collision_state`: Collision detection results
- `camera_feed`: Video stream (base64 encoded frames)
- `planning_visualization`: Planning tree visualization

---

## Web Application

### Frontend Architecture

**Framework**: React 18+ with TypeScript

**State Management**: Zustand (lightweight) or Redux Toolkit

**Component Structure**:

```
src/
├── components/
│   ├── Robot3D/              # 3D robot visualization
│   │   ├── RobotViewer.tsx   # Main Three.js canvas
│   │   ├── URDFLoader.ts     # Load URDF models
│   │   └── Controls.tsx      # Orbit controls, lighting
│   ├── Controls/
│   │   ├── JointSliders.tsx  # Manual joint control
│   │   ├── CartesianJog.tsx  # XYZ position control
│   │   └── TeachMode.tsx     # Freedrive mode
│   ├── Planning/
│   │   ├── MotionPlan.tsx    # Planning interface
│   │   ├── Trajectory.tsx    # Trajectory visualization
│   │   └── Obstacles.tsx     # Scene editing
│   ├── Vision/
│   │   ├── CameraView.tsx    # Live camera feed
│   │   ├── Calibration.tsx   # Calibration tools
│   │   └── Detection.tsx     # Object detection overlay
│   └── Hardware/
│       ├── RobotConnect.tsx  # Connect to real robots
│       └── Dashboard.tsx     # Status monitoring
├── scenes/
│   ├── Simulation.tsx        # Simulation workspace
│   ├── Programming.tsx       # Offline programming
│   └── Calibration.tsx       # Vision calibration
├── stores/
│   ├── robotStore.ts         # Robot state
│   ├── sceneStore.ts         # 3D scene objects
│   └── uiStore.ts            # UI preferences
├── services/
│   ├── api.ts                # REST API client
│   └── websocket.ts          # WebSocket manager
└── utils/
    ├── transforms.ts         # Math utilities
    └── urdf.ts               # URDF parsing helpers
```

### 3D Visualization

**Library**: Three.js with React Three Fiber

**Features**:
- URDF model loading and rendering
- Real-time joint state updates (60 FPS target)
- Trajectory visualization (animated paths)
- Collision geometry display (wireframes)
- Point cloud rendering (for vision)
- Interactive controls (orbit, pan, zoom)
- Lighting and shadows for realism

**Performance**:
- Use instancing for repeated geometries
- Level-of-detail (LOD) for complex meshes
- Frustum culling for large scenes

### UI/UX Design

**Layout**:
```
┌─────────────────────────────────────────────────────────┐
│  Header: Logo | Robot Select | Connection Status       │
├─────────────┬───────────────────────────────────────────┤
│             │                                           │
│   Sidebar   │          3D Viewport                      │
│   Tools:    │          (Three.js)                       │
│   - FK/IK   │                                           │
│   - Plan    │                                           │
│   - Vision  │                                           │
│   - Control │                                           │
│             │                                           │
│             │                                           │
├─────────────┴───────────────────────────────────────────┤
│  Bottom Panel: Logs | Joint Values | Trajectory Timeline│
└─────────────────────────────────────────────────────────┘
```

**Color Scheme**: Dark theme (easier on eyes for long sessions)

**Key Interactions**:
1. Click robot in 3D → Select link/joint
2. Drag end-effector → IK solve in real-time
3. Right-click 3D space → Add obstacle
4. Timeline scrubbing → Animate trajectory

---

### Backend Architecture (FastAPI)

**Project Structure**:

```
backend/
├── app/
│   ├── main.py               # FastAPI app initialization
│   ├── config.py             # Configuration management
│   ├── api/
│   │   ├── v1/
│   │   │   ├── robots.py     # Robot endpoints
│   │   │   ├── kinematics.py # FK/IK endpoints
│   │   │   ├── planning.py   # Motion planning
│   │   │   ├── vision.py     # Vision endpoints
│   │   │   └── hardware.py   # Hardware control
│   │   └── deps.py           # Dependency injection
│   ├── models/
│   │   ├── robot.py          # Pydantic models
│   │   ├── kinematics.py
│   │   └── planning.py
│   ├── services/
│   │   ├── robot_service.py  # Business logic
│   │   ├── planning_service.py
│   │   └── vision_service.py
│   ├── websocket/
│   │   ├── manager.py        # WebSocket connection manager
│   │   └── handlers.py       # Message handlers
│   └── database/
│       ├── models.py         # SQLAlchemy models
│       └── crud.py           # Database operations
├── tests/
└── requirements.txt
```

**Example Endpoint**:

```python
# app/api/v1/kinematics.py
from fastapi import APIRouter, Depends, HTTPException
from app.models.kinematics import IKRequest, IKResponse
from app.services.robot_service import RobotService
import robospace as rs

router = APIRouter()

@router.post("/{robot_id}/ik", response_model=IKResponse)
async def compute_inverse_kinematics(
    robot_id: str,
    request: IKRequest,
    robot_service: RobotService = Depends()
):
    """Compute inverse kinematics for target pose."""
    robot = robot_service.get_robot(robot_id)
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not found")

    ik_solver = rs.InverseKinematics(robot.model)

    target_pose = rs.Transform(
        translation=request.target_pose.position,
        rotation=rs.Rotation.from_euler(request.target_pose.orientation)
    )

    solution = ik_solver.solve(target_pose, seed=request.seed)

    if solution is None:
        return IKResponse(success=False, error="IK solution not found")

    return IKResponse(
        success=True,
        solution=solution.tolist(),
        computation_time_ms=ik_solver.last_computation_time
    )
```

---

## Development Phases

### Phase 1: Foundation (Months 1-2)

**Goal**: Establish core infrastructure

**Tasks**:
- [x] Project repository setup
- [ ] CMake build system for C++ libraries
- [ ] Directory structure implementation
- [ ] Math library (Transform, Rotation classes)
- [ ] Robot model representation (Joint, Link, KinematicChain)
- [ ] URDF parser integration
- [ ] Forward kinematics implementation
- [ ] pybind11 setup for Python bindings
- [ ] Basic Python API for math and FK
- [ ] Unit tests (C++ with GTest, Python with pytest)
- [ ] CI/CD pipeline (GitHub Actions)
- [ ] Documentation setup (Sphinx, Doxygen)

**Deliverables**:
- C++ library: `librobospace-core.so`
- Python package: `pip install robospace`
- Example: Load URDF, compute FK

**Testing**:
- Unit tests: 80%+ coverage
- Test robots: UR5, Panda, custom 6-DOF

**Milestone**: Can load a robot and compute forward kinematics from Python

---

### Phase 2: Kinematics & Basic Planning (Months 3-4)

**Goal**: Complete kinematics and simple motion planning

**Tasks**:
- [ ] Inverse kinematics (analytical for UR, KUKA)
- [ ] Numerical IK solver (damped least squares, Levenberg-Marquardt)
- [ ] Jacobian calculator (geometric, analytic)
- [ ] Collision detection integration (FCL)
- [ ] Basic trajectory generation (joint space, linear interpolation)
- [ ] OMPL integration for path planning
- [ ] Simple planners: RRT, RRT-Connect
- [ ] Python API for IK, Jacobian, planning
- [ ] Performance benchmarks
- [ ] Extended documentation and examples

**Deliverables**:
- IK solver for common robots
- RRT-based path planner
- Trajectory generation utilities

**Testing**:
- IK accuracy tests (analytical vs numerical)
- Planning success rate on standard benchmarks
- Performance: FK < 10μs, IK < 1ms, Planning < 5s

**Milestone**: Can plan collision-free paths for robot arms

---

### Phase 3: Web Interface (Months 5-6)

**Goal**: Build web-based UI for visualization and control

**Tasks**:
- [ ] FastAPI backend setup
- [ ] REST API implementation (robots, kinematics, planning)
- [ ] WebSocket server for real-time updates
- [ ] React frontend initialization (Vite + TypeScript)
- [ ] Three.js integration
- [ ] URDF loader for Three.js (urdf-loader library)
- [ ] 3D robot visualization
- [ ] Joint sliders for manual control
- [ ] IK interface (drag end-effector)
- [ ] Motion planning UI
- [ ] Trajectory visualization and playback
- [ ] Scene editing (add/remove obstacles)
- [ ] Responsive design (desktop focus, mobile-friendly)

**Deliverables**:
- Web application accessible at `localhost:8000`
- 3D robot visualization
- Planning and simulation interface

**Testing**:
- Frontend unit tests (Jest)
- End-to-end tests (Playwright)
- Performance: 60 FPS rendering, < 100ms API latency

**Milestone**: Web app for robot visualization and motion planning

---

### Phase 4: Dynamics & Simulation (Months 7-8)

**Goal**: Physics-based simulation

**Tasks**:
- [ ] Dynamics module (RNEA, CRBA)
- [ ] Gravity compensation
- [ ] Bullet physics engine integration
- [ ] Simulated robot with actuators
- [ ] Simulation world and objects
- [ ] Controller interface (position, velocity)
- [ ] Sensor models (joint encoders, force sensors)
- [ ] Python simulation API
- [ ] Web interface for simulation control
- [ ] Real-time simulation visualization

**Deliverables**:
- Physics-based simulation
- Realistic robot behavior
- Web-based simulation control

**Testing**:
- Dynamics accuracy (compare with reference implementations)
- Simulation stability (long-running tests)
- Real-time performance (100Hz+ control loop)

**Milestone**: Realistic robot simulation in web interface

---

### Phase 5: Computer Vision (Months 9-10)

**Goal**: Vision integration for robotics

**Tasks**:
- [ ] Camera calibration tools
- [ ] Hand-eye calibration implementation
- [ ] ArUco/AprilTag marker detection
- [ ] Pose estimation (PnP)
- [ ] Point cloud processing (PCL/Open3D)
- [ ] Object detection integration
- [ ] Visual servoing basics (PBVS)
- [ ] Python vision API
- [ ] Web interface for calibration
- [ ] Live camera feed display
- [ ] Detection overlay visualization

**Deliverables**:
- Vision library for robotics
- Calibration tools
- Web-based vision interface

**Testing**:
- Calibration accuracy (reprojection error < 0.5px)
- Pose estimation accuracy (< 5mm, < 5°)
- Detection frame rate (> 30 FPS)

**Milestone**: Vision-guided pick-and-place demo

---

### Phase 6: Hardware Integration (Months 11-12)

**Goal**: Connect to real robots

**Tasks**:
- [ ] Hardware abstraction layer (HAL) design
- [ ] Plugin architecture for drivers
- [ ] Universal Robots driver (RTDE)
- [ ] ABB driver (EGM) [optional]
- [ ] Generic TCP/IP driver template
- [ ] Safety monitoring and limits
- [ ] Real-time control interface
- [ ] State synchronization (sim ↔ real)
- [ ] Web interface for hardware connection
- [ ] Hardware status dashboard
- [ ] Emergency stop integration

**Deliverables**:
- At least one working robot driver (UR)
- Hardware control API
- Web interface for robot control

**Testing**:
- Hardware-in-the-loop tests
- Safety tests (limits, e-stop)
- Latency measurements (< 10ms)

**Milestone**: Control real robot via web interface

---

### Phase 7: Advanced Features (Months 13-15)

**Goal**: Production-ready capabilities

**Tasks**:
- [ ] Advanced planning (CHOMP, TrajOpt)
- [ ] Dual-arm coordination
- [ ] Force control (impedance/admittance)
- [ ] Offline programming (generate robot code)
- [ ] Digital twin (real-time synchronization)
- [ ] Task-level programming interface
- [ ] Grasp planning (basic)
- [ ] Machine learning integration (PyTorch/TF)
- [ ] Multi-user support (authentication)
- [ ] Project save/load functionality
- [ ] Performance optimization

**Deliverables**:
- Advanced planning algorithms
- Force control capabilities
- Offline programming tool

**Testing**:
- Planning benchmarks (success rate, quality)
- Force control accuracy
- System integration tests

**Milestone**: Production-ready for industrial applications

---

### Phase 8: ROS Integration & Ecosystem (Months 16-18)

**Goal**: Integration with robotics ecosystem

**Tasks**:
- [ ] ROS2 message definitions
- [ ] ROS2 bridge for robospace
- [ ] MoveIt integration (use robospace as planning plugin)
- [ ] Gazebo plugin for simulation
- [ ] ros2_control integration
- [ ] Example ROS2 packages
- [ ] Cloud deployment (Docker, Kubernetes)
- [ ] Monitoring and logging (Prometheus, Grafana)
- [ ] Multi-robot support
- [ ] Enterprise features (audit logs, RBAC)
- [ ] Documentation and tutorials
- [ ] Community website

**Deliverables**:
- Full ROS2 integration
- Cloud-deployable system
- Comprehensive documentation

**Testing**:
- ROS integration tests
- Load testing (multiple users, robots)
- Deployment tests (Docker, K8s)

**Milestone**: Full-featured robotics platform with ROS support

---

## Testing Strategy

### C++ Testing

**Framework**: Google Test

**Coverage Goal**: 80%+ line coverage

**Test Types**:
1. **Unit Tests**: Individual classes and functions
2. **Integration Tests**: Module interactions
3. **Benchmark Tests**: Performance regression detection

**Example**:
```cpp
TEST(ForwardKinematicsTest, UR5BasicPose) {
    auto robot = Robot::from_urdf("test_data/ur5.urdf");
    ForwardKinematics fk(robot);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Transform result = fk.compute(q);

    // Expected position for all-zero configuration
    EXPECT_NEAR(result.translation().x(), 0.817, 1e-3);
    EXPECT_NEAR(result.translation().y(), 0.191, 1e-3);
    EXPECT_NEAR(result.translation().z(), 0.005, 1e-3);
}
```

### Python Testing

**Framework**: pytest

**Coverage Goal**: 85%+ line coverage

**Test Types**:
1. **Unit Tests**: Python API functions
2. **Integration Tests**: Full workflows
3. **Doctests**: Examples in docstrings

**Example**:
```python
def test_inverse_kinematics_ur5():
    robot = rs.Robot.from_urdf("test_data/ur5.urdf")
    ik = rs.InverseKinematics(robot)

    target = rs.Transform(translation=[0.4, 0.2, 0.3])
    solution = ik.solve(target)

    assert solution is not None
    assert len(solution) == 6

    # Verify FK matches target
    fk = rs.ForwardKinematics(robot)
    result = fk.compute(solution)
    np.testing.assert_allclose(result.translation, target.translation, atol=1e-3)
```

### Web Testing

**Frontend**:
- **Unit Tests**: Jest (React components)
- **E2E Tests**: Playwright (user workflows)

**Backend**:
- **API Tests**: pytest with FastAPI TestClient
- **Load Tests**: Locust (concurrent users, robots)

**Example E2E Test**:
```typescript
test('plan motion and visualize', async ({ page }) => {
  await page.goto('http://localhost:8000');

  // Load robot
  await page.click('[data-testid="load-robot"]');
  await page.selectOption('[data-testid="robot-select"]', 'ur5e');

  // Set target pose
  await page.fill('[data-testid="target-x"]', '0.4');
  await page.fill('[data-testid="target-y"]', '0.2');
  await page.fill('[data-testid="target-z"]', '0.3');

  // Plan
  await page.click('[data-testid="plan-button"]');
  await expect(page.locator('[data-testid="plan-status"]')).toHaveText('Success');

  // Verify trajectory visualization
  await expect(page.locator('canvas')).toBeVisible();
});
```

### Continuous Integration

**Platform**: GitHub Actions

**Workflow**:
```yaml
name: CI

on: [push, pull_request]

jobs:
  cpp-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Install dependencies
        run: sudo apt-get install -y libeigen3-dev libfcl-dev
      - name: Build
        run: |
          mkdir build && cd build
          cmake .. -DBUILD_TESTS=ON
          make -j$(nproc)
      - name: Test
        run: cd build && ctest --output-on-failure
      - name: Coverage
        run: lcov --capture --directory . --output-file coverage.info

  python-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.10'
      - name: Install package
        run: pip install -e .[dev]
      - name: Test
        run: pytest --cov=robospace --cov-report=xml

  web-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
      - name: Install
        run: cd web/frontend && npm ci
      - name: Test
        run: npm test
      - name: E2E
        run: npx playwright test
```

---

## Deployment & DevOps

### Local Development

**C++ Development**:
```bash
# Build C++ libraries
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON
make -j$(nproc)
ctest
```

**Python Development**:
```bash
# Editable install
pip install -e .[dev]
pytest tests/
```

**Web Development**:
```bash
# Backend
cd backend
pip install -r requirements.txt
uvicorn app.main:app --reload

# Frontend
cd web/frontend
npm install
npm run dev
```

### Docker Deployment

**Dockerfile** (Multi-stage build):
```dockerfile
# Stage 1: Build C++ libraries
FROM ubuntu:22.04 AS cpp-builder
RUN apt-get update && apt-get install -y \
    build-essential cmake libeigen3-dev libfcl-dev libompl-dev
COPY core/ /src/core/
WORKDIR /src/build
RUN cmake ../core && make -j$(nproc)

# Stage 2: Python environment
FROM python:3.10-slim
RUN apt-get update && apt-get install -y libeigen3-dev libfcl-dev
COPY --from=cpp-builder /src/build/lib*.so /usr/local/lib/
COPY python/ /app/python/
COPY backend/ /app/backend/
WORKDIR /app
RUN pip install -e ./python && pip install -r backend/requirements.txt

# Stage 3: Frontend build
FROM node:18 AS frontend-builder
COPY web/frontend /app/frontend
WORKDIR /app/frontend
RUN npm ci && npm run build

# Final stage
FROM python:3.10-slim
COPY --from=python-env /usr/local/lib /usr/local/lib
COPY --from=python-env /app /app
COPY --from=frontend-builder /app/frontend/dist /app/static
WORKDIR /app/backend
ENV PYTHONPATH=/app/python
EXPOSE 8000
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**Docker Compose** (Development):
```yaml
version: '3.8'

services:
  backend:
    build: .
    ports:
      - "8000:8000"
    volumes:
      - ./backend:/app/backend
      - ./python:/app/python
    environment:
      - DATABASE_URL=postgresql://user:pass@db/robospace
    depends_on:
      - db

  db:
    image: postgres:15
    environment:
      POSTGRES_USER: user
      POSTGRES_PASSWORD: pass
      POSTGRES_DB: robospace
    volumes:
      - postgres-data:/var/lib/postgresql/data

  frontend:
    build:
      context: ./web/frontend
    ports:
      - "3000:3000"
    volumes:
      - ./web/frontend:/app
    environment:
      - VITE_API_URL=http://localhost:8000

volumes:
  postgres-data:
```

### Cloud Deployment (Kubernetes)

**Deployment Manifest**:
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: robospace
spec:
  replicas: 3
  selector:
    matchLabels:
      app: robospace
  template:
    metadata:
      labels:
        app: robospace
    spec:
      containers:
      - name: robospace
        image: robospace/robospace:latest
        ports:
        - containerPort: 8000
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-secret
              key: url
        resources:
          requests:
            memory: "1Gi"
            cpu: "500m"
          limits:
            memory: "2Gi"
            cpu: "2000m"
```

---

## Performance Considerations

### C++ Optimization

**Compiler Flags**:
```cmake
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -fsanitize=address")
```

**Performance Targets**:
| Operation | Target Latency | Notes |
|-----------|----------------|-------|
| Forward Kinematics | < 10 μs | 6-DOF robot |
| Jacobian Calculation | < 50 μs | Geometric Jacobian |
| Inverse Kinematics (numerical) | < 1 ms | Single solution |
| Collision Check (single pair) | < 100 μs | Convex meshes |
| Motion Planning | < 5 s | Simple scenes |
| Dynamics (RNEA) | < 20 μs | 6-DOF robot |

**Optimization Strategies**:
- **Eigen Optimization**: Enable vectorization, aligned allocations
- **Caching**: Pre-compute constant transformations
- **Parallel Planning**: Multi-threaded planning for exploration
- **Lazy Evaluation**: Compute only when needed

### Python Performance

**Minimize Python/C++ Boundary Crossings**:
- Batch operations (compute FK for multiple configurations)
- Return NumPy arrays, not lists
- Use NumPy views instead of copies

**Example**:
```python
# Bad: Many Python/C++ calls
results = [fk.compute(q) for q in joint_configs]

# Good: Single call with batch
results = fk.compute_batch(joint_configs)  # Returns (N, 4, 4) array
```

### Web Performance

**Frontend**:
- WebGL optimization (instancing, LOD)
- Lazy loading for robot meshes
- Web Workers for heavy computation
- Virtual scrolling for large lists

**Backend**:
- Async I/O (FastAPI + asyncio)
- Connection pooling (database)
- Caching (Redis for frequent queries)
- Load balancing (multiple workers)

**Network**:
- WebSocket for real-time (lower overhead than REST)
- Binary formats for large data (MessagePack, Protocol Buffers)
- Compression (gzip)

---

## Security & Safety

### Robot Safety

**Software Safety**:
- **Joint Limits**: Enforce position, velocity, acceleration limits
- **Singularity Avoidance**: Detect and avoid kinematic singularities
- **Collision Detection**: Real-time collision checking
- **Workspace Limits**: Define safe operation zones
- **E-Stop Integration**: Software emergency stop

**Example**:
```cpp
class SafetyMonitor {
public:
    bool check_joint_limits(const Eigen::VectorXd& q);
    bool check_velocity_limits(const Eigen::VectorXd& qd);
    bool check_workspace(const Transform& pose);
    bool check_collisions(const Robot& robot, const Scene& scene);
    void emergency_stop();
};
```

### Web Security

**Authentication & Authorization**:
- JWT tokens for API access
- Role-based access control (RBAC)
- Multi-user support with isolated sessions

**Input Validation**:
- Pydantic models for all API inputs
- Range checks for joint positions, velocities
- Sanitize file uploads (URDF, meshes)

**Network Security**:
- HTTPS/WSS in production
- CORS configuration
- Rate limiting (prevent DoS)

**Example**:
```python
from pydantic import BaseModel, validator

class JointState(BaseModel):
    positions: list[float]

    @validator('positions')
    def check_range(cls, v):
        if len(v) != 6:
            raise ValueError('Must have 6 joints')
        if any(abs(x) > 2*pi for x in v):
            raise ValueError('Joint position out of reasonable range')
        return v
```

---

## Future Roadmap

### Phase 9: Mobile Robots (Months 19-21)

- Differential drive, Ackermann steering models
- SLAM integration (GMapping, Cartographer)
- Navigation stack (path planning, obstacle avoidance)
- Multi-robot coordination

### Phase 10: Advanced AI/ML (Months 22-24)

- Reinforcement learning for control (Stable Baselines3)
- Imitation learning (behavior cloning)
- Learning-based grasping (GraspNet, DexNet)
- Sim-to-real transfer

### Phase 11: Advanced Simulation (Months 25-27)

- Deformable object simulation
- Fluid simulation (for liquids)
- Contact-rich manipulation
- Parallel simulation (Isaac Gym style)

### Phase 12: Commercial Features (Months 28-30)

- Fleet management (multiple robots)
- Production monitoring and analytics
- Predictive maintenance
- OPC UA integration (industrial automation)

---

## Appendix

### A. Dependencies and Licenses

| Library | License | Purpose |
|---------|---------|---------|
| Eigen | MPL 2.0 | Linear algebra |
| FCL | BSD | Collision detection |
| OMPL | BSD | Motion planning |
| Bullet | Zlib | Physics simulation |
| OpenCV | Apache 2.0 | Computer vision |
| pybind11 | BSD | Python bindings |
| FastAPI | MIT | Web framework |
| React | MIT | Frontend framework |
| Three.js | MIT | 3D visualization |

**Robospace License**: MIT (tentative, allows commercial use)

### B. Coordinate Frames and Conventions

**Coordinate Systems**:
- **World Frame**: Fixed, right-handed (X: forward, Y: left, Z: up)
- **Robot Base Frame**: Defined in URDF
- **End-Effector Frame**: Tool center point (TCP)
- **Camera Frame**: OpenCV convention (Z: forward, Y: down, X: right)

**Rotation Representations**:
- **Rotation Matrix**: SO(3), 3x3 orthogonal matrix
- **Quaternion**: [w, x, y, z], unit quaternion (preferred for interpolation)
- **Euler Angles**: XYZ intrinsic (roll-pitch-yaw)
- **Axis-Angle**: Rotation vector [θ kx, θ ky, θ kz]

**Units**:
- **Length**: meters (m)
- **Angle**: radians (rad)
- **Time**: seconds (s)
- **Mass**: kilograms (kg)
- **Force**: newtons (N)

### C. Naming Conventions

**C++**:
- Namespaces: `robospace::module_name`
- Classes: `PascalCase`
- Functions: `snake_case`
- Variables: `snake_case`
- Constants: `UPPER_CASE` or `kConstantName`
- Member variables: `member_` (trailing underscore)

**Python**:
- Modules: `snake_case`
- Classes: `PascalCase`
- Functions: `snake_case`
- Variables: `snake_case`
- Constants: `UPPER_CASE`

**Web**:
- Components: `PascalCase` (React)
- Files: `PascalCase.tsx` for components, `snake_case.ts` for utilities
- CSS classes: `kebab-case`

### D. References

**Robotics**:
1. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*
2. Siciliano, B., et al. (2009). *Robotics: Modelling, Planning and Control*
3. Lynch, K. M., & Park, F. C. (2017). *Modern Robotics*

**Computer Vision**:
1. Hartley, R., & Zisserman, A. (2004). *Multiple View Geometry*
2. Szeliski, R. (2022). *Computer Vision: Algorithms and Applications*

**Motion Planning**:
1. LaValle, S. M. (2006). *Planning Algorithms*
2. Choset, H., et al. (2005). *Principles of Robot Motion*

**Software**:
1. Meyers, S. (2014). *Effective Modern C++*
2. Gamma, E., et al. (1994). *Design Patterns*

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-11-08 | Initial | First draft of design document |

---

**Document Status**: ✅ Ready for Review

**Next Steps**:
1. Review and approve design document
2. Create GitHub project board with tasks
3. Begin Phase 1 implementation
4. Schedule weekly progress reviews

