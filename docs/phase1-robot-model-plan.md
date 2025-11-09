# Phase 1: Robot Model - Updated Implementation Plan

**Goal:** Implement robot modeling with KinematicTree + Frames, supporting dual DH conventions

**Based on:** Research from Modern Robotics, Peter Corke, ROS/KDL, Pinocchio, RoboDK

---

## Design Decisions Summary

1. ✅ **KinematicTree + Frames** (not just KinematicChain)
   - Tree = Link-Joint topology
   - Frames = Additional coordinate systems (TCP, sensors, calibration targets)

2. ✅ **Both Standard and Modified DH**
   - Standard DH: Universal Robots, Yaskawa
   - Modified DH: ABB, Fanuc, Stäubli
   - Cannot auto-convert, must support both

3. ✅ **6-DOF Error Modeling**
   - Add error frames to each link for calibration
   - Optimize full SE(3) (36 params) vs 4 DH params (24 params)

4. ✅ **Extensibility**
   - Support robot on rail (add prismatic virtual joint)
   - Support mobile base (add floating 6-DOF virtual joint)
   - Support multi-sensor setups (add frames dynamically)

5. ⏸️ **Defer Full Item Pattern to Phase 2+**
   - Keep Phase 1 focused on single robot
   - Add multi-robot, workpieces, full station later

---

## Implementation Order (Step-by-Step)

### Step 1: Core Data Structures (Week 1, Days 1-2)
**Goal:** Define basic building blocks

**Files to create:**
- `include/robospace/model/joint.hpp`
- `include/robospace/model/link.hpp`
- `include/robospace/model/dh_params.hpp`
- `src/model/joint.cpp`
- `src/model/link.cpp`
- `src/model/dh_params.cpp`

**Tasks:**
- [ ] Create `JointType` enum (REVOLUTE, PRISMATIC, CONTINUOUS, FIXED)
- [ ] Create `DHConvention` enum (STANDARD, MODIFIED)
- [ ] Implement `DHParams` struct with both conventions
- [ ] Implement `Joint` class
  - Name, type, parent/child link IDs
  - DH parameters, screw axis, URDF origin+axis (triple representation)
  - Limits (position, velocity, effort)
- [ ] Implement `Link` class
  - Name, physical properties (mass, CoM, inertia)
  - Geometry placeholders (visual, collision mesh paths)
- [ ] Write tests for Joint and Link construction

**Deliverable:** Can create Joint and Link objects
```cpp
Joint j1("shoulder_pan", JointType::REVOLUTE, 0, 1);
j1.set_dh_params(DHParams(0, 0, 0.089159, 0, DHConvention::MODIFIED));
j1.set_limits(-M_PI, M_PI);

Link l1("shoulder_link");
l1.set_mass(3.7);
```

---

### Step 2: Frame Class (Week 1, Days 3-4)
**Goal:** First-class Frame for coordinate systems

**Files to create:**
- `include/robospace/model/frame.hpp`
- `src/model/frame.cpp`

**Tasks:**
- [ ] Implement `Frame` class
  - Name, parent link ID, offset (SE3)
  - Update offset (for calibration)
  - Query methods
- [ ] Write tests for Frame
  - Construction
  - Offset updates
  - Relationship to parent link

**Deliverable:** Can create frames on links
```cpp
Frame tcp("tcp", flange_link_id, SE3::Translation(0, 0, 0.1));
Frame camera("camera", flange_link_id, SE3::RotY(M_PI/2) * SE3::Translation(0.05, 0, 0));
```

---

### Step 3: KinematicTree (Week 1, Day 5 - Week 2, Day 1)
**Goal:** Link-Joint topology management

**Files to create:**
- `include/robospace/model/kinematic_tree.hpp`
- `src/model/kinematic_tree.cpp`

**Tasks:**
- [ ] Implement `KinematicTree` class
  - Store vectors of Links and Joints
  - Parent-child relationship arrays
  - Name-to-ID lookup maps
  - `add_link()`, `add_joint()` methods
  - Tree traversal (get_joint_chain, parent/child queries)
  - Root link identification
  - DOF calculation (count non-fixed joints)
- [ ] Write tests
  - Simple 2-link chain
  - Branching tree (multi-arm)
  - Chain extraction
  - Topology queries

**Deliverable:** Can build and query kinematic trees
```cpp
KinematicTree tree;
int base_id = tree.add_link(Link("base"));
int link1_id = tree.add_link(Link("link1"));
int joint1_id = tree.add_joint(Joint("joint1", JointType::REVOLUTE, base_id, link1_id));

auto chain = tree.get_joint_chain(base_id, link1_id);  // Returns [joint1_id]
```

---

### Step 4: Robot Class Skeleton (Week 2, Days 2-3)
**Goal:** High-level API without kinematics yet

**Files to create:**
- `include/robospace/model/robot.hpp`
- `src/model/robot.cpp`

**Tasks:**
- [ ] Implement `Robot` class
  - Constructor with name
  - `KinematicTree tree_` member
  - `std::vector<Frame> frames_` member
  - Name-to-ID lookup for links, joints, frames
  - `add_frame()` method
  - Accessors: `link()`, `joint()`, `frame()` by name/ID
  - Counts: `num_links()`, `num_joints()`, `num_frames()`, `num_positions()`
  - Special frame IDs: base_frame_id_, tcp_frame_id_
  - Frame management: `link_id()`, `joint_id()`, `frame_id()`
- [ ] Write tests
  - Robot construction
  - Adding frames
  - Name lookup
  - DOF counting

**Deliverable:** Can create robot with frames (no kinematics yet)
```cpp
Robot robot("ur5e");
// ... build tree manually for now (URDF parser comes later)

int tcp_id = robot.add_frame("tcp", "tool0", SE3::Translation(0, 0, 0.1));
int camera_id = robot.add_frame("camera", "tool0", SE3::RotY(M_PI/2));

const Frame& tcp = robot.frame("tcp");
```

---

### Step 5: URDF Parser (Week 2, Days 4-5 - Week 3, Days 1-2)
**Goal:** Load robots from URDF files

**Files to create:**
- `include/robospace/model/urdf_parser.hpp`
- `src/model/urdf_parser.cpp`

**Dependencies:**
- Add TinyXML2 to CMakeLists.txt

**Tasks:**
- [ ] Implement `URDFParser` class
  - Parse `<robot>` tag (name)
  - Parse `<link>` tags (name, visual, collision, inertial)
  - Parse `<joint>` tags (name, type, parent, child, origin, axis, limits)
  - Build `KinematicTree` from parsed data
  - Error handling (missing links, invalid topology)
  - Convert URDF origin+axis to screw axis representation
  - Optionally compute DH parameters (if possible)
- [ ] Add `Robot::from_urdf()` static factory method
- [ ] Write tests
  - Parse simple 2-DOF URDF
  - Parse UR5 URDF
  - Parse Panda URDF
  - Error cases (invalid URDF, missing files)

**Test URDF files needed:**
- `test_data/simple_2r.urdf` (2-link RR robot)
- `test_data/ur5.urdf` (download from Universal Robots)
- `test_data/panda.urdf` (download from Franka Emika)

**Deliverable:** Can load real robots from URDF
```cpp
Robot robot = Robot::from_urdf("models/ur5.urdf");
std::cout << "Loaded " << robot.name() << " with "
          << robot.num_positions() << " DOF" << std::endl;
```

---

### Step 6: Forward Kinematics (Week 3, Days 3-5)
**Goal:** Compute end-effector pose from joint angles

**Files to create:**
- `include/robospace/kinematics/forward_kinematics.hpp`
- `src/kinematics/forward_kinematics.cpp`
- `core/kinematics/CMakeLists.txt` (new directory)

**Tasks:**
- [ ] Create kinematics directory structure
- [ ] Implement `ForwardKinematics` base class (interface)
- [ ] Implement `DHForwardKinematics` class
  - Compute FK using DH parameters (both conventions)
  - Support Standard DH: Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
  - Support Modified DH: Rot_x(α) * Trans_x(a) * Trans_z(d) * Rot_z(θ)
  - Chain DH transforms from base to tip
- [ ] Implement FK to arbitrary frames
  - Compute transform to link, then apply frame offset
  - `compute_link_pose(q, link_id)`
  - `compute_frame_pose(q, frame_id)`
- [ ] Add FK methods to Robot class
  - `Robot::fkine(q, frame_name)` - delegate to FK solver
  - `Robot::get_frame_pose(frame_id, q)` - absolute pose
  - Lazy initialization of FK solver
- [ ] Write tests
  - 2-DOF test (known analytical solution)
  - UR5 FK (compare to manufacturer data)
  - FK to different frames (tcp, camera)
  - Performance benchmark (target: < 10 μs)

**Deliverable:** Working forward kinematics
```cpp
Robot robot = Robot::from_urdf("ur5.urdf");
Eigen::VectorXd q(6);
q << 0, -M_PI/2, M_PI/2, 0, M_PI/2, 0;

SE3 tcp_pose = robot.fkine(q, "tcp");
std::cout << "TCP position: " << tcp_pose.translation().transpose() << std::endl;

// FK to camera frame (if added)
robot.add_frame("camera", "tool0", SE3::RotY(M_PI/2));
SE3 camera_pose = robot.fkine(q, "camera");
```

---

### Step 7: DH Parameter Factory (Week 4, Days 1-2)
**Goal:** Create robots from DH parameters directly

**Tasks:**
- [ ] Implement `Robot::from_dh()` static factory
  - Takes vector of DHParams
  - Creates links and joints automatically
  - Assigns default names (link0, link1, joint0, joint1, ...)
  - Sets up base and TCP frames
- [ ] Write tests
  - Create UR5 from DH parameters
  - Create Panda from DH parameters
  - Verify FK matches URDF-loaded robot

**Deliverable:** Can create robots without URDF
```cpp
std::vector<DHParams> ur5_dh = {
    {M_PI/2, 0,      0.089159, 0, DHConvention::MODIFIED},
    {0,      -0.425, 0,        0, DHConvention::MODIFIED},
    {0,      -0.392, 0,        0, DHConvention::MODIFIED},
    {M_PI/2, 0,      0.10915,  0, DHConvention::MODIFIED},
    {-M_PI/2, 0,     0.09465,  0, DHConvention::MODIFIED},
    {0,      0,      0.0823,   0, DHConvention::MODIFIED}
};

Robot robot = Robot::from_dh("ur5e", ur5_dh);
```

---

### Step 8: CMake Integration & Build (Week 4, Day 3)
**Goal:** Everything compiles and links

**Tasks:**
- [ ] Update root `CMakeLists.txt`
  - Add model subdirectory
  - Add kinematics subdirectory
- [ ] Create `core/model/CMakeLists.txt`
  - List all model sources
  - Link Eigen, TinyXML2
- [ ] Create `core/kinematics/CMakeLists.txt`
  - List kinematics sources
  - Link to model library
- [ ] Update `core/CMakeLists.txt`
  - Combine math, model, kinematics into `librobospace-core`
- [ ] Update `tests/CMakeLists.txt`
  - Add model tests
  - Add kinematics tests
  - Link test executables

**Deliverable:** `make build && make test-cpp` passes with all tests

---

### Step 9: Comprehensive Testing (Week 4, Days 4-5)
**Goal:** Validate all components work together

**Tasks:**
- [ ] Integration tests
  - Load UR5 from URDF, compute FK, verify against known data
  - Load Panda from URDF, compute FK
  - Create robot from DH, verify FK
  - Add custom frames, compute FK to those frames
- [ ] Edge case tests
  - Invalid URDF (missing links)
  - Out-of-limit joint angles
  - Unknown frame names
- [ ] Performance benchmarks
  - FK < 10 μs (target)
  - URDF parsing < 100 ms
- [ ] Test with both DH conventions
  - Standard DH robot
  - Modified DH robot
  - Verify transforms are correct

**Deliverable:** 80%+ test coverage, all tests passing

---

### Step 10: Documentation (Week 4, Day 5 - Week 5, Day 1)
**Goal:** Document the robot model API

**Tasks:**
- [ ] Add Doxygen comments to all public APIs
  - Joint, Link, Frame, KinematicTree, Robot classes
  - ForwardKinematics classes
  - DHParams struct
- [ ] Write usage examples in comments
- [ ] Create `docs/robot-model-guide.md`
  - Explain design (KinematicTree + Frames)
  - Explain DH conventions
  - Show examples (URDF, DH, frames)
  - Explain extensibility (rail, mobile base, calibration)
- [ ] Update README.md with robot model examples

**Deliverable:** Clear documentation for robot modeling

---

## Updated CMake Structure

```
core/
├── CMakeLists.txt          # Top-level core build
├── math/                   # ✅ Already done
│   └── CMakeLists.txt
├── model/                  # 🚧 This phase
│   ├── CMakeLists.txt
│   ├── joint.cpp
│   ├── link.cpp
│   ├── frame.cpp
│   ├── dh_params.cpp
│   ├── kinematic_tree.cpp
│   ├── robot.cpp
│   └── urdf_parser.cpp
└── kinematics/             # 🚧 This phase
    ├── CMakeLists.txt
    ├── forward_kinematics.cpp
    └── dh_forward_kinematics.cpp

include/robospace/
├── math/                   # ✅ Already done
├── model/                  # 🚧 This phase
│   ├── joint.hpp
│   ├── link.hpp
│   ├── frame.hpp
│   ├── dh_params.hpp
│   ├── kinematic_tree.hpp
│   ├── robot.hpp
│   └── urdf_parser.hpp
└── kinematics/             # 🚧 This phase
    ├── forward_kinematics.hpp
    └── dh_forward_kinematics.hpp

tests/cpp/
├── test_joint.cpp          # 🚧 This phase
├── test_link.cpp
├── test_frame.cpp
├── test_kinematic_tree.cpp
├── test_robot.cpp
├── test_urdf_parser.cpp
└── test_forward_kinematics.cpp

test_data/                  # 🚧 This phase
├── simple_2r.urdf
├── ur5.urdf
└── panda.urdf
```

---

## Testing Strategy

### Unit Tests (Per-Class)
- Joint: construction, limits, DH params, screw axis
- Link: construction, physical properties
- Frame: construction, offset updates
- KinematicTree: add links/joints, topology, chain extraction
- Robot: construction, frame management, lookups
- URDFParser: parse valid/invalid URDF
- ForwardKinematics: 2-DOF, 6-DOF, both DH conventions

### Integration Tests
- Load URDF → compute FK → verify against known poses
- Create from DH → compute FK → compare to URDF robot
- Add frames → compute FK to frames

### Performance Benchmarks
- FK computation: target < 10 μs
- URDF parsing: target < 100 ms

---

## Success Criteria (This Phase)

✅ Can create Joint, Link, Frame objects
✅ Can build KinematicTree programmatically
✅ Can load robots from URDF files
✅ Can create robots from DH parameters (both conventions)
✅ Can add custom frames (TCP, sensors) dynamically
✅ Forward kinematics works correctly
✅ FK to arbitrary frames works
✅ All tests passing (80%+ coverage)
✅ Performance targets met (FK < 10 μs)
✅ Documentation complete

---

## Deferred to Later Phases

⏸️ **Inverse Kinematics** → Phase 2
⏸️ **Jacobian Computation** → Phase 2
⏸️ **Full Item Pattern** (multi-robot, workpieces) → Phase 3
⏸️ **Python Bindings** → After C++ API stable
⏸️ **Collision Detection** → Phase 3
⏸️ **Dynamics** → Phase 4

---

## Key Advantages of This Design

1. **Metrology-Ready** ⭐
   - Add calibration target frames to each link
   - Optimize 6-DOF error frames with laser tracker data
   - Laser tracker can be world reference frame

2. **Industrial Compatibility** ⭐
   - Support both DH conventions (ABB/Fanuc vs UR/Yaskawa)
   - TCP frames (not just flange)
   - Easy to add workpiece frames later

3. **Extensibility** ⭐
   - Robot on rail: add prismatic virtual joint
   - Mobile base: add 6-DOF floating joint
   - Multi-sensor: add frames dynamically

4. **Calibration Flexibility** ⭐
   - 6-DOF error modeling (36 params vs 24 DH params)
   - Update frame offsets after calibration
   - Error frames preserve kinematic structure

---

## Timeline Estimate

**Total: 4-5 weeks**

- Week 1: Steps 1-3 (Joint, Link, Frame, KinematicTree)
- Week 2: Steps 4-5 (Robot class, URDF parser)
- Week 3: Steps 6 (Forward kinematics)
- Week 4: Steps 7-9 (DH factory, CMake, testing)
- Week 5: Step 10 (Documentation, polish)

**Milestone:** Ready for Phase 2 (Inverse Kinematics)
