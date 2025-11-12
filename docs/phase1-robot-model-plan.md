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

### Step 1: Core Data Structures (Week 1, Days 1-2) ✅ DONE
**Goal:** Define basic building blocks

**Files created:**
- `include/robospace/model/joint.hpp`
- `include/robospace/model/link.hpp`
- `include/robospace/model/dh_params.hpp`
- `src/model/joint.cpp`
- `src/model/link.cpp`
- `src/model/dh_params.cpp`

**Completed:**
- ✅ Create `JointType` enum (REVOLUTE, PRISMATIC, CONTINUOUS, FIXED)
- ✅ Create `DHConvention` enum (STANDARD, MODIFIED)
- ✅ Implement `DHParams` struct with both conventions + offset
- ✅ Implement `Joint` class
  - Triple kinematic representation (DH, URDF origin+axis, screw)
  - Limits (position, velocity, effort)
  - Industrial robot features: axis_direction, coupling_terms
- ✅ Implement `Link` class (inherits Entity)
  - Physical properties (mass, CoM, inertia)
  - Geometry (visual/collision mesh paths)
- ✅ 33 tests passing (basic + industrial robot features)

**Deliverable:** Can create Joint and Link objects
```cpp
Joint j1("shoulder_pan", JointType::REVOLUTE, 0, 1);
j1.set_dh_params(DHParams(0, 0, 0.089159, 0, DHConvention::MODIFIED));
j1.set_limits(-M_PI, M_PI);

Link l1("shoulder_link");
l1.set_mass(3.7);
```

---

### Step 2: Entity Refactor + Frame (Week 1, Days 3-4) ✅ DONE
**Goal:** Flexible scene graph with first-class Frames

**Files created:**
- `include/robospace/model/entity.hpp` (NEW - base class)
- `src/model/entity.cpp`
- `include/robospace/model/frame.hpp`
- `src/model/frame.cpp`

**Completed:**
- ✅ Implement `Entity` base class (abstract)
  - Scene graph management (parent, children, pose)
  - `pose_world()` - recursive world pose computation
  - Type system (FRAME, LINK, ROBOT, TOOL, TARGET)
- ✅ Refactor `Link` to inherit from Entity
- ✅ Implement `Frame` inherits Entity
  - Flexible scene graph (Frame→Frame, Link→Frame supported)
  - `update_pose()` for calibration updates
- ✅ Add `compute_transform(source, target)` utility
  - Compute transform between any entities in graph
- ✅ 28 tests (construction, scene graph, world pose, use cases)

**Deliverable:** Can create frames on links
```cpp
Frame tcp("tcp", flange_link_id, SE3::Translation(0, 0, 0.1));
Frame camera("camera", flange_link_id, SE3::RotY(M_PI/2) * SE3::Translation(0.05, 0, 0));
```

---

### Step 3: KinematicTree (Week 1, Day 5 - Week 2, Day 1) ✅ DONE
**Goal:** Serial chain kinematics for industrial robots

**Files created:**
- `include/robospace/model/kinematic_tree.hpp`
- `src/model/kinematic_tree.cpp`

**Completed:**
- ✅ Implement `KinematicTree` class for serial chains
  - Store vectors of Links and Joints
  - `add_link()`, `add_joint()` methods
  - `set_configuration(q)` for joint angles
  - `compute_forward_kinematics()` - O(n) chain traversal
  - `link_pose(id)` - get link pose in base frame
  - Industrial robot support (axis_direction, J2-J3 coupling)
- ✅ Write 15 comprehensive tests
  - Construction & validation (N+1 links for N joints)
  - Configuration management
  - FK for serial chains (zero config, elbow bend, both joints)
  - Industrial robots (inverted axes, Fanuc-style coupling)
  - Error handling (invalid trees, missing config)

**Current Limitation:**
⚠️ **Serial chains only** - no branching topology support
- Supports: Base--J1--L1--J2--L2--J3--L3 (6-axis arms)
- NOT supported: Multi-branch trees (dual-arm, humanoids)

**Deliverable:** Can build and query kinematic trees
```cpp
KinematicTree tree;
int base_id = tree.add_link(Link("base"));
int link1_id = tree.add_link(Link("link1"));
int joint1_id = tree.add_joint(Joint("joint1", JointType::REVOLUTE, base_id, link1_id));

auto chain = tree.get_joint_chain(base_id, link1_id);  // Returns [joint1_id]
```

---

### Step 4: Robot Class Skeleton (Week 2, Days 2-3) ✅ DONE
**Goal:** High-level API with tool management and frame hierarchy

**Files created:**
- `include/robospace/model/robot.hpp`
- `include/robospace/model/tool.hpp`
- `src/model/robot.cpp`
- `src/model/tool.cpp`

**Completed:**
- ✅ Implement `Robot` class (inherits Entity)
  - Constructor with name and optional parent
  - `KinematicTree tree_` member
  - `std::vector<Tool> tools_` member
  - Name-to-ID lookup for links, joints, tools
  - `add_link()`, `add_joint()`, `add_tool()` methods
  - Accessors: `link()`, `joint()`, `tool()` by name/ID (const & non-const)
  - Counts: `num_links()`, `num_joints()`, `num_tools()`, `num_positions()`
  - Special accessors: `base_link()`, `flange_link()`, `base_frame()`
  - Tool management: `active_tool()`, `set_active_tool()`
  - Validation: `is_valid()` delegate to kinematic tree
- ✅ Implement `Tool` class (inherits Frame)
  - TCP pose relative to flange
  - Simple wrapper around Frame for semantics
- ✅ Enhanced Entity with virtual `pose()` and `set_pose()`
  - Robot overrides to return TCP pose (via FK in future)
- ✅ Write 41 comprehensive tests
  - Construction & building
  - Link/joint/tool accessors (ID and name)
  - Tool management (add, active tool, parent assignment)
  - Base frame & flange frame
  - Validation & error handling
- ✅ 240 tests passing total

**Deliverable:** Can create robot with tools
```cpp
Robot robot("ur5e");
robot.add_link(Link("base"));
// ... add more links

Tool gripper("gripper", SE3::Translation(0, 0, 0.15));
robot.add_tool(gripper);
robot.set_active_tool("gripper");

const Tool& tcp = robot.active_tool();
```

---

### Step 5: URDF Parser (Week 2, Days 4-5 - Week 3, Days 1-2) ✅ DONE
**Goal:** Load robots from URDF files

**Files created:**
- `include/robospace/model/urdf_parser.hpp`
- `src/model/urdf_parser.cpp`
- `test_data/simple_2r.urdf` (2-DOF test robot)
- `test_data/ur5_simplified.urdf` (6-DOF industrial robot)
- `tests/cpp/test_urdf_parser.cpp`

**Dependencies:**
- ✅ Added TinyXML2 to CMakeLists.txt via FetchContent

**Completed:**
- ✅ Implement `URDFParser` class
  - Parse `<robot>` tag (name)
  - Parse `<link>` tags (name, visual, collision, inertial)
  - Parse `<joint>` tags (name, type, parent, child, origin, axis, limits)
  - Build `KinematicTree` from parsed data (2-pass: links first, then joints)
  - Error handling (missing links, invalid topology, file not found)
  - Convert URDF origin+axis to SE3 representation
  - Support RPY to rotation matrix (ZYX Euler convention)
  - Parse inertial properties (mass, CoM, inertia tensor)
  - Parse joint limits (position, velocity, effort)
  - Static methods: `parse_file()`, `parse_string()`
- ✅ Add `Robot::from_urdf()` and `Robot::from_urdf_string()` factory methods
- ✅ Write 7 comprehensive tests
  - Load simple 2R robot (3 links, 2 joints)
  - Load UR5 robot (7 links, 6 joints)
  - Link properties (mass, CoM, inertia)
  - Joint properties (type, parent/child, axes, limits, origins)
  - Parse from string
  - Error handling (file not found, invalid XML, missing elements)
  - Origin with RPY rotations
- ✅ 247 tests passing total

**Note:** DH parameter extraction deferred - URDF uses origin+axis representation which is more general than DH. DH can be computed separately if needed for specific algorithms.

**Deliverable:** Load real robots with one line!
```cpp
// Load robot from URDF file
Robot ur5 = Robot::from_urdf("models/ur5.urdf");

// Query robot properties
std::cout << "Robot: " << ur5.name() << std::endl;
std::cout << "DOF: " << ur5.num_positions() << std::endl;
std::cout << "Joints: " << ur5.num_joints() << std::endl;

// Access robot components
const Joint& shoulder = ur5.joint("shoulder_pan_joint");
const Link& base = ur5.link("base_link");
```

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

## Future Extensions (Beyond Phase 1)

### Branching Kinematic Trees (Phase 2+)

**Current Limitation:**
- `KinematicTree` supports **serial chains only** (99% of industrial robots)
- No multi-branch topology (dual-arm, humanoid, parallel robots)

**Why Deferred:**
1. 99% of target use cases are serial manipulators (Fanuc, KUKA, ABB, UR)
2. Significant complexity: graph data structure, multi-end-effector FK, complex Jacobian
3. Workaround exists: model dual-arm as two separate `Robot` instances

**Future Implementation Path:**
- Create `TreeKinematics` class alongside `KinematicTree`
- Use graph data structure (adjacency list) instead of vectors
- FK requires specifying target branch/end-effector
- Examples: humanoids (2 arms + 2 legs), dual-arm workstations, mobile manipulators

**Inspiration:**
- Robotics Library (RL): `rl::mdl::Model` with graph support
- Pinocchio: Full tree kinematics for humanoids
- ROS KDL: Tree structure with segment chains

**Design for Extensibility:**
- Current `KinematicTree` is **internal to Robot class**
- User API won't change when adding `TreeKinematics`
- `Robot` can switch between `KinematicTree` (serial) and `TreeKinematics` (branching) internally

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
