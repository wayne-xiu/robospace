# Robot API Analysis and Redesign Proposal

## Reference API Analysis

### RoboDK API (Industrial Robotics Focus)
**Organization**: Simple, action-oriented
```cpp
// Joint Space
Joints() / setJoints()
MoveJ()
JointLimits() / setJointLimits()

// Cartesian Space
Pose() / setPose()
MoveL() / MoveC()
PoseTool() / setPoseTool()

// Kinematics
SolveFK(joints)
SolveIK(pose)
```

**Key Patterns**:
- Consistent get/set naming (Joints/setJoints, Pose/setPose)
- Clear separation of joint space vs Cartesian space
- Tool management integrated (PoseTool)
- Simple method names (FK, IK, not compute_forward_kinematics)

### Peter Corke Robotics Toolbox (Academic/Research Focus)
**Organization**: Comprehensive, analysis-focused
```python
# Kinematics
fkine(q)  # Returns SE3
fkine_all(q)  # All link poses
jacob0(q)  # Jacobian in base frame
jacobe(q)  # Jacobian in EE frame

# Dynamics
rne(q, qd, qdd)  # Inverse dynamics
inertia(q)  # Inertia matrix

# Analysis
manipulability(q)
reach  # Property
iscollided()

# Construction
Robot.URDF(file)
```

**Key Patterns**:
- Short method names (fkine, not compute_fk)
- Stateless methods (pass q explicitly)
- Properties for static info (reach, name)
- Separate dynamics and analysis methods

### Robotics Library (RL) (Real-time Control Focus)
**Organization**: State-based, control-oriented
```cpp
// State Management
setPosition(q) / getPosition()
getVelocity() / getAcceleration() / getTorque()

// Kinematics
forwardPosition()  # Computes FK using stored state
getOperationalPosition(i)  # Get result for operational point i
```

**Key Patterns**:
- Explicit get/set for all state
- Compute then query pattern (forwardPosition(), then getOperationalPosition())
- Supports velocity, acceleration, torque state

## Current Robot Class Issues

### 1. Inconsistent Naming
- `pose()` vs `get_tcp_pose()` vs `get_current_tcp_pose()` - THREE methods for TCP pose
- `joints()` vs `joint(id)` - CONFUSING! Is `joints()` plural of `joint()`?
- `compute_fk()` vs `get_tcp_pose()` - both do FK but named differently

### 2. Poor Organization
Methods appear in random order:
```cpp
Line 18: Constructor
Line 21: Factory method
Line 24: add_link
Line 27: add_tool
Line 36: pose()  (FK)
Line 40: link(id)
Line 50: link_id()
Line 59: set_base_frame
Line 80: joints()  (joint config)
Line 87: compute_fk()  (FK again)
```
No clear grouping by functionality.

### 3. Too Many Redundant Methods
- `link(int)`, `link(string)`, `link(int) const`, `link(string) const` - 4 overloads
- `joint(int)`, `joint(string)`, `joint(int) const`, `joint(string) const` - 4 overloads
- `tool(int)`, `tool(string)`, `tool(int) const`, `tool(string) const` - 4 overloads
- **Total: 12 accessor methods** for links/joints/tools

- `compute_fk(q, link)`, `get_tcp_pose(q)`, `get_current_tcp_pose()`, `compute_all_link_poses(q)` - 4 FK methods
- Plus `pose()` override from Entity - makes **5 FK methods total**

### 4. Missing Important Methods
- **No Jacobian methods** (jacob0, jacobe) - needed for velocity kinematics, IK
- **No joint limits access** at Robot level (have to dig into tree)
- **No velocity/acceleration state** (only position via joints())
- **No q_zero, q_ready** standard configurations
- **No fkine shorthand** (only verbose compute_fk with link name)

### 5. Confusing API
- `pose()` requires calling `set_joints()` first, throws if not set - unintuitive
- `kinematic_tree()` exposed - exposes internal implementation detail
- `move_base_by()` - what does "move base by" mean? Not clear
- `set_pose()` throws "not implemented" - why have it then?
- `base_link()` / `flange_link()` - convenience methods but inconsistent (why not `tcp_link()`?)

### 6. No Clear Mental Model
User doesn't know:
- Where to start? Construction? Configuration?
- How to do FK? `pose()`, `compute_fk()`, `get_tcp_pose()`, `get_current_tcp_pose()`?
- Joint space vs Cartesian space operations?
- Stateful (uses stored q) vs stateless (pass q) methods?

## Proposed Robot API Redesign

### Design Principles
1. **Consistent naming**: Use standard names from literature (fk, ik, jacob0, jacobe)
2. **Clear grouping**: Group by functionality with comments
3. **Minimal surface**: Remove redundant, confusing, or internal methods
4. **Stateless preferred**: Pass q explicitly (like Peter Corke), but also support stateful for convenience
5. **Industrial + Academic**: Cover both practical use (like RoboDK) and research (like Peter Corke)

### Proposed Header Organization

```cpp
class Robot : public Entity {
public:
    // ===== CONSTRUCTION =====
    explicit Robot(const std::string& name);
    static Robot from_urdf(const std::string& urdf_path);
    static Robot from_urdf_string(const std::string& urdf_string);

    // ===== ROBOT MODEL =====
    int dof() const;
    int num_links() const;
    int num_joints() const;
    int num_tools() const;
    bool is_valid() const;

    std::pair<Eigen::VectorXd, Eigen::VectorXd> joint_limits() const;

    // ===== JOINT SPACE (Configuration) =====
    void set_q(const Eigen::VectorXd& q);
    const Eigen::VectorXd& q() const;

    // Standard configurations
    void set_home(const Eigen::VectorXd& q);
    const Eigen::VectorXd& home() const;
    bool has_home() const;
    Eigen::VectorXd q_zero() const;

    // ===== FORWARD KINEMATICS =====
    // Stateless (explicit q parameter)
    math::SE3 fk(const Eigen::VectorXd& q) const;
    math::SE3 fk(const Eigen::VectorXd& q, const std::string& link_name) const;
    math::SE3 fk(const Eigen::VectorXd& q, int link_id) const;
    std::vector<math::SE3> fk_all(const Eigen::VectorXd& q) const;

    // Stateful (uses stored q)
    math::SE3 fk() const;
    math::SE3 fk(const std::string& link_name) const;
    math::SE3 fk(int link_id) const;
    std::vector<math::SE3> fk_all() const;

    // ===== DIFFERENTIAL KINEMATICS =====
    Eigen::MatrixXd jacob0(const Eigen::VectorXd& q) const;
    Eigen::MatrixXd jacobe(const Eigen::VectorXd& q) const;
    Eigen::MatrixXd jacob0() const;
    Eigen::MatrixXd jacobe() const;

    // ===== INVERSE KINEMATICS =====
    // (Step 7 - placeholders)
    // Eigen::VectorXd ik(const math::SE3& T) const;
    // bool ik(const math::SE3& T, Eigen::VectorXd& q_out) const;

    // ===== BASE FRAME =====
    void set_base(const math::SE3& base);
    const math::SE3& base() const;

    // ===== TOOL / TCP =====
    int add_tool(const Tool& tool);
    const Tool& tool(int id) const;
    const Tool& tool(const std::string& name) const;

    void set_active_tool(int id);
    void set_active_tool(const std::string& name);
    bool has_active_tool() const;
    const Tool& active_tool() const;

    // ===== LINKS & JOINTS ACCESS =====
    const Link& link(int id) const;
    const Link& link(const std::string& name) const;
    const Joint& joint(int id) const;
    const Joint& joint(const std::string& name) const;

private:
    KinematicTree tree_;
    math::SE3 base_frame_;
    std::vector<Tool> tools_;
    int active_tool_id_;
    Eigen::VectorXd home_position_;
    std::unordered_map<std::string, int> link_name_to_id_;
    std::unordered_map<std::string, int> joint_name_to_id_;
    std::unordered_map<std::string, int> tool_name_to_id_;
};
```

### Key Changes

#### Removed (18 methods → cleaner):
- `Robot(name, parent)` - rarely used
- `add_link()`, `add_joint()` - use factory methods only
- `pose()` override - confusing for robots, use `fk()` instead
- `set_pose()` - not implemented, will be `ik()` later
- `move_base_by()` - unclear, use `set_base(base() * delta)` instead
- `kinematic_tree()` - internal implementation detail
- `joints()`, `set_joints()` - renamed to `q()`, `set_q()`
- `compute_fk()`, `get_tcp_pose()`, `get_current_tcp_pose()`, `compute_all_link_poses()` - unified as `fk()`, `fk_all()`
- `base_link()`, `flange_link()`, `base_link_id()`, `flange_link_id()` - use `link(0)`, `link(num_links()-1)`
- Non-const `link()`, `joint()`, `tool()` accessors - model is const, configure via `set_q()`
- `has_link()`, `has_joint()`, `link_id()`, `joint_id()`, `has_tool()`, `tool_id()` - internal queries, not typically needed

#### Added (6 methods):
- `joint_limits()` - query joint limits
- `q_zero()` - standard zero configuration
- `jacob0(q)`, `jacobe(q)` - Jacobian matrices (with stateful variants)
- `fk()` stateful variants - convenience methods using stored q

#### Renamed (4 methods):
- `joints()` → `q()` (standard notation)
- `set_joints()` → `set_q()`
- `base_frame()` → `base()` (shorter)
- `set_base_frame()` → `set_base()`

### Benefits

1. **Clear organization**: 7 clear sections (Construction, Model, Joint Space, FK, Jacobians, Base, Tools)
2. **Consistent naming**: `fk()` for all FK, `q()` for configuration, `jacob0/e()` standard names
3. **Less clutter**: 44 methods → 32 methods (-27%)
4. **More capable**: Added Jacobians, joint limits, q_zero
5. **Intuitive**: Stateless (explicit q) and stateful (stored q) patterns clear
6. **Industry standard**: Matches RoboDK simplicity + Peter Corke comprehensiveness

### Usage Examples

```cpp
// Construction
Robot robot = Robot::from_urdf("ur5.urdf");

// Model queries
int dof = robot.dof();
auto [qmin, qmax] = robot.joint_limits();

// Joint space configuration
Eigen::VectorXd q = robot.q_zero();
q(1) = 0.5;
robot.set_q(q);

// Forward kinematics (stateless)
math::SE3 T_tcp = robot.fk(q);
math::SE3 T_link3 = robot.fk(q, "link_3");
auto all_poses = robot.fk_all(q);

// Forward kinematics (stateful - uses stored q)
robot.set_q(q);
math::SE3 T_tcp = robot.fk();
math::SE3 T_link3 = robot.fk("link_3");

// Jacobian
Eigen::MatrixXd J0 = robot.jacob0(q);  // Base frame
Eigen::MatrixXd Je = robot.jacobe(q);  // End-effector frame

// Tool management
int id = robot.add_tool(gripper);
robot.set_active_tool(id);
math::SE3 T_tcp = robot.fk(q);  // Includes active tool offset

// Base frame
robot.set_base(SE3::Translation(0, 0, 0.5));  // Robot on table
```

Much clearer and more intuitive!

## Implementation Plan

1. Add new methods (`fk`, `jacob0`, `jacobe`, `joint_limits`, `q_zero`)
2. Add deprecated warnings to old methods
3. Update all tests to use new API
4. Remove deprecated methods
5. Update documentation

## Open Questions

1. Should we keep Entity base class and `pose()` override, or remove it?
2. Do we need velocity/acceleration state (`qd()`, `qdd()`) now, or wait for dynamics?
3. Should `link()` / `joint()` return const only, or keep non-const for modifications?
