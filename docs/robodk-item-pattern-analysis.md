# RoboDK Item Pattern: Analysis & Application to Robospace

**Date:** 2025-11-08
**Purpose:** Analyze RoboDK's unified "Item" architecture and apply insights to robospace design

---

## Executive Summary

RoboDK uses a brilliant **unified Item pattern** where everything in the workspace (robots, targets, tools, frames, sensors, programs, etc.) is represented as an `Item` object. This provides exceptional API simplicity and consistency.

**Key Insight:** Instead of having separate classes for Robot, Tool, Target, Frame, etc., **one Item class does it all** - the behavior changes based on the item's type.

**Recommendation for Robospace:** Adopt a similar but enhanced pattern for our web-based, modern architecture.

---

## How RoboDK's Item Pattern Works

### 1. Core Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    RoboDK Application                   │
│              (Separate desktop process)                 │
└─────────────────────┬───────────────────────────────────┘
                      │
                      │ TCP/IP Socket Connection
                      │ (Commands & Responses)
                      │
┌─────────────────────▼───────────────────────────────────┐
│                 Robolink() Object                       │
│           (Client-side API connection)                  │
│                                                         │
│  ┌───────────────────────────────────────────────┐    │
│  │         Item Factory Methods                   │    │
│  │  - Item(name, type) → Item                    │    │
│  │  - ItemList() → List[Item]                    │    │
│  │  - AddRobot(file) → Item                      │    │
│  │  - AddFrame(name) → Item                      │    │
│  │  - AddTarget(name) → Item                     │    │
│  └───────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
                      │
                      │ Creates
                      ▼
          ┌─────────────────────┐
          │    Item Object      │
          ├─────────────────────┤
          │ - item: int (ID)    │  ← Pointer/ID to object in RoboDK
          │ - type: int         │  ← ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, etc.
          │ - link: Robolink    │  ← Reference to connection
          └─────────────────────┘
```

### 2. Item Type System

**All item types (20+ types):**

```python
# Core types
ITEM_TYPE_STATION = 1       # Workspace (.rdk files)
ITEM_TYPE_ROBOT = 2         # Robot
ITEM_TYPE_FRAME = 3         # Reference frame
ITEM_TYPE_TOOL = 4          # Tool/gripper
ITEM_TYPE_OBJECT = 5        # CAD object (.stl, .step)
ITEM_TYPE_TARGET = 6        # Target pose
ITEM_TYPE_CURVE = 7         # Curve/path
ITEM_TYPE_PROGRAM = 8       # Program (visual programming)
ITEM_TYPE_INSTRUCTION = 9   # Program instruction
ITEM_TYPE_PROGRAM_PYTHON = 10  # Python macro
ITEM_TYPE_MACHINING = 11    # Machining project
ITEM_TYPE_CAMERA = 19       # Camera sensor
ITEM_TYPE_GENERIC = 20      # Generic object
```

**The magic:** One `Item` class with type-specific behavior!

### 3. How It Works: Unified Interface

**Example: Everything is an Item**

```python
from robodk.robolink import *

# Connect to RoboDK
RDK = Robolink()

# Retrieve different types - all return Item objects
robot = RDK.Item('UR10e', ITEM_TYPE_ROBOT)
tool = RDK.Item('Gripper', ITEM_TYPE_TOOL)
frame = RDK.Item('Workstation', ITEM_TYPE_FRAME)
target = RDK.Item('Target 1', ITEM_TYPE_TARGET)
program = RDK.Item('MainProgram', ITEM_TYPE_PROGRAM)
camera = RDK.Item('Camera 1', ITEM_TYPE_CAMERA)

# All share common methods
robot.Name()        # → 'UR10e'
tool.Name()         # → 'Gripper'
camera.Name()       # → 'Camera 1'

robot.Pose()        # Current pose
tool.Pose()         # Tool pose relative to parent
camera.Pose()       # Camera pose

robot.setParent(frame)  # Attach robot to frame
tool.setParent(robot)   # Attach tool to robot
camera.setParent(frame) # Attach camera to frame

# Type-specific methods only work on appropriate types
robot.MoveJ(target)     # ✓ Works (robot-specific)
camera.MoveJ(target)    # ✗ Silently does nothing (not a robot)

# Check type
if robot.Type() == ITEM_TYPE_ROBOT:
    robot.MoveL(target)
```

### 4. Key Design Patterns

#### Pattern 1: Proxy/Handle Pattern

**Items are lightweight proxies to objects in RoboDK:**

```python
class Item:
    def __init__(self, link: Robolink, ptr_item: int, itemtype: int):
        self.link = link         # Connection to RoboDK
        self.item = ptr_item     # Unique ID (pointer) in RoboDK
        self.type = itemtype     # ITEM_TYPE_*
```

- **Item object** = Handle/ID + connection reference
- **Actual data** lives in RoboDK application
- **Every method call** = Network request to RoboDK

**Benefits:**
- Lightweight: Item objects are tiny (just 3 fields)
- No synchronization issues: RoboDK is single source of truth
- Consistent: All state managed centrally

#### Pattern 2: Tree/Scene Graph Structure

**Everything is a node in a tree:**

```
Station (root)
├── Frame: "Workstation"
│   ├── Robot: "UR10e"
│   │   ├── Tool: "Gripper"
│   │   └── Camera: "Wrist Camera"
│   ├── Object: "Table"
│   └── Target: "PickPoint"
└── Program: "MainProgram"
    ├── Instruction: "MoveJ"
    └── Instruction: "MoveL"
```

**Methods for tree manipulation:**

```python
# Tree navigation
item.Parent()           # Get parent item
item.Childs()          # Get all children
item.setParent(parent) # Move item in tree

# Query relationships
robot.getLink(ITEM_TYPE_TOOL)  # Get first tool attached to robot
frame.Childs()                  # All items under this frame
```

#### Pattern 3: Method Polymorphism Based on Type

**Same method name, different behavior based on type:**

```python
# Pose() method exists for all items, but meaning differs:
robot.Pose()    # → Base pose of robot in world frame
tool.Pose()     # → Tool pose relative to robot flange
target.Pose()   # → Target pose in space
object.Pose()   # → Object pose in world frame
camera.Pose()   # → Camera pose

# Type-specific methods
robot.Joints()           # Get joint angles (only robots)
robot.MoveJ(target)      # Move to target (only robots)
program.RunProgram()     # Execute program (only programs)
object.setColor([1,0,0]) # Set color (objects, robots, etc.)
```

**Implementation detail:** Methods check item type internally:

```python
def MoveJ(self, target):
    # Internally in RoboDK, this checks:
    # if (this.type != ITEM_TYPE_ROBOT):
    #     return  # Do nothing
    # else:
    #     ... execute move
```

#### Pattern 4: Type-Safe Retrieval

**Retrieve items with type filtering:**

```python
# Get first robot (any name)
robot = RDK.Item('', ITEM_TYPE_ROBOT)

# Get all robots
all_robots = RDK.ItemList(filter=ITEM_TYPE_ROBOT)

# Get all items
all_items = RDK.ItemList()

# User picks an item
frame = RDK.ItemUserPick('Select a frame', ITEM_TYPE_FRAME)
```

---

## C++ Implementation

**Same pattern in C++:**

```cpp
// Header: robodk_api.h
class Item {
    RoboDK *_RDK;        // Pointer to RoboDK connection
    quint64 _PTR;        // Item ID/pointer
    int _TYPE;           // Item type

public:
    // Common methods (all items)
    QString Name();
    void setName(const QString& name);
    Mat Pose();
    void setPose(const Mat& pose);
    void setParent(Item parent);
    Item Parent();

    // Type-specific methods
    void MoveJ(Item target);        // Robots only
    void MoveL(Item target);        // Robots only
    tJoints Joints();               // Robots only
    void RunProgram();              // Programs only

    // Type checking
    int Type();
    bool Valid();
};
```

**Usage:**

```cpp
RoboDK rdk;
Item robot = rdk.getItem("UR10e", RoboDK::ITEM_TYPE_ROBOT);
Item tool = rdk.getItem("Gripper", RoboDK::ITEM_TYPE_TOOL);
Item target = rdk.getItem("Target1", RoboDK::ITEM_TYPE_TARGET);

robot.setPoseTool(tool);
robot.MoveJ(target);
```

---

## Advantages of the Item Pattern

### ✅ Pros

1. **API Simplicity**
   - One class to learn instead of 20+
   - Consistent method names across all types
   - Easy to extend with new item types

2. **Flexibility**
   - Same code works with different item types
   - Generic programming: `for item in all_items: item.Pose()`
   - Easy to add new item types without breaking API

3. **Hierarchical Structure**
   - Natural parent-child relationships
   - Easy scene graph manipulation
   - Clear ownership semantics

4. **Type Safety (with checks)**
   - `item.Type()` returns type constant
   - Runtime type checking when needed
   - Explicit type filtering in queries

5. **Network Efficiency**
   - Items are lightweight (just IDs)
   - Actual data stays in RoboDK server
   - No large object transfers

### ❌ Cons

1. **No Compile-Time Type Safety**
   - Can call `camera.MoveJ(target)` - fails at runtime
   - IDE can't help with type-specific autocomplete
   - Easy to make type mistakes

2. **Documentation Complexity**
   - Which methods work on which types?
   - Need extensive documentation
   - Learning curve: "Does this method work on tools?"

3. **Performance**
   - Every method call = network round trip
   - Can't do local computation
   - Batching needed for complex operations

4. **No Strong Typing Benefits**
   - Can't use C++ templates effectively
   - No Python type hints distinguish item types
   - Refactoring tools have limited help

---

## How to Apply to Robospace

### Recommendation: **Hybrid Approach**

Combine the best of RoboDK's simplicity with modern type safety.

### Architecture for Robospace

```python
# Base class (like RoboDK's Item)
class Item:
    """Base class for all items in robospace workspace"""
    def __init__(self, scene_id: str, item_type: ItemType):
        self._id = scene_id
        self._type = item_type

    # Common methods for ALL items
    def name(self) -> str: ...
    def set_name(self, name: str): ...
    def pose(self) -> Transform: ...
    def set_pose(self, pose: Transform): ...
    def parent(self) -> Optional['Item']: ...
    def set_parent(self, parent: 'Item'): ...
    def children(self) -> List['Item']: ...
    def visible(self) -> bool: ...
    def set_visible(self, visible: bool): ...
    def type(self) -> ItemType: ...

# Specialized classes inherit from Item
class Robot(Item):
    """Robot-specific functionality"""
    def __init__(self, scene_id: str):
        super().__init__(scene_id, ItemType.ROBOT)

    # Robot-specific methods
    def joints(self) -> np.ndarray: ...
    def set_joints(self, q: np.ndarray): ...
    def move_j(self, target: Union['Target', Transform]): ...
    def move_l(self, target: Union['Target', Transform]): ...
    def jacobian(self) -> np.ndarray: ...

class Tool(Item):
    """Tool/gripper functionality"""
    def __init__(self, scene_id: str):
        super().__init__(scene_id, ItemType.TOOL)

    # Tool-specific methods
    def tcp_pose(self) -> Transform: ...
    def set_tcp_pose(self, pose: Transform): ...
    def open(self): ...
    def close(self): ...
    def grasp_force(self) -> float: ...

class Target(Item):
    """Target pose functionality"""
    def __init__(self, scene_id: str):
        super().__init__(scene_id, ItemType.TARGET)

    # Target-specific methods
    def target_pose(self) -> Transform: ...
    def set_target_pose(self, pose: Transform): ...
    def joint_config(self) -> Optional[np.ndarray]: ...

class Frame(Item):
    """Reference frame"""
    def __init__(self, scene_id: str):
        super().__init__(scene_id, ItemType.FRAME)

    # Frame has mostly base Item methods
    # (pose, parent, children)

class MachineTool(Item):
    """CNC machine, mill, lathe"""
    def __init__(self, scene_id: str, machine_type: MachineType):
        super().__init__(scene_id, ItemType.MACHINE_TOOL)
        self._machine_type = machine_type

    # Machine tool methods
    def axis_positions(self) -> Dict[str, float]: ...
    def set_axis_positions(self, positions: Dict[str, float]): ...
    def execute_gcode(self, program: str): ...
    def work_coordinate(self) -> Transform: ...

class LaserTracker(Item):
    """Metrology laser tracker"""
    def __init__(self, scene_id: str):
        super().__init__(scene_id, ItemType.LASER_TRACKER)

    # Tracker methods
    def measure(self) -> Transform: ...
    def track_smr(self) -> bool: ...
    def accuracy(self) -> float: ...

class Camera(Item):
    """Vision sensor"""
    def __init__(self, scene_id: str):
        super().__init__(scene_id, ItemType.CAMERA)

    # Camera methods
    def intrinsics(self) -> CameraIntrinsics: ...
    def capture(self) -> np.ndarray: ...
    def set_resolution(self, width: int, height: int): ...
```

### Scene/Workspace Manager (like Robolink)

```python
class Workspace:
    """Central workspace manager (like RoboDK's Robolink)"""

    def __init__(self):
        self._items: Dict[str, Item] = {}
        self._root = Frame("world")

    # Item retrieval (similar to RoboDK)
    def item(self, name: str, item_type: Optional[ItemType] = None) -> Optional[Item]:
        """Get item by name and optionally type"""
        ...

    def item_list(self, filter_type: Optional[ItemType] = None) -> List[Item]:
        """Get all items, optionally filtered by type"""
        ...

    # Item creation with specific types
    def add_robot(self, urdf_file: str, name: str) -> Robot:
        """Add robot from URDF, returns Robot object"""
        robot = Robot.from_urdf(urdf_file)
        robot.set_name(name)
        self._items[robot._id] = robot
        return robot

    def add_tool(self, name: str) -> Tool:
        """Add tool, returns Tool object"""
        tool = Tool(self._generate_id())
        tool.set_name(name)
        self._items[tool._id] = tool
        return tool

    def add_machine_tool(self, config: str, name: str) -> MachineTool:
        """Add CNC machine, returns MachineTool object"""
        machine = MachineTool.from_config(config)
        machine.set_name(name)
        self._items[machine._id] = machine
        return machine

    def add_laser_tracker(self, name: str, tracker_type: str) -> LaserTracker:
        """Add metrology laser tracker"""
        tracker = LaserTracker(self._generate_id())
        tracker.set_name(name)
        self._items[tracker._id] = tracker
        return tracker

    def add_target(self, name: str, pose: Transform) -> Target:
        """Add target pose"""
        target = Target(self._generate_id())
        target.set_name(name)
        target.set_target_pose(pose)
        self._items[target._id] = target
        return target
```

### Usage Example

```python
import robospace as rs

# Create workspace
ws = rs.Workspace()

# Add items with specific types
robot = ws.add_robot("ur10e.urdf", "Robot1")        # Returns Robot
tool = ws.add_tool("Gripper")                       # Returns Tool
mill = ws.add_machine_tool("dmu50.yaml", "Mill1")   # Returns MachineTool
tracker = ws.add_laser_tracker("Tracker1", "API")   # Returns LaserTracker
target = ws.add_target("Pick", pose)                # Returns Target

# Type safety: IDE knows these are specific types
robot.move_j(target)         # ✓ IDE autocompletes robot methods
tool.open()                  # ✓ IDE autocompletes tool methods
mill.execute_gcode("G0 X0")  # ✓ IDE autocompletes machine methods
tracker.measure()            # ✓ IDE autocompletes tracker methods

# But they're all Items!
all_items: List[rs.Item] = ws.item_list()
for item in all_items:
    print(f"{item.name()}: {item.pose()}")  # Common interface

# Type checking and casting
if isinstance(robot, rs.Robot):
    print(f"Robot joints: {robot.joints()}")

# Tree structure (like RoboDK)
robot.set_parent(ws.root())
tool.set_parent(robot)
mill.set_parent(ws.root())
tracker.set_parent(ws.root())
```

---

## Comparison: RoboDK vs Robospace

| Feature | RoboDK Approach | Robospace Approach |
|---------|----------------|-------------------|
| **Base Design** | Single `Item` class | Base `Item` + specialized subclasses |
| **Type Safety** | Runtime only | Compile-time (Python: type hints, C++: classes) |
| **API Consistency** | All items have same methods | Common methods in `Item`, specific in subclasses |
| **IDE Support** | Limited autocomplete | Full autocomplete for type-specific methods |
| **Flexibility** | Very flexible, untyped | Flexible but type-safe |
| **Learning Curve** | Need to memorize which methods work on which types | IDE guides you to correct methods |
| **Code Style** | `item.Type() == ITEM_TYPE_ROBOT` | `isinstance(item, Robot)` |
| **Adding New Types** | No code changes needed | Add new subclass |

---

## Implementation Recommendations for Robospace

### Phase 1: Core Item System (Month 1-2)

```python
# core/item.py
from enum import Enum
from typing import Optional, List, TYPE_CHECKING
import numpy as np

class ItemType(Enum):
    STATION = 1
    ROBOT = 2
    FRAME = 3
    TOOL = 4
    OBJECT = 5
    TARGET = 6
    PROGRAM = 8
    CAMERA = 9
    MACHINE_TOOL = 10
    LASER_TRACKER = 11
    LASER_SCANNER = 12

class Item:
    """Base class for all workspace items"""

    def __init__(self, item_id: str, item_type: ItemType):
        self._id = item_id
        self._type = item_type
        self._name = ""
        self._pose = Transform.identity()
        self._parent: Optional['Item'] = None
        self._children: List['Item'] = []
        self._visible = True

    # Properties
    @property
    def id(self) -> str:
        return self._id

    @property
    def type(self) -> ItemType:
        return self._type

    def name(self) -> str:
        return self._name

    def set_name(self, name: str):
        self._name = name

    # Pose methods
    def pose(self) -> Transform:
        """Get pose in parent frame"""
        return self._pose

    def set_pose(self, pose: Transform):
        """Set pose in parent frame"""
        self._pose = pose

    def pose_abs(self) -> Transform:
        """Get absolute pose in world frame"""
        if self._parent is None:
            return self._pose
        return self._parent.pose_abs() * self._pose

    # Tree methods
    def parent(self) -> Optional['Item']:
        return self._parent

    def set_parent(self, parent: Optional['Item']):
        if self._parent is not None:
            self._parent._children.remove(self)
        self._parent = parent
        if parent is not None:
            parent._children.append(self)

    def children(self) -> List['Item']:
        return self._children.copy()

    # Visibility
    def visible(self) -> bool:
        return self._visible

    def set_visible(self, visible: bool):
        self._visible = visible

    # Serialization (for web API)
    def to_dict(self) -> dict:
        return {
            'id': self._id,
            'type': self._type.name,
            'name': self._name,
            'pose': self._pose.to_dict(),
            'visible': self._visible,
            'parent_id': self._parent._id if self._parent else None
        }
```

### Phase 2: Specialized Item Classes

```python
# core/robot_item.py
class Robot(Item):
    """Robot item with kinematics and control"""

    def __init__(self, item_id: str, urdf_path: str):
        super().__init__(item_id, ItemType.ROBOT)
        self._model = RobotModel.from_urdf(urdf_path)
        self._joints = np.zeros(self._model.dof)
        self._fk = ForwardKinematics(self._model)
        self._ik = InverseKinematics(self._model)

    # Robot-specific methods
    def joints(self) -> np.ndarray:
        return self._joints.copy()

    def set_joints(self, q: np.ndarray):
        self._joints = q.copy()
        # Update end-effector pose
        self._update_ee_pose()

    def move_j(self, target: Union[Target, Transform],
               speed: float = 1.0) -> Trajectory:
        """Plan joint motion to target"""
        if isinstance(target, Target):
            target_pose = target.target_pose()
        else:
            target_pose = target

        # Solve IK
        q_target = self._ik.solve(target_pose, seed=self._joints)
        if q_target is None:
            raise ValueError("IK failed")

        # Plan trajectory
        planner = MotionPlanner(self._model)
        return planner.plan_joint_trajectory(self._joints, q_target, speed)

    def jacobian(self, frame: str = "tool0") -> np.ndarray:
        """Compute Jacobian"""
        jac_calc = JacobianCalculator(self._model)
        return jac_calc.compute(self._joints, frame)

    def dof(self) -> int:
        return self._model.dof

# core/machine_tool_item.py
class MachineTool(Item):
    """CNC machine tool"""

    def __init__(self, item_id: str, config_path: str):
        super().__init__(item_id, ItemType.MACHINE_TOOL)
        self._config = MachineToolConfig.from_file(config_path)
        self._axis_positions = {axis: 0.0 for axis in self._config.axes}

    def axis_positions(self) -> Dict[str, float]:
        return self._axis_positions.copy()

    def set_axis_positions(self, positions: Dict[str, float]):
        for axis, pos in positions.items():
            if axis not in self._axis_positions:
                raise ValueError(f"Unknown axis: {axis}")
            self._axis_positions[axis] = pos

    def execute_gcode(self, program: Union[str, Path]) -> GCodeExecution:
        """Execute G-code program"""
        parser = GCodeParser()
        instructions = parser.parse_file(program)
        executor = GCodeExecutor(self)
        return executor.execute(instructions)

    def work_coordinate(self, g_code: str = "G54") -> Transform:
        """Get work coordinate system"""
        return self._config.work_coordinates.get(g_code, Transform.identity())
```

---

## Benefits for Robospace

### 1. Type Safety + Flexibility

```python
# Type-safe robot operations
robot: Robot = ws.add_robot("ur10.urdf", "Robot1")
robot.move_j(target)  # ✓ Type checker knows this is valid

# But still flexible with base Item
def move_all_robots_home(items: List[Item]):
    for item in items:
        if isinstance(item, Robot):
            item.move_j(item.home_joints())

# Generic item operations
def export_scene(items: List[Item], filename: str):
    data = [item.to_dict() for item in items]
    json.dump(data, open(filename, 'w'))
```

### 2. Clear API for Machine Tools

```python
# Natural API for machine tools
mill: MachineTool = ws.add_machine_tool("dmu50.yaml", "Mill1")
robot: Robot = ws.add_robot("ur10e.urdf", "Robot1")

# Unified cell programming
cell = ManufacturingCell(ws)
cell.add_item(robot)
cell.add_item(mill)

# Robot loads part into mill
robot.move_to(mill.work_table_pose())
mill.open_door()
robot.place_part()
mill.close_door()
mill.execute_gcode("part_program.nc")
```

### 3. Metrology Integration

```python
# Laser tracker as a first-class item
tracker: LaserTracker = ws.add_laser_tracker("Tracker1", "API")
tracker.set_pose(tracker_base_pose)
tracker.connect("192.168.1.100")

# Robot calibration
calibrator = RobotCalibrator(robot, tracker)
calibration_poses = calibrator.generate_poses(num=50)

for pose in calibration_poses:
    robot.move_j(pose)
    measurement = tracker.measure()
    calibrator.add_sample(pose, measurement)

calibrated_dh = calibrator.calibrate()
robot.update_dh_parameters(calibrated_dh)
```

### 4. Web API Compatibility

```python
# REST API endpoint
@app.get("/api/items")
async def get_items(item_type: Optional[str] = None):
    """Get all items or filtered by type"""
    if item_type:
        items = workspace.item_list(filter_type=ItemType[item_type])
    else:
        items = workspace.item_list()

    return [item.to_dict() for item in items]

# WebSocket update
async def broadcast_item_update(item: Item):
    """Send item update to all connected clients"""
    await websocket_manager.broadcast({
        'type': 'item_update',
        'data': item.to_dict()
    })
```

---

## Conclusion

**RoboDK's Item pattern is brilliant for its simplicity and consistency.** For robospace, we should adopt a **hybrid approach**:

1. **Keep the unified Item base class** for common operations
2. **Add type-specific subclasses** (Robot, MachineTool, LaserTracker, etc.) for type safety
3. **Maintain tree structure** for scene graph hierarchy
4. **Support polymorphic operations** where appropriate

**Result:** Best of both worlds
- ✓ Simple, consistent API (like RoboDK)
- ✓ Type safety and IDE support (modern Python/C++)
- ✓ Extensible for new item types
- ✓ Perfect for web API serialization

This will make robospace **easier to use than RoboDK** while being **more powerful and flexible**!

---

**Next Steps:**
1. Implement base `Item` class
2. Create specialized classes (Robot, Tool, MachineTool, etc.)
3. Build `Workspace` manager
4. Design serialization for web API
5. Implement tree manipulation methods
