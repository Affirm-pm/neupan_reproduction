# CLAUDE.md — FineNav-Engine

## Project Overview

FineNav-Engine is a **navigation application accelerator framework** for autonomous robots (wheelchairs, AMRs, etc.). Built on ROS 2 Humble and C++20, it provides a **core framework** that defines the organizational structure and interfaces for planner and map modules. External algorithm developers implement these interfaces to build custom planners and maps.

- **Author:** Jinghui Wang (wangjinghui@sjtu.edu.cn), IWIN-FINS Lab, SJTU
- **License:** MPL 2.0
- **C++ Standard:** C++20
- **Build System:** colcon + ament_cmake / ament_cmake_auto
- **Libraries:** Eigen3, PCL, OpenCV, nlohmann-json, BehaviorTree.CPP, fmt
- **Code Style:** Google-based `.clang-format` with 4-space indent, 120-char line limit

---

## Architecture

```
FineNavEngine  ─── Application entry point / orchestrator
  ├── FineNavLocalizer      (ESKF state estimation)
  ├── Navigator             (Behavior-tree task navigator)
  ├── MapServer<MapT>       (Map lifecycle + sensor fusion)
  └── ControlLayer<PlannerSet, Policy>
       └── bind_map_view() ── connects Map → Planner via MapView
```

### Core Design Philosophy

The engine uses **C++20 concepts** (not virtual base classes) to define the two fundamental contracts:

1. **`MapConcept`** — A spatial data container with a window/rolling mechanism: `M: Position3D → DataType`
2. **`PlannerConcept`** — An algorithm that reads a MapView and generates a trajectory: `P: (State, Reference, MapView) → Trajectory`

Planners **do not own maps**; they receive lightweight `IMapView` adapters that expose only what the algorithm needs. This enables loose coupling and independent testing.

---

## Package Map

| Package | Role |
|---|---|
| `finenav_core` | Core framework: concepts, MapServer, ControlLayer, PlannerSet, DataPacket |
| `finenav_engine` | Top-level `FineNavEngine` orchestrator |
| `finenav_interface` | **Reference implementations** showing how to implement MapConcept / PlannerConcept |
| `finenav_map/grid_map` | Template-based 3D rolling grid map (`GridMap<T>`) |
| `finenav_map/occ_grid_2d` | 2D occupancy grid map implementing MapConcept |
| `finenav_planner/finenav_route_planner` | A*, BFS, and topological route (GeoJSON) planners |
| `finenav_planner/finenav_mppi_controller` | MPPI controller with configurable critics |
| `finenav_navigator` | Behavior-tree-driven task navigator (BehaviorTree.CPP) |
| `finenav_localizer` | ESKF (Error-State Kalman Filter) for 6-DOF state estimation |
| `finenav_motion_decorator` | Trajectory post-processing: trim, smooth, seek-to-nearest |
| `finenav_collision_model` | OBB collision model with precompute/check API |
| `finenav_util` | Utilities: AlgoConfigurator, NodeThread, CloudPublishHelper, SimpleActionServer |
| `finenav_msgs` | ROS 2 message/action definitions |
| `finenav_demo` | Integration demos (STVL + MPPI) |
| `finenav_route` | (WIP, untracked) |
| `finenav_simulator` | (WIP, untracked) |

---

## Key Interfaces

### MapConcept (`finenav_core/include/finenav_core/concepts.hpp:30`)

```cpp
template <typename T>
concept MapConcept = requires(T map, const Position3D& pos,
                               const typename T::ConfigType& config,
                               const typename T::DataType& data) {
    typename T::ConfigType;
    typename T::DataType;
    { map.configure(config) } -> std::same_as<void>;
    { map.isInside(pos) }      -> std::same_as<bool>;
    { map.getWindowBounds() }  -> std::convertible_to<Region3D>;
    { map.getWindowCenter() }  -> std::convertible_to<Position3D>;
    { map.shiftWindowTo(pos) } -> std::same_as<void>;
};
```

**Implementations:** `OccGrid2DMap` (`finenav_map/occ_grid_2d/`), `GridMap<T>` (`finenav_map/grid_map/`), `SimpleTestMap` (`finenav_interface/`).

### PlannerConcept (`finenav_core/include/finenav_core/concepts.hpp:69`)

```cpp
template <typename T>
concept PlannerConcept = requires(T planner, const typename T::ConfigType& config,
                                   const PlanningContext<T>& ctx,
                                   const typename T::IMapView& map_view) {
    typename T::ConfigType;
    typename T::IMapView;
    typename InputDataT<T>;
    typename OutputDataT<T>;
    { planner.configure(config) }  -> std::same_as<void>;
    { planner.reset() }            -> std::same_as<void>;
    { planner.plan(ctx, map_view) } -> std::same_as<std::optional<OutputDataT<T>>>;
};
```

Each planner defines:
- `ConfigType` — generated parameter struct (via `generate_parameter_library`)
- `IMapView` — inner class defining the map interface the planner needs
- `InputProfile` / `OutputProfile` — `IOProfile` constexpr configuring which fields (pose, vel, accel, time) the `DataPacket` carries
- `plan(ctx, map_view)` — core algorithm returning `std::optional<OutputDataT<Self>>`
- `reset()` — called at episode boundaries (new goal arrived)
- `configure(params)` — dynamic parameter update

**Implementations:**
| Planner | Package | IMapView |
|---|---|---|
| `AstarPathSearch` | `finenav_route_planner` | `astar::IMapView` |
| `BfsPathSearch` | `finenav_route_planner` | `astar::IMapView` |
| `RoutePathSearch` | `finenav_route_planner` | `astar::IMapView` |
| `MPPIController` | `finenav_mppi_controller` | `nav2_mppi_controller::IMapView` |
| `SimpleTestPlanner` | `finenav_interface` | inner `IMapView` |

### IMapView for Route Planners (`finenav_planner/finenav_route_planner/include/imap_view.hpp`)

The unified map interface shared by `AstarPathSearch`, `BfsPathSearch`, and `RoutePathSearch`:
```cpp
class IMapView {
    virtual bool isWalkable(const Position3D& p) const = 0;  // world-frame
    virtual double getCost(const Position3D& p) const = 0;   // extra traversal cost (>= 0)
    virtual double getStepSize() const = 0;                  // metres per grid step
};
```

### Motion Decorator (`finenav_motion_decorator/include/i_motion_decorator.hpp`)

Trajectory post-processing with pipe syntax:
```cpp
traj | SeekToNearestPoint(robot_pos) | SmoothPath() | TrimByDistance(3.0);
```

Decorators: `SeekToNearestPoint`, `SmoothPath`, `TrimByDistance`, `TrimByDuration`, `TrimByCount`, `TrimByAABB`.

### Collision Model (`finenav_collision_model/`)

OBB-based with precompute/check split for high-frequency MPPI usage:
```cpp
model.precompute(world_pose);
bool collides = model.checkCollision([](const Position3D& p) { return map.isOccupied(p); });
int  cost     = model.checkCost([](const Position3D& p) { return map.getCost(p); });
```

---

## Execution Pipeline

### Two Planning Policies (`finenav_core/include/finenav_core/plan/execution_policy.hpp`)

- **`SingleShotPolicy`** — Action goal → one `plan()` call → succeed. For offline/A*-BFS.
- **`TrackingPolicy{20.0}`** — Action goal → loop `plan()` at N Hz until canceled or goal reached. For MPPI.

### Plan Lifecycle (per ControlLayer)

```
Goal arrives → execute()
  → resetActivePlanner()         [episode boundary]
  → planOnce(goal):
      1. on_pre_plan(ctx)         [user callback: trim reference traj, smooth, etc.]
      2. build PlanningContext    [robot_state from Localizer, goal_pose, ref_traj]
      3. map_view_factory()       [RAII read lock, build IMapView from MapT]
      4. planner.plan(ctx, view)  [algorithm execution]
      5. on_post_plan(ctx)        [user callback: publish cmd_vel, etc.]
```

### Pipeline Chaining

Multiple `ControlLayer`s can be chained via `FineNavEngine::registerPipeline()`:
```
line_path → [RoutePlanner (SingleShot)] → mppi_layer_plan → [MPPIController (Tracking)] → /cmd_vel
```

---

## AlgoConfigurator System (`finenav_util/include/finenav_util/algo_configurator.hpp`)

Every algorithm (map/planner) registers its parameter handling via the `FINENAV_REGISTER_ALGO_CONFIG` macro:

```cpp
FINENAV_REGISTER_ALGO_CONFIG(finenav::AstarPathSearch, astar_planner)
```

This:
1. Instantiates a `astar_planner::ParamListener` from `generate_parameter_library`
2. Calls `algo.configure(params)` with initial parameters
3. Registers a dynamic update callback
4. Returns the listener's lifetime as `shared_ptr<void>`

Parameters are defined in `.yaml` files alongside the package (e.g., `astar.yaml`, `bfs.yaml`, `route.yaml`).

---

## Behavior Tree Navigation (`finenav_navigator/`)

Task-level orchestration uses BehaviorTree.CPP v4. Available BT plugins:

| Plugin | Type | Purpose |
|---|---|---|
| `ComputePlan` | Action | Sends goal to ControlLayer action server |
| `IsGoalReached` | Condition | Checks pose + speed tolerance against blackboard |
| `IsStuck` | Condition | Timeout-based stuck detection |
| `PublishZeroCmdVel` | Action | Sends zero velocity for safe stop |

BT XML files are application-provided (see `finenav_example/getting_started/behavior_trees/`); the framework ships no built-in trees.

---

## DataPacket / IOProfile System (`finenav_core/include/finenav_core/plan/data_packet.hpp`)

Trajectory data uses mixin-based composition controlled by compile-time `IOProfile`:
```cpp
static constexpr IOProfile InputProfile  = { .pose = true, .vel = false, .accel = false, .time = false };
static constexpr IOProfile OutputProfile = { .pose = true, .vel = false, .accel = false, .time = false };
```

Only enabled fields occupy memory — no wasted space for unused velocity/acceleration data.

---

## MapServer Pattern

`MapServer<MapT>` manages a single map instance with:
- **Periodic update cycle** (sensor fusion): `addUpdateSource(name, policy, fuser)` registers data sources
- **RAII locked views**: `getLockedReadView()` → `LockedMapRO` (shared lock), `getLockedWriteView()` → `LockedMapRW` (exclusive lock)
- **Optional window tracking**: follows robot via `FineNavLocalizer` pointer
- **Pre/post-update hooks**: for external processing (STVL pruning, visualization)

---

## Common Patterns When Adding a New Algorithm

### Adding a New Planner

1. Define the class with `ConfigType`, `InputProfile`, `OutputProfile`, inner `IMapView` class
2. Implement `configure()`, `reset()`, `plan()`
3. Register with `FINENAV_REGISTER_ALGO_CONFIG(MyPlanner, my_planner_ns)`
4. Create `my_planner_ns.yaml` parameter file
5. Add to a `PlannerSet<...>` when instantiating a `ControlLayer`

### Adding a New Map

1. Implement `MapConcept` interface: `configure()`, `isInside()`, `getWindowBounds()`, `getWindowCenter()`, `shiftWindowTo()`
2. Define `ConfigType`, `DataType` type aliases
3. Register with `FINENAV_REGISTER_ALGO_CONFIG(MyMap, my_map_ns)`
4. Create an `IMapView` adapter for each planner that consumes this map

---

## Build Commands

```bash
cd /home/huigg/Desktop/FineNav_Engine_ws
colcon build --symlink-install
source install/setup.bash
```

## Key Dependencies

- **ROS 2 Humble** (rclcpp, rclcpp_action, tf2, nav_msgs, sensor_msgs, geometry_msgs)
- **generate_parameter_library** — for compile-time parameter generation from YAML
- **Eigen3** — linear algebra (Eigen::Vector3d, Eigen::AlignedBox3d, Eigen::Isometry3d)
- **PCL** — point cloud processing (STVL terrain mapping)
- **BehaviorTree.CPP v4** — task-level navigation logic
- **nlohmann-json** — GeoJSON parsing for route graphs
- **OpenCV** — image/occupancy-grid processing utilities

## Naming Conventions & Code Style

- Namespace: `finenav` for most code; `astar` for pure search algorithms; `route` for route graph
- File naming: `snake_case` for `.hpp` / `.cpp`
- Types: `PascalCase` for classes/structs (`ControlLayer`, `MapServer`)
- Variables: `snake_case_` for members, `snake_case` for locals
- Concepts: `PascalCase` suffix `Concept` (`MapConcept`, `PlannerConcept`)
- Template parameter naming for maps: `MapT`; for planners: `PlannerT`, `PlannersT`
- ROS node wrapper pattern: `NodeThread` runs a private node in a dedicated thread
