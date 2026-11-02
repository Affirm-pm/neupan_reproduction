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
| `finenav_common` | Bottom layer shared by `finenav_core` and `finenav_visualizer`: the `Position3D`/`Region3D` geometry vocabulary, the `Frame`/`frames()` vocabulary, `IParamPollable`, and the cross-DSO registry spine (`RegistryTable`, `FINENAV_DEFINE_REGISTRY_SINGLETON`) |
| `finenav_core` | Core framework: concepts, MapServer, ControlLayer, PlannerSet, DataPacket |
| `finenav_engine` | Top-level `FineNavEngine` orchestrator |
| `finenav_interface` | **Reference implementations** showing how to implement MapConcept / PlannerConcept |
| `finenav_map/grid_map` | Template-based 3D rolling grid map (`GridMap<T>`) |
| `finenav_map/occ_grid_2d` | 2D occupancy grid map implementing MapConcept |
| `finenav_planner/finenav_route_planner` | A*, BFS, and topological route (GeoJSON) planners |
| `finenav_planner/finenav_mppi_SE2` | SE(2) ground MPPI planner with configurable critics |
| `finenav_navigator` | Behavior-tree-driven task navigator (BehaviorTree.CPP) |
| `finenav_localizer` | ESKF (Error-State Kalman Filter) for 6-DOF state estimation |
| `finenav_motion_decorator` | Trajectory post-processing: trim, smooth, seek-to-nearest |
| `finenav_collision_model` | OBB collision model with precompute/check API |
| `finenav_util` | Generic ROS plumbing, no navigation vocabulary: NodeThread, SimpleActionServer, `FINENAV_LOG_*` macros, Stopwatch |
| `finenav_msgs` | ROS 2 message/action definitions |
| `finenav_demo` | Integration demos (STVL + MPPI) |
| `finenav_route` | (WIP, untracked) |
| `finenav_simulator` | (WIP, untracked) |

### One spelling per name

A plugin includes **only** the role entry it needs (`finenav_core/plugin_map.hpp`,
`plugin_planner.hpp`, `plugin_world_view.hpp`, or `algo_register_macros.hpp`) —
the include root *is* the plugin contract; anything under a subdirectory is
framework-internal. Names then split by layer, with no second spelling for
either:

- **Vocabulary → `finenav::common::`** — `Position3D`, `Region3D`, `Frame`,
  `frames()`. These are conventions the whole framework shares, defined below
  `finenav_core` (the visualizer needs them too). Their free functions
  (`operator+`, `norm`, `abs`, `contains`, `center`, `sizes`) are found by ADL,
  so no using-declaration is needed at a call site.
- **Role interface → `finenav::core::`** — the `Map` / `Planner` / `WorldView`
  concepts, `Trajectory`, `IOProfile`, `MapRuntime`, `PlanningContext`.

`finenav_core` deliberately does **not** re-export the vocabulary under a
`finenav::core::` name: one type has one spelling, so a plugin author never has
to guess which of two names for the same type is the right one.

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
| `MPPISE2` | `finenav_mppi_SE2` | `finenav_mppi_SE2::IWorldView` |
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

## Control Pipeline (dynamic scheduling & fault handling)

The pipeline is a dataflow of single-planner stages (`ControlLayer<PlannerT>`) chained into branches and started/stopped at runtime by the behavior tree. The subtle part is the **state machine** built from START/STOP commands, per-stage feedback statuses, and the stashed reference between stages — a change that looks local to one of these pieces usually breaks an invariant two layers away.

Files: `finenav_core/include/finenav_core/plan/{control_layer,pipeline_node,i_pipeline_node,execution_policy}.hpp` (+ `plan/detail/*_impl.hpp`), `finenav_core/{include/finenav_core,src}/task_bus/task_bus.{hpp,cpp}`, `finenav_navigator/{include,src}/pipeline_controller.{hpp,cpp}`, `finenav_navigator/src/plugins/action/*.cpp`.

### Topology & `_root`

- A `BranchTable` holds named, ordered stage sequences; the same stage name in two branches is the **same resource** (fan-in / fan-out), so it is really a small dataflow graph.
- **`_root` is not a stored node.** `registerControlBranch()` wires `stages.front() → connectFrom("_root")` and `stages[i] → connectFrom(stages[i-1])` (`finenav_engine_cpp/src/finenav_engine.cpp:312-317`). The navigation goal is a `TaskFeedback{RUNNING}` published on `_root/__feedback` by `PipelineController::publishNavigationGoal()`. There is no `_root` worker, so `stopAll()` never touches it and the goal survives every STOP.
- Branch output is the **leaf**'s feedback, forwarded to `navigator::IChassisCommand`; multiple fresh leaves → refuse + warn.
- Layers' private nodes and the navigator both use the host node's name as *namespace*, so their topic names agree; intra-process comms make the `shared_ptr<const TaskFeedback>` pass-through zero-copy.

### TaskBus (`TaskMaster` / `TaskWorker`)

Two intra-process topics per stage name:

| Topic | Type | Direction |
|---|---|---|
| `<name>/__cmd` | `TaskCmd` (START / STOP) | Master → Worker |
| `<name>/__feedback` | `TaskFeedback` (IDLE / RUNNING / STOPPED / FAULTED) | Worker → Master, and → every stage that `connectFrom()`-ed it |

- `TaskMaster::feedback(name)` returns `nullptr` until the first message arrives — never treat as a fault.
- `ackStart()` consumes START **and clears any pending STOP**: *a START supersedes a prior STOP* (`task_bus.hpp:113-135`).
- Only `RUNNING` carries a trajectory.

### Two-level status model — report, never guess

- **Micro (per stage):** each `ControlLayer` publishes its own status; the only framework policy is *fault propagation* (FAULTED upstream → this stage publishes FAULTED). A broken pipeline therefore shows a FAULTED **suffix**, and the most-upstream FAULTED stage is the root cause — what `GetFaultedControlLayer` returns.
- **Macro (per branch):** the branch's state = its leaf's state; `ExecuteControlBranch::onRunning` returns FAILURE if *any* stage is FAULTED.
- The framework **reports** status and **never chooses recovery**. Recovery is BT logic: `GetFaultedControlLayer → RestartFromControlLayer` (point self-heal from the root cause), `RestartControlBranch` (whole branch), or let FAILURE propagate to `stopAll()`.

### Trigger policies & worker lifecycle

Each layer runs one parked worker thread (`control_layer_impl.hpp:62-79`):

```
while (ok && !stop_):
    if hasStart():
        ackStart()            # consumes START, clears any STOP
        waitForState()        # blocks until a RobotState exists (or STOP/shutdown)
        resetPlanner()        # episode boundary, once per START
        episode_body()        # runPeriodic() or runEventDriven()
    sleep(1ms)
```

- **`PeriodicPolicy{frequency_hz}`** — a `rclcpp::Rate` loop: `shouldStop → STOPPED`; `planRefState()` gates (`kUpstreamFaulted → FAULTED`, `kNoInput → sleep+continue`, `kValid → plan`); `planOnce()` `nullopt → FAULTED`.
- **`EventDrivenPolicy`** — one `planOnce()` per *new* upstream reference, **on the worker thread**. `onPlanRef()` only stashes the reference and sets `ref_pending_`; the episode body consumes the flag (commit `4ca5717`: planning moved off the callback thread → single-threaded planner + terminal FAULTED).
- `resetPlanner()` runs once per START, so a new mid-episode reference reaches `plan()` **without** a `reset()`.

### `PlanRefState` — the reference gate

`plan_ref_msg_` is the single stashed reference (last-writer-wins, never cleared on STOP). `planRefState()` maps its status (`pipeline_node_impl.hpp:127-135`):

| Stashed feedback | `PlanRefState` | Effect |
|---|---|---|
| none | `kNoInput` | wait |
| `FAULTED` | `kUpstreamFaulted` | propagate fault |
| `RUNNING` | `kValid` | plan |
| `IDLE` / `STOPPED` / unknown | `kNoInput` | wait |

Both policies gate on this *before* `planOnce()`, so upstream absence/fault surfaces as an explicit state — never as "planner returned nullopt". The reference is converted to `InputTrajectoryT` **copy-on-change**, keyed on the message identity.

### `planOnce()` and the `nullopt` contract

**`nullopt` == failure**: the planner had valid, ready input and produced no trajectory. It is the only thing that legitimately faults a stage. Reachable causes:

| Cause | Meaning |
|---|---|
| `getRobotState()` empty mid-episode | localizer dropout (faults the layer) |
| `planner.plan()` returns `nullopt` | the real algorithm failure |

Misconfiguration (no WorldView / no state provider) and "no reference yet" are **gated out before** `planOnce()` by `waitForState()` + `planRefState()` — do not use `nullopt` to mean "not ready".

Sequence: `pollAll()` → `PlanningContext{state, cached_ref}` (read-only views) → read-lock every map → `prepare_(ctx)` (once per plan, under the lock) → `planner.plan(ctx, view)` → `toROSMsg`. There is deliberately **no** pre/post-plan user callback — reference manipulation is an upstream decorator stage, result handling is `IChassisCommand`, visualization is the injected `FineNavVisualizer`.

### Episode boundary & reference hygiene

`clearFaultedPlanRef()` resets the stashed reference **only if FAULTED**; a `RUNNING` reference is kept (it is the current valid input — the `_root` goal, or the upstream's live output). Called at **every** `publishFaulted()` / `publishStopped()` site, so a restarted stage waits for fresh output but resumes the same goal.

### `PipelineController` — the BT's only entry point

- **`active_` invariant** (`pipeline_controller.hpp:84-87`): the ordered stages of the active branch; **empty = nothing active**. Set by `activateBranch`/`restartBranch`, cleared by `stopAll`; never derived from feedback.
- `activateBranch(b)` is a **set difference** (not stop-all/start-all): stop `active_∖b` **downstream-first**, start `b∖active_` **upstream-first**, keep shared stages. Re-activating the active branch is a **no-op** (use `restartFrom`/`restartBranch` to restart).
- `isActive(b)` = `active_ == branchStages(b)` and non-empty — the authority `onHalted()` uses.
- `restartFrom(layer)` restarts the active branch's suffix `[layer..leaf]`; `restartBranch(b)` restarts the whole branch and marks it active; `stopBranch(b)` stops a branch **without** clearing `active_`.

### BT integration

`ExecuteControlBranch` (`StatefulActionNode`, port `branch`):

| Callback | Behaviour |
|---|---|
| `onStart` | FAILURE if `_pipeline` missing or branch unknown; else `activateBranch` → RUNNING |
| `onRunning` | FAILURE if branch empty or any stage FAULTED; else RUNNING (never SUCCESS — completion is decided elsewhere) |
| `onHalted` | `stopAll()` **only if** `isActive(branch)` (a switch already handled stopping via set difference) |

Supporting nodes: `GetFaultedControlLayer`, `RestartFromControlLayer`, `RestartControlBranch`, `PublishNavigationGoal`, `FollowTargetAsGoal`.

### Invariants / gotchas

1. `nullopt` == failure; "not ready" is gated out before `planOnce()`.
2. FAULTED is **terminal per episode** (the body returns; a later STOP does not overwrite it).
3. STOP never clears the reference; only a FAULTED reference is cleared.
4. A new reference does **not** call `reset()` — planners that accumulate state across a mid-episode reference change must handle it in `plan()`.
5. An EventDriven stage plans per *event*, not per stashed reference — `ref_pending_` must be set by a fresh `onPlanRef`. Consequence: `restartFrom()` on an event-driven **first** stage (whose only upstream is `_root`) leaves it waiting for a new goal publish instead of re-planning the stashed goal. No reachable code path today, but note it before wiring recovery for a first-stage fault.
6. Re-activating the active branch is a no-op; `restartFrom`/`restartBranch` are the restart paths.
7. START supersedes STOP within a 1 ms window — you cannot start-and-stop in one round-trip.
8. Only `RUNNING` feedback carries a trajectory; `nullptr` feedback is legitimate (pre-warm), not a fault.

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
