# Getting Started - Simulation

**Version: 0.5.0**

Minimal FineNav-Engine navigation stack driven by a simulator.

```text
/goal_pose -> FineNavEngine -> branch "navigate"
                                  |
                       DummyLinePlanner
                       EventDrivenPolicy
                                  |
                             pose path
                                  |
                       DummyFollowPlanner
                       PeriodicPolicy{10.0}
                                  |
                             /cmd_vel

DummyLineWorldView ----+
                       +---- shared DummyMap
DummyFollowWorldView --+
```

## Prerequisites

- Ubuntu 22.04 and ROS 2 Humble
- Workspace built with `colcon build --symlink-install`
- Workspace environment sourced with `source install/setup.bash`

## Run

Start the simulator, navigation application, and RViz together:

```bash
ros2 launch finenav_example hello_finenav.launch.py
```

The launch file uses Gazebo odometry and loads runtime settings from
[config/hello_finenav.yaml](../config/hello_finenav.yaml).
Press **Play** in Gazebo. RViz opens with the `map` frame, robot model, TF,
odometry, and pose tools configured by
[config/hello_finenav.rviz](../config/hello_finenav.rviz).
Gazebo odometry supplies the robot pose without manual initialization.
Use **2D Goal Pose** to select a goal.

Set `use_rviz:=false` to omit RViz. Set `world_name` to a world file in
`finenav_simulator/resource/worlds`, or `params_file` to your application YAML.

Alternatively, send a goal at a nearby unobstructed position:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

The coordinates above are an example; choose them for the robot's actual spawn
pose and world. Inspect the output with:

```bash
ros2 topic echo /cmd_vel
```

## Pipeline

`hello_finenav.cpp` loads the `finenav_example` plugin package and creates two
control layers:

```cpp
engine->createControlLayer(
    "dummy_line_world_view", "line_layer", core::EventDrivenPolicy{});
engine->createControlLayer(
    "dummy_follow_world_view", "follow_layer", core::PeriodicPolicy{10.0});
engine->registerControlBranch("navigate", {"line_layer", "follow_layer"});
```

`line_layer` generates a straight pose path for each new goal. `follow_layer`
selects a fixed-distance lookahead point from that path and publishes velocity
commands with a configured periodic rate of 10 Hz. FineNavEngine resolves both
WorldView registrations to the same `DummyMap` type and reuses one map server.

`DummyLineWorldView` implements `DummyLinePlanner::IWorldView`;
`DummyFollowWorldView` implements `DummyFollowPlanner::IWorldView`. They are
separate adapters for the two Planners.

The `GettingStartedNavigate` behavior tree activates the `navigate` branch.
It requests a stop when `IsGoalReached` succeeds and can restart from a faulted
control layer, with up to three recovery attempts.

## Example limitations

DummyMap treats all positions as unoccupied, so the example does not avoid
obstacles. DummyFollowPlanner does not implement terminal braking or final
heading alignment. The default goal condition checks position, heading, and
speed; reaching the goal position alone may therefore leave navigation active.
Configure compatible completion conditions or use a planner with the required
terminal behavior before relying on automatic goal completion.

## Parameters

| Node | Parameter | Purpose |
| --- | --- | --- |
| `/hello_finenav` | `use_sim_time` | Use the simulator clock. |
| `/hello_finenav/finenav_navigator` | `behavior_trees` | Load the example behavior tree. |
| `/hello_finenav/finenav_localizer` | `odom_topic` | Read simulator odometry. |
| `/hello_finenav/finenav_localizer` | `state_history_duration` | Retain states for delayed measurements. |
| `/hello_finenav/dummy_map` | `dummy_map.window_size` | Read-only example parameter; currently stored without changing the fixed map bounds. |
| `/hello_finenav/line_layer` | `dummy_line_planner.path_resolution` | Set the straight-path sample spacing. |
| `/hello_finenav/follow_layer` | `dummy_follow_planner.*` | Set tracking speeds and lookahead distance. |

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Robot does not move | Confirm `/cmd_vel` is published and simulation time advances. |
| No navigation tree | Confirm the workspace is sourced and `behavior_trees` resolves. |
| Plugin not registered | Check the `finenav_example` ament plugin resource after building. |
