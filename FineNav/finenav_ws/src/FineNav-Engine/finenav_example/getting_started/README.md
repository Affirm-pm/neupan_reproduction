# Getting Started — Simulation

Minimal FineNav-Engine navigation stack driven by a simulator.

```
/odom → FineNavLocalizer (ESKF) → BtNavigator → ComputePlan("my_layer")
                                                    ↓
                                            ControlLayer<DummyPlanner>
                                                    ↓
                                               /cmd_vel
/goal_pose → FineNavEngine → navigateTo()
```

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- Workspace built: `colcon build --symlink-install && source install/setup.bash`

## Quick Start

### Terminal 1 — Simulator

```bash
ros2 launch finenav_simulator finenav_simulator.launch.py \
  lio_type:=static \
  use_sim_time:=true \
  world_name:=flat_small_scale_maze.world \
  publish_map_tf:=true
```

### Terminal 2 — FineNav-Engine

```bash
cd ~/FineNav_Engine_ws && source install/setup.bash
ros2 run finenav_example hello_finenav \
  --ros-args --params-file src/FineNav-Engine/finenav_example/getting_started/params/sim_localizer.yaml
```

### Terminal 3 — Set Initial Pose
TODO: use Rviz2 tool to set initial pose.
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    stamp: {sec: 16, nanosec: 360000000},
    frame_id: 'odom'
  },
  pose: {
    pose: {
      position: {
        x: 2.000000000423794,
        y: 2.0,
        z: 0.7500389896494932
      },
      orientation: {
        x: 1.617940655980588e-18,
        y: 2.638455507896242e-10,
        z: 4.989162198923731e-21,
        w: 1.0
      }
    },
    covariance: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ]
  }
}"
```

Now, you can use Rviz2 to publish a goal pose. The dummy planner computes a twist toward the goal.  Verify with `ros2 topic echo /cmd_vel`.

## How It Works

`FineNavEngine` subscribes to `/goal_pose` and triggers `navigateTo()`.  The BT ID is set in `main.cpp`:

```cpp
engine->setDefaultBehaviorTree("GettingStartedNavigate");
```

The behavior tree (`behavior_trees/navigate_to_pose.xml`) wires the example's
`layer="my_layer"` and `planner_id="dummy_planner"` via TaskChannel IPC.
Navigator reads feedback from the layer and publishes `/cmd_vel`.

## Key Parameters (`sim_localizer.yaml`)

| Section | Parameter | Purpose |
|---|---|---|
| `hello_finenav/finenav_localizer` | `odom0: "/odom"` | Subscribe to simulator ground truth |
| | `publish_tf: true` | Broadcast `map→base_link` |
| | `force_2d_mode: true` | Planar constraint |
| `hello_finenav/finenav_navigator` | `behavior_trees` | Load the example's BT XML |

## Troubleshooting

| Symptom | Check |
|---|---|
| Robot doesn't move | `ros2 topic echo /cmd_vel` |
| No TF | Ensure `publish_tf: true` in params |
| "behavior tree ID is empty" | `setDefaultBehaviorTree()` was not called |
| "Failed to create tree" | Ensure `behavior_trees` includes the example's BT directory |
