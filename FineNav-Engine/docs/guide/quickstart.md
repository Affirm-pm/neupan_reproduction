# 1. Run the example

Complete [Installation](installation.md), then use one launch file to start the
simulator, navigation application, and RViz.

## Launch

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

Press **Play** in Gazebo.

## Send a goal

Use RViz's **2D Goal Pose** to select a nearby goal. No manual initial pose is needed.
Choose a short unobstructed straight path and observe the robot moving.

## Launch arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `world_name` | `flat_small_scale_maze.world` | Simulator world. |
| `params_file` | `config/hello_finenav.yaml` | Navigation application parameters. |
| `use_rviz` | `true` | Whether to start RViz. |

## Common issues

| Symptom | First check |
| --- | --- |
| Package or executable not found | Confirm the build succeeded and `install/setup.bash` was sourced. |
| Robot remains still | Confirm Gazebo is playing and a goal has been sent from RViz. |

The example source code and its parameter configuration are in
[hello_finenav.cpp](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/getting_started/hello_finenav.cpp) and
[hello_finenav.yaml](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/config/hello_finenav.yaml).

The following tutorials explain in detail how to build the example project from scratch:

- [2. Choose Map and Planner](choose-algorithms.md)
- [3. Write a WorldView](worldview.md)
- [4. Assemble a control pipeline](cpp-application.md)
- [5. Configure a Behavior Tree](behavior-tree.md)
