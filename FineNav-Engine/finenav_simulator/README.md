# finenav_simulator

**Version: 0.5.0**

Gazebo Sim resources, ROS bridges, and helper nodes for FineNav-Engine simulation,
including robot spawning, point-cloud conversion, teleoperation, and odometry.

## Run

Use Ubuntu 22.04, ROS 2 Humble, and Gazebo Sim / Fortress with a graphical desktop.
After [building the workspace](../README.md#build), run:

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
ros2 launch finenav_simulator finenav_simulator.launch.py lio_type:=gazebo_odom use_sim_time:=true world_name:=flat_small_scale_maze.world
```

The launch entry point is
[launch/complete/finenav_simulator.launch.py](launch/complete/finenav_simulator.launch.py).
Gazebo odometry supplies `/odom`; helper nodes publish the corresponding TF and
an identity `map` to `odom` transform.

## Localization modes

| `lio_type` | Behavior |
| --- | --- |
| `gazebo_odom` | Use Gazebo odometry without an external SLAM package. Recommended for the example. |
| `fast_lio` | Start the external `fast_lio` package; also start `octomap_server` unless disabled in the localization launch configuration. |

The launch default is `fast_lio`, so select `gazebo_odom` explicitly when those
external packages are not installed. Legacy values `static` and `ground_truth`
currently follow the Gazebo odometry path; `static` is not a sensors-only mode.

The `publish_map_tf` argument controls the optional static `map` to `odom`
transform in the FAST-LIO path. It is not needed for `gazebo_odom`.

## Nodes and configuration

- `point_cloud_converter_node`: point-cloud conversion.
- `simulator_helper_node`: keyboard teleoperation, odometry path, and TF helpers.
- [ros_gz_bridge.yaml](config/ros_gz_bridge.yaml): Gazebo-to-ROS topic bridges.
- [world_spawn_poses.yaml](config/world_spawn_poses.yaml): robot spawn pose per world.
- [fastlio_gazebo_config.yaml](config/fastlio_gazebo_config.yaml): FAST-LIO parameters.

## Teleoperation

The helper reads key codes from `/keyboard/keypress` (`std_msgs/Int32`) and
publishes `/cmd_vel`. Keys must be sent by a keyboard event source; typing into
the launch terminal alone does not publish them.

| Key | Action |
| --- | --- |
| W / S | Forward / backward. |
| A / D | Turn left / right. |
| X | Stop. |
| Z / C | Increase / decrease linear speed by 0.1 m/s. |
| Q / E | Increase / decrease angular speed by 0.1 rad/s. |

Initial speeds are 0.5 m/s and 1.0 rad/s. Use one command source at a time when
running teleoperation and navigation.

## Worlds

Set `world_name` to a file under `resource/worlds/` and configure its spawn pose
in `world_spawn_poses.yaml` when adding a world.

- `map_desert.world`, `map_hill.world`, `map_vocano.world`: terrain resources
  referenced from [uneven_planner](https://github.com/ZJU-FAST-Lab/uneven_planner).
- `flat_small_scale_maze.world`: maze resource; see the
  [referenced publication](https://ieeexplore.ieee.org/abstract/document/9636611).

Continue with the [navigation example](../finenav_example/getting_started/README.md).
