# Quickstart

Download FineNav-Engine, build it in a ROS 2 workspace, and run the included
navigation example.

!!! note "Example scope"
    The example uses `dummy_map`, `dummy_planner`, and `dummy_world_view`.
    It demonstrates navigation setup; the dummy components do not provide obstacle avoidance.

## 1. Prepare your environment

Use **Ubuntu 22.04 with ROS 2 Humble**. This is the environment used by the
engine's CI. If ROS 2 is not installed, follow the
[ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
and choose the desktop installation.

Install the development tools:

```bash
sudo apt update
sudo apt install -y git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
```

## 2. Download the source

Create a workspace and clone all three repositories into `src`. Use the
engine's `dev` branch for these examples; the map and planner repositories
use their default branches:

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone --branch dev https://github.com/FineNav/FineNav-Engine.git \
  src/FineNav-Engine
git clone https://github.com/FineNav/finenav_map.git src/finenav_map
git clone https://github.com/FineNav/finenav_planner.git src/finenav_planner
```

The workspace should now contain:

```text
finenav_ws/
└── src/
    ├── FineNav-Engine/
    ├── finenav_map/
    └── finenav_planner/
```

If GitHub requests authentication, use an account with access to the repository.
Keep subsequent dependency and build commands in the workspace root,
`~/finenav_ws`.

## 3. Install dependencies

Initialize rosdep once on your machine. Skip `sudo rosdep init` if it has
already been initialized:

```bash
sudo rosdep init
rosdep update
```

Install the package dependencies:

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

For the simulation example, also install Gazebo integration, RViz, and the
launch-time Python dependencies:

```bash
sudo apt install -y ros-humble-ros-gz ros-humble-rviz2 \
  ros-humble-xacro python3-yaml
```

The simulator uses Gazebo Sim 6 (Fortress) and needs a graphical desktop.
The example below uses Gazebo odometry, so a separate Fast-LIO installation
is not required.

## 4. Build and load the workspace

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

After the build succeeds, check that ROS 2 can find the example executable:

```bash
ros2 pkg executables finenav_example
```

The output should include `finenav_example hello_finenav`.
Source `install/setup.bash` in each new terminal before using the built packages.

## 5. Start the simulator

In the first terminal:

```bash
cd ~/finenav_ws
source install/setup.bash
ros2 launch finenav_simulator finenav_simulator.launch.py \
  lio_type:=gazebo_odom use_sim_time:=true \
  world_name:=flat_small_scale_maze.world publish_map_tf:=true
```

Press **Play** in Gazebo to start the simulation clock.

## 6. Start FineNav

In a second terminal:

```bash
cd ~/finenav_ws
source install/setup.bash
ros2 run finenav_example hello_finenav --ros-args \
  --params-file src/FineNav-Engine/finenav_example/getting_started/params/sim_localizer.yaml
```

Wait for `FineNav-Engine ready.` and for the simulator to publish odometry.

## 7. Send a goal

Open RViz in a third terminal:

```bash
cd ~/finenav_ws
source install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

Set RViz's **Fixed Frame** to `map`. Use **2D Goal Pose** to send a goal on
`/goal_pose`; select an unobstructed goal for this dummy example.

## Next: use your own components

The example application loads plugins, creates a layer from `dummy_world_view`,
and registers the `navigate` branch. To use your components, write a WorldView
and replace its plugin path and registered name in that application.

Continue with [Choose Map and Planner](choose-algorithms.md).
The complete [example application](https://github.com/FineNav/FineNav-Engine/blob/81c8f277518fb9229eb5e78be43375227490ce39/finenav_example/getting_started/main.cpp)
and [simulation configuration](https://github.com/FineNav/FineNav-Engine/blob/81c8f277518fb9229eb5e78be43375227490ce39/finenav_example/getting_started/params/sim_localizer.yaml)
are useful starting points.
