# FineNav2D

## How to use

1. Clone the repository
   
```shell
mkdir finenav_ws && cd finenav_ws/
git clone https://github.com/FineNav/FineNav-Engine.git
git switch dev
```

2. Build the project

```shell
rosdep install --from-paths src --ignore-src -r
colcon build --symlink-install
```

3. Launch the example
```shell
source install/setup.bash
ros2 run finenav_example hello_finenav --ros-args --params-file ./src/FineNav-Engine/finenav_example/getting_started/params/sim_localizer.yaml

# Another terminal
ros2 launch finenav_simulator finenav_simulator.launch.py \
  lio_type:=static \
  use_sim_time:=true \
  world_name:=flat_small_scale_maze.world \
  publish_map_tf:=true
  
# Press the start button, and use Rviz to visualize and navigate the robot
```
