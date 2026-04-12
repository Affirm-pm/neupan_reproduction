# first start
cd ***_ws //切换到你的工作空间
colcon build --packages-select finenav_simulator
source install/setup.bash

# for real_sim
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
source install/setup.bash
export DISPLAY=:0

ros2 launch finenav_simulator finenav_simulator.launch.py \
  lio_type:=static \
  use_sim_time:=true \
  world_name:=flat_small_scale_maze.world \
  publish_map_tf:=true

# lio_type
lio_type:=static 纯传感器模式
lio_type:=fast_lio fast_lio模式

# 以下是可选的world，将参数替换为world_name，添加额外的world记得调整导航车出生点
https://github.com/ZJU-FAST-Lab/uneven_planner //不平坦地形
map_desert.world
map_hill.world
map_vocano.world

https://ieeexplore.ieee.org/abstract/document/9636611 //小迷宫地图
flat_small_scale_maze.world

# 遥控操作指南
W
前进，线速度设为 +linear_speed
S
后退，线速度设为 -linear_speed
A
左转，角速度设为 +angular_speed
D
右转，角速度设为 -angular_speed
X
线速度和角速度都清零
Z
增大线速度档位，默认每次 +0.1
C
减小角速度档位，默认每次 -0.1
Q
增大角速度档位，默认每次 +0.1
E
减小角速度档位，默认每次 -0.1





