## Clone

```bash
mkdir -p ~/neupan_reproduction_ws/src
cd ~/neupan_reproduction_ws/src
git clone --branch lr https://github.com/Affirm-pm/neupan_reproduction.git .
```

## Build

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-colcon-common-extensions python3-rosdep libbenchmark-dev
cd ~/neupan_reproduction_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to finenav_example
```

## Run(sim)

```bash
cd ~/neupan_reproduction_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

默认使用 FineNav 官方 A* 全局规划器与 NeuPAN 局部规划器。
接入结构、外部地图配置及局部模式切换见
[全局规划器说明](FineNav-Engine/finenav_example/create_custom_global_planner/README.md)。
