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
source /home/affirm/finenav_ws/install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```
