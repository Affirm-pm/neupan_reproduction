# 如何安装

配置工作空间，安装依赖并编译 FineNav-Engine。

## 1. 配置 Ubuntu 和 ROS 2

使用 **Ubuntu 22.04 + ROS 2 Humble**。
按照 [ROS 2 官方安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
安装 Humble Desktop。以下命令均在 Ubuntu 终端执行。

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
```

## 2. 克隆仓库

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone https://github.com/FineNav/FineNav-Engine.git src/FineNav-Engine
```

## 3. 安装依赖

```bash
sudo rosdep init  # 已初始化的机器跳过此行
rosdep update

cd ~/finenav_ws
source /opt/ros/humble/setup.bash
sudo apt install -y ros-humble-xacro python3-yaml
rosdep install --from-paths src/FineNav-Engine --ignore-src --rosdistro humble -y
```

## 4. 编译

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

启动示例、仿真配置与目标发送见 [1. 运行示例](quickstart.md)。
