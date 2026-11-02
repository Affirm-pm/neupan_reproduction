# 1. 运行示例

完成[如何安装](installation.md)后，通过一个 launch 文件启动仿真、导航应用和 RViz。

## 启动

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

启动后，在 Gazebo 中点击 **Play**。

## 发送目标

使用 RViz 的 **2D Goal Pose** 在机器人附近设置目标点，无需手动设置初始位姿。
选择一段无遮挡的直线路径，观察机器人运动。

## 启动参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `world_name` | `flat_small_scale_maze.world` | 仿真场景。 |
| `params_file` | `config/hello_finenav.yaml` | 导航应用配置文件。 |
| `use_rviz` | `true` | 是否启动 RViz。 |

## 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 找不到包或可执行文件 | 确认编译成功，并已加载 `install/setup.bash`。 |
| 机器人不动 | 确认 Gazebo 已点击 **Play**，并已在 RViz 中发送目标。 |

示例的源代码以及相应的参数配置在
[hello_finenav.cpp](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/getting_started/hello_finenav.cpp) 和
[hello_finenav.yaml](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/config/hello_finenav.yaml) 中。

后续教程将详细介绍如何从 0 开始构造示例项目：

- [2. 选择 Map 和 Planner](choose-algorithms.md)
- [3. 编写 WorldView](worldview.md)
- [4. 组装控制 Pipeline](cpp-application.md)
- [5. 按需求配置行为树](behavior-tree.md)
