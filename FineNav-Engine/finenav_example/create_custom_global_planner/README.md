# FineNav 官方全局规划器 + NeuPAN

控制链：`导航目标 → global_layer (官方 A*，1 Hz) → my_layer (NeuPAN，10 Hz) → 底盘速度`。
沿用 NeuPAN 的 `Planner + WorldView` 插件接入方式，通过 `createControlLayer` 创建全局层，使用 `registerControlBranch` 将两层串联。

## 来源与适配

官方依赖为 [FineNav/finenav_planner](https://github.com/FineNav/finenav_planner)
的 `finenav_route_planner`，固定提交
`810e427e2a546029c7025c128ce9203f4636dbf3`，源码原样随工作区提供。
`GlobalPlanner` 调用其中的 `AstarPathSearch`；补齐精确起终点、目标朝向和二维高度。
使用四邻接和零目标容差，每秒从当前位姿重新搜索。路径两端使用精确位姿，连接段在膨胀地图上检查。NeuPAN 根据距终点的距离减速，并在同一目标的多次重规划之间保留终点对齐与停滞检测状态。

`GlobalGridWorldView` 与 NeuPAN WorldView 一样自行订阅输入，回调只缓存消息，
计算在规划线程中执行。DummyMap 仅满足注册接口，不作为障碍物来源。
地图更新后重新计算膨胀；无地图、无效地图、起终点阻塞或无路可走返回规划失败，
交给现有控制链及行为树停止和恢复，不退回穿墙直线。

## 编译、启动

```bash
cd ~/neupan_reproduction_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:/opt/ros/humble/bin CMAKE_BUILD_PARALLEL_LEVEL=2 \
  colcon build --packages-up-to finenav_example \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON \
  -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
  -Dtl-expected_DIR=/usr/share/cmake/tl-expected
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

在 Gazebo 按 Play，等地图和点云就绪，再通过 RViz 的 2D Goal Pose 下发目标。
RViz 中绿色是全局路径 `/finenav_global_path`，蓝色是 NeuPAN 参考路径，另有 MPC 预测轨迹。

默认地图来自 `flat_small_scale_maze.world` 及其嵌套 SDF 模型的碰撞盒，
以 0.1 m 分辨率发布 `/global_map`。这是仿真已知地图，不是在线 SLAM；
动态障碍仍由 NeuPAN 点云避障处理，不会自动写入这张静态地图。
地图发布器只支持该平面迷宫，其他世界需要外部地图。

保持原有仅局部规划模式：

```bash
ros2 launch finenav_example hello_finenav.launch.py use_global_planner:=false
```

接入外部地图：

```bash
ros2 launch finenav_example hello_finenav.launch.py \
  publish_global_map:=false global_map_topic:=/map
```

输入类型为 `nav_msgs/OccupancyGrid`，QoS 为 Reliable + Transient Local。
地图必须处于 FineNav 世界坐标系（默认 `map`），原点姿态无旋转；不做隐式 TF。
未知区域和地图外部均不可通行。地图分辨率支持 0.01–2 m，最多 1000 万格。
按静态地图语义保留最后一次消息，发布者退出不会使地图过期。

参数节点为 `/hello_finenav/global_layer`：

- `global_grid_world_view.map_topic`：地图话题，启动时设置。
- `global_grid_world_view.inflation_radius`：默认 0.51 m，覆盖 0.7 × 0.6 m 机器人外接圆和余量。
- `global_grid_world_view.clearance_cost_distance` 默认 0.7 m。在膨胀边界外的这段距离内，越靠墙的可通行栅格代价越高，给底盘转弯留出空间。
- `global_grid_world_view.occupied_threshold`：默认 50；未知值始终阻塞。
- `finenav_global_planner.max_planning_time`：默认 0.8 s。
- `finenav_global_planner.max_iterations`：默认 200000。

膨胀还包含栅格面积的保守补偿，狭窄通道可能判为不可通行。
该适配仅用于平面 SE(2) 导航，不支持 FineNav 三维地形通行性地图。

## 验证

```bash
colcon test --packages-select finenav_route_planner finenav_example
colcon test-result --verbose
/usr/bin/python3 src/FineNav-Engine/finenav_example/create_custom_global_planner/test/ros_smoke.py
```

C++ 测试覆盖绕墙、封闭无解、精确终点和朝向、未知区域、膨胀、地图替换、错误坐标系及无效输入。NeuPAN 回归还覆盖连续重规划下的停滞检测、终点减速、仅更改终点朝向和静态 TF 接收。

`ros_smoke.py` 在 ROS 域 117 中启动引擎，输入合成地图、里程计和点云。它先检查绕障路径和速度输出，再封闭通道并检查停车命令。

无界面 Gazebo 闭环测试使用仿真的里程计和激光雷达，在 ROS 域 119 启动上面的默认双层规划配置。每次运行会将里程计轨迹、全局路径和目标结果写入 `trace.json`，将引擎与仿真日志写入 `launch.log`。

```bash
/usr/bin/python3 src/FineNav-Engine/finenav_example/create_custom_global_planner/test/gazebo_navigation.py
```

测试从默认出生点附近先驶向 `(2, 3, π/2)`，随后从墙左端绕行至 `(5, 6, 0)`。每个目标均要求导航动作成功，停车后位置误差小于 0.25 m、航向误差小于 0.1 rad。脚本同时检查停车命令、里程计速度和到达目标的全局路径。

2026-11-17 在默认迷宫连续运行两轮，四次导航动作均成功，停车后里程计线速度和角速度均为零。两轮的绕墙目标位置误差分别为 0.106 m 和 0.139 m。官方规划包 57 个用例、示例包 52 个用例及封闭地图停车测试均通过。
