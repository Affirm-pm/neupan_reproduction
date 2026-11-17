# FineNav 官方全局规划器 + NeuPAN

控制链：`导航目标 → global_layer (官方 A*，1 Hz) → my_layer (NeuPAN，10 Hz) → 底盘速度`。
沿用 NeuPAN 的 `Planner + WorldView` 插件接入方式，通过 `createControlLayer` 创建全局层，使用 `registerControlBranch` 将两层串联。

## 来源与适配

官方依赖为 [FineNav/finenav_planner](https://github.com/FineNav/finenav_planner)
的 `finenav_route_planner`，固定提交
`810e427e2a546029c7025c128ce9203f4636dbf3`，源码原样随工作区提供。
`GlobalPlanner` 调用其中的 `AstarPathSearch`；补齐精确起终点、目标朝向和二维高度。
使用四邻接和零目标容差，每秒从当前位姿重新搜索。路径两端使用精确位姿，连接段在膨胀地图上检查。NeuPAN 根据距终点的距离减速，并在同一目标的多次重规划之间保留终点对齐与停滞检测状态。
NeuPAN 还会根据前方约 0.5 m 路径的航向偏差降低参考速度，避免差速底盘以巡航速度
切入急转弯内侧。导航行为树最终失败时，导航器显式发布零速度。

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
默认仿真配置同时让全局层订阅 `/cloud_for_lio`：每轮重规划将最新、有效的点云
临时叠加到静态地图，再统一膨胀与搜索。新放入箱子后，A* 可以生成绕行路径，
NeuPAN 负责执行和近距离避障。原始 `/global_map` 不被修改；临时障碍格保留 3 秒，
防止单帧漏检或短暂遮挡导致路径突然穿回箱子。持续不再观测到的障碍物会在超时后
清除。该叠加不提供长期历史建图或运动预测。
仿真 `OdometryPublisher` 配置 `xyz_offset = (0, 0, -0.75)`，将模型原点转换到
`base_link`；仅设置 `robot_base_frame` 不会转换位姿。缺少此偏移会把地面点抬高
0.75 m，绕障通道可能被虚假障碍封闭。
对应行为可见 [Gazebo 6 OdometryPublisher 源码](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/odometry_publisher/OdometryPublisher.cc)。
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
- `global_grid_world_view.use_live_obstacles`：插件默认关闭，`neupan_sim.yaml` 中开启。
- `global_grid_world_view.obstacle_persistence`：临时障碍格保留时间，默认 3 s。
- `global_grid_world_view.obstacles.*`：复用 `neupan_world_view` 的点云/激光、TF、
  高度、距离及超时参数。更换局部点云源时也要修改这里。全局层使用完整碰撞点集，
  不受优化器 `max_points`/`scan_downsample` 截断影响。实时观测无效时不生成通行路径。
- `finenav_global_planner.max_planning_time`：默认 0.8 s。
- `finenav_global_planner.max_iterations`：默认 200000。

膨胀还包含栅格面积的保守补偿，狭窄通道可能判为不可通行。
实时模式下，起点周围 0.8 m 内的额外栅格补偿区会按查询点到静态占用格矩形、
实时点边界的实际距离复核，仍要求完整 `inflation_radius` 间距，并为连接段采样
留出余量。这样车体安全但刚进入栅格补偿区时，可以退出和绕行；不会清空障碍格。
该适配仅用于平面 SE(2) 导航，不支持 FineNav 三维地形通行性地图。

## 验证

```bash
colcon test --packages-select finenav_route_planner finenav_example
colcon test-result --verbose
/usr/bin/python3 src/FineNav-Engine/finenav_example/create_custom_global_planner/test/ros_smoke.py
```

C++ 测试覆盖绕墙、封闭无解、精确终点和朝向、未知区域、膨胀、地图替换、错误坐标系及无效输入。NeuPAN 回归还覆盖连续重规划下的停滞检测、终点减速、仅更改终点朝向和静态 TF 接收。

`ros_smoke.py` 在 ROS 域 117 中启动引擎，输入合成地图、里程计和点云。
它检查绕障路径和速度输出，然后保持地图和目标不变，仅在点云中加入/移除障碍物，
验证路径随之绕行/恢复；最后封闭通道并检查停车命令。

无界面 Gazebo 闭环测试使用仿真的里程计和激光雷达，在 ROS 域 119 启动上面的默认双层规划配置。每次运行会将里程计轨迹、全局路径和目标结果写入 `trace.json`，将引擎与仿真日志写入 `launch.log`。

```bash
/usr/bin/python3 src/FineNav-Engine/finenav_example/create_custom_global_planner/test/gazebo_navigation.py
```

向正在执行的路径插入真实 Gazebo 箱子，验证重规划、绕行、停车及最终到达：

```bash
/usr/bin/python3 src/FineNav-Engine/finenav_example/create_custom_global_planner/test/gazebo_navigation.py --live-obstacle
```

测试从默认出生点附近先驶向 `(2, 3, π/2)`，随后从墙左端绕行至 `(5, 6, 0)`。每个目标均要求导航动作成功，停车后位置误差小于 0.25 m、航向误差小于 0.1 rad。脚本同时检查停车命令、里程计速度和到达目标的全局路径。

2026-11-17 在默认迷宫连续运行两轮，四次导航动作均成功，停车后里程计线速度和角速度均为零。两轮的绕墙目标位置误差分别为 0.106 m 和 0.139 m。官方规划包 57 个用例、示例包 52 个用例及封闭地图停车测试均通过。

同日新增实时障碍物回归：向目标 `(6, 2, 0)` 的执行中路径插入位于 `(3.8, 2)` 的
0.6 × 0.6 × 1 m Gazebo 箱子，小车绕行并导航成功，位置误差 0.181 m、航向误差
0.091 rad，停车后线速度和角速度均为零。修复后的普通目标及绕墙目标也成功；
示例包 58 个 C++ 用例通过。
