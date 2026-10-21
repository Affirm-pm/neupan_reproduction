# 原 FineNav 仿真接入 NeuPAN

本目录是 `finenav_example` 的适配配置与工具。启动器包含原 `finenav_simulator/launch/complete/finenav_simulator.launch.py`，使用原 `fine_robot`、`flat_small_scale_maze.world`、三维 GPU 激光、里程计、点云转换器和 TF。没有新建仿真 ROS 包、机器人模型或世界。

## 构建和启动

依赖：ROS 2 Humble、原 FineNav 依赖、Gazebo Fortress（Sim 6）、`ros_gz_sim`、`ros_gz_bridge`、`robot_state_publisher`、`sensor_msgs_py`、`tf2_ros_py`、NumPy 和 RViz。运行无需 PyTorch；重训才需要 PyTorch。

在 FineNav 根目录执行：

```bash
bash tools/build_gazebo.sh
bash tools/run_gazebo.sh
```

RViz 默认使用可旋转的三维 Orbit 视角。Gazebo 自动开始运行，机器人在原迷宫 `(2,2)` 出生。RViz 的 **2D Goal Pose** 对应 `/goal_pose`；先尝试 `(4,2)`，拖动箭头指定最终朝向。不需要启动 `fins_localization`、`fins_driver` 或 FAST-LIO，也不需要手动设置初始定位。

无窗口验证：

```bash
bash tools/run_gazebo.sh gui:=false rviz:=false
```

等价 ROS 入口（已构建、在 FineNav 根目录）：

```bash
source finenav_ws/install/setup.bash
export ROS_DOMAIN_ID=188
export ROS_LOCALHOST_ONLY=1
ros2 launch finenav_example neupan_gazebo.launch.py
```

运行脚本默认 ROS 域为 188，手动另开的 RViz、目标发布器和测试终端需要相同的域。默认只使用本机 ROS 通信。`world_name` 可选择原仿真已有且配置了出生点的世界，但当前适配和验证针对平地小迷宫；下述墙体检查脚本也只适用这个世界。

## 数据适配

- 原激光链：Gazebo `/lidar/points` → `/livox/lidar` → 原转换器 `/cloud_for_lio`（`base_link`）→ 按点云时间戳查询 TF → `/aligned_cloud`（`map`）。保留真实激光扫描，过滤车体自身回波；NeuPAN 世界视图再按高度筛除地面和高处点。
- 原定位链：Gazebo `/ground_truth_odometry` → `/odom` → `/localization_pose` 和 `/simulation/measured_twist`。使用 Gazebo 真值，未验证 SLAM。
- 原 SDF 的模型原点 `base_origin` 位于地面 `base_link` 上方 0.75 m。此启动配置让原 helper 发布 `odom → base_origin`，再补 `base_origin → base_link` 的 -0.75 m 静态变换；原状态发布器提供其余关节 TF。这样地面点仍在约 z=0，而不是被抬高成障碍。
- FineNav/NeuPAN `/neupan/cmd_vel` → 适配器 → 原桥接器 `/cmd_vel` → 原差速驱动。适配器使用墙上时钟检查命令、点云和里程计超时，数据失效即发零速；此入口禁用 Gazebo 键盘遥控，避免同时控制。

给原 `launch/core/gazebo.launch.py` 和 `launch/core/robot.launch.py` 增加可选启动参数（自动运行/无窗口、键盘开关、真值 TF 子坐标系）；直接使用原示例时默认行为不变。同时修正原仿真 SDF→URDF 转换器丢失 `file://` 网格路径与缩放的问题，确保 RViz 能显示原雷达支架。导航引擎框架、原世界与机器人文件均未修改。

## 统一车型和训练模型

原车体长 0.6 m、宽 0.4 m；加轮子后包络约 0.7 × 0.6 m。`../planner.yaml` 与 `../models/diff_0p7_0p6.bin` 成对使用，训练参数、权重、导出一致性样本和误差报告在 `../models/`。0.6 m 高度是仿真筛选与包络配置，神经网络仅学习平面轮廓。项目所有 NeuPAN 入口统一使用上述模型；不再使用之前暂定的 0.4 × 0.4 m 模型。

如修改原机器人几何，需要重新核对包络、`base_origin` 外参、自身回波过滤及高度范围，并重新训练模型。在 FineNav 根目录、已安装 NumPy/PyTorch 的 Python 环境中：

```bash
python3 finenav_ws/src/FineNav-Engine/finenav_example/create_custom_neupan_planner/tools/train_rectangle.py \
  --length 0.7 --width 0.6 --height 0.6 --model-name diff_0p7_0p6.bin \
  --output finenav_ws/src/FineNav-Engine/finenav_example/create_custom_neupan_planner/models
```

重新构建后生效。仿真参数文件使用 launch 路径替换，应通过上述 launch 入口启动。

## 验证

仿真启动后，在另一终端、FineNav 根目录执行：

```bash
source finenav_ws/install/setup.bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
python3 finenav_ws/src/FineNav-Engine/finenav_example/create_custom_neupan_planner/gazebo/test_navigation.py
```

脚本向原迷宫中的 `(4,2, yaw=0.5)` 发目标，检查 Action 成功、实际里程计误差、真实激光与转换点云、最终零速，以及机器人包络对原迷宫墙体至少 5 cm 的几何净空。此项净空检查并非 Gazebo 接触传感器的全量碰撞验证。

到点后可继续验证墙内目标：

```bash
python3 finenav_ws/src/FineNav-Engine/finenav_example/create_custom_neupan_planner/gazebo/test_navigation.py \
  --blocked --x 4 --y 4 --yaw 1.5708 --output /tmp/finenav-original-wall.json
```

观察 15 秒后取消：不能误报到达墙内目标，不能侵入墙体净空，取消后需停车。本次实测停车与净空检查通过，但引擎取消返回 `ABORTED (6)`，而不是 `CANCELED (5)`，因此该脚本会如实返回失败并记录 `cancel_contract_passed: false`；详见 `VALIDATION.md`。结果默认写入 `/tmp`，本次实测记录见 `test/`。

这证明原场景中局部导航与安全停车链路可运行，不代表具备全局路径规划或可以自主通过整个迷宫。复杂墙体与死胡同可能造成局部极小。导航成功后 Ctrl+C 的既有 Action Server 退出异常仍未改动引擎框架修复；定位器也可能输出状态历史时间戳告警，需与团队独立跟进。
