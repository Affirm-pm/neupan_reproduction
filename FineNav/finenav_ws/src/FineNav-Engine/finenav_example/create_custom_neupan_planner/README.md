# NeuPAN 接入 FineNav-Engine

本扩展使用现有 `FINENAV_REGISTER_MAP / PLANNER / WORLDVIEW` 接口，在引擎进程内调用 `neupan_cpp/libneupan` 的 DUNE + NRMP；运行时不需要 Python、PyTorch 或独立 NeuPAN ROS 节点。

已有文件的修改仅在 `finenav_example/CMakeLists.txt`、`package.xml` 和 `getting_started/main.cpp`。导航引擎框架、`fins_driver`、`fins_localization` 均未修改。默认不传 NeuPAN 配置时仍选择原 Dummy 示例。

## 构建与启动

整包包含并列的 `finenav_ws/` 和 `neupan_cpp_ws/`。请一同传输，使用根目录的 `tools/package_project.py` 可生成排除编译缓存的源码压缩包；迁移步骤见整包根目录 `README.md`。

本机工作区实际路径是 `~/FineNav/finenav_ws`，对应需求中的 `FineNav-Engine_ws`。

```bash
cd ~/FineNav/finenav_ws
bash src/FineNav-Engine/finenav_example/create_custom_neupan_planner/tools/build.sh
source install/setup.bash
ros2 run finenav_example hello_finenav --ros-args \
  --params-file src/FineNav-Engine/finenav_example/getting_started/params/sim_localizer.yaml \
  --params-file src/FineNav-Engine/finenav_example/create_custom_neupan_planner/neupan_sim.yaml
```

`neupan_sim.yaml` 按需求保留文件名，内容是**实车输入配置**：后加载，覆盖第一个文件的 `use_sim_time: true` 为 `false`，禁用 `/odom`，启用绝对定位 `/localization_pose`。仅启动第 2 步时可以正常加载插件并等待传感器；没有真实定位和点云时不产生运动指令。

构建脚本使用 ROS 2 Humble、Eigen 3.4、yaml-cpp、已有 FineNav 依赖，以及本整包内的 `FineNav/neupan_cpp_ws/src/neupan_cpp/libneupan`。OSQP / osqp-eigen / QDLDL 的本机源码快照位于工作区 `neupan_dependencies/src`，离线编译，不向系统目录安装。可用环境变量 `NEUPAN_SOURCE_DIR` 指定其他 libneupan 目录。两个求解器动态库随示例安装，启动只需 source 本工作区。

实车的第 1、3 步保持原指令。RViz 的 Fixed Frame 设为 `map`，添加 `/aligned_cloud`、`/global_map_viz` 和 `/hello_finenav/finenav_localizer/nav_state`（实际话题可用 `ros2 topic list` 核对），通过 **2D Goal Pose** 向 `/goal_pose` 发目标。引擎不重复广播定位程序提供的 TF。

## 数据与算法接口

| 输入/输出 | 类型 | 约定 |
|---|---|---|
| `/localization_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 绝对位姿，`frame_id=map`，持续更新 |
| `/aligned_cloud` | `sensor_msgs/PointCloud2` | 已对齐到 map 的当前局部点云，float32 XYZ，小端格式 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 发 map 坐标目标，包含终点朝向 |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x`、`angular.z`，对齐现有串口驱动 |

`NeuPANCloudMap` 通过 MapRuntime 注册点云订阅；`NeuPANWorldView` 在框架读锁内裁剪高度、距离并作 XY 体素降采样；`NeuPANPlanner` 将 robot_state 和参考 poses 转换为 NeuPAN 状态与路径，再交回 Twist 给原控制链发布。支持多点参考轨迹；目标变化和恢复传感器后清除旧的优化状态。

默认是差速运动学（该 C++ 移植当前支持的类型）。即便底盘能横移，本接入也只输出前进/倒退与角速度。默认控制周期 0.1 s，预测 20 步，参考速度 0.25 m/s，速度上限 0.3 m/s、0.6 rad/s，加速度上限 0.5 m/s²、1.0 rad/s²。首个输出也按测得速度限加速度；紧急停车直接输出零。

新插件在无输入、数据超过 0.5 s、坐标帧错误、空/全无效点云、非法位姿、求解失败或碰撞检查不通过时输出零速度。消息时间戳和本机单调时钟均用于超时检查。一个有效点云经过地面/高度过滤后没有障碍，允许按空闲局部空间规划；原始空点云不会被当作“没有障碍”。

插件明确要求定位程序已对齐的 `map` 点云，不猜测或替换 frame_id。其他雷达原始坐标系需要先正确转换后再接入。

## 尺寸、模型和参数

暂定车体长宽高 **0.4 × 0.4 × 0.5 m**，平面矩形以运动参考点为中心，wheelbase=0。DUNE 训练使用 0.4 × 0.4 m 的二维投影；高度用于点云过滤，不是二维网络的输入。

`models/rectangle.pth` 是训练权重；`models/diff_0p4.bin` 是 C++ NPTF 模型，包含 G/h 足迹元数据。加载时 libneupan 校验模型和规划器尺寸一致，尺寸不匹配直接拒绝启动。`training_report.json` 记录训练配置、验证误差和模型哈希。

训练使用原 NeuPAN `ObsPointNet` 架构，并针对矩形用解析投影计算原 DUNE 对偶问题的最优标签；不改变运行时的 DUNE/NRMP 算法。样本兼顾近车体区域和 ±6 m 范围，固定随机种子，训练 16000 次。5 万个留出点上，近场（离矩形 <0.3 m）距离误差 MAE 约 0.166 mm、P99 约 0.819 mm。这是模型几何误差，不是实车定位或避障精度。

重训示例（在安装了 NumPy、PyTorch 的 Python 环境中执行）：

```bash
python3 \
  src/FineNav-Engine/finenav_example/create_custom_neupan_planner/tools/train_rectangle.py \
  --length 0.4 --width 0.4 --steps 16000
```

训练脚本默认使用包内保留版权声明的原 `ObsPointNet` 架构副本和包内 C++ 导出工具，不再要求外部 `~/NeuPAN` 或 `~/neupan_cpp_ws`。如需指定其他原版源码，可用 `--upstream /path/to/NeuPAN`、`--cpp /path/to/libneupan`。

得到真实参数后：

1. 更新 `planner.yaml` 的 robot.length/width/wheelbase、动力学上限及必要的优化权重，并用相同几何重训。当前训练脚本适用于中心对称矩形、wheelbase=0；偏置参考点或非矩形应使用原 NeuPAN 训练流程及匹配的 G/h 导出。
2. 更新 WorldView 的高度过滤范围，以及 `neupan_planner.robot_height`（用于参数记录/日志）。默认高度范围是定位原始 base 位姿之上 0.05–0.5 m，假定 base 原点位于地面高度；真实传感器安装与 base 原点高度必须据实调整。不会自动删除车体内部点，以免把真实侵入障碍误删。
3. 重新编译安装，再做实车验证。长度/宽度变化必须同步模型；只改 YAML 不会绕过尺寸检查。

参数可在第二个启动配置中追加，例如：

```yaml
/hello_finenav/my_layer:
  ros__parameters:
    neupan_planner:
      config_file: /absolute/path/planner.yaml
      model_file: /absolute/path/model.bin
      robot_height: 0.5
      goal_xy_tolerance: 0.08
      goal_yaw_tolerance: 0.08
      collision_margin: 0.05
    neupan_world_view:
      pose_topic: /localization_pose
      data_timeout: 0.5
      min_height: 0.05
      max_height: 0.5
      max_range: 6.0
      voxel_size: 0.04
```

插件参数为启动时只读，避免运行中切换几何/模型。`voxel_size` 最大 0.04 m；独立矩形扫掠检查额外留出 0.06 m 覆盖体素误差与采样移动量。碰撞检查使用全部过滤点，不只使用 DUNE 选取的点。

## 验证

本机结果：11 项 C++ 测试全部通过，ROS 模拟链路通过；退出时的框架问题单独记录。完整证据见 [VALIDATION.md](VALIDATION.md)。

```bash
source install/setup.bash
ctest --test-dir build/finenav_example -R test_neupan_adapter --output-on-failure
python3 src/FineNav-Engine/finenav_example/create_custom_neupan_planner/tools/smoke_test.py
```

ROS 验证脚本默认使用独立 `ROS_DOMAIN_ID=187` 和本机通信，只启动引擎与模拟位姿/点云，**不启动驱动**。覆盖无点云停车、有效数据运动、近障碍停车、点云/定位中断停车与恢复、错误 frame 拒绝、闭环到点与朝向控制。日志为 `/tmp/neupan_engine_smoke.log`。C++ 用例额外覆盖模型尺寸拒绝、无效数值、扫掠旋转碰撞、首帧限加速度及偏置障碍绕行。

### 当前边界与已知问题

- 这是 NeuPAN 局部规划器接入，没有增加 A* 等全局搜索。单个目标用直线初始化；正面障碍、对称局部极小、U 型障碍或无通路时可能停住，不能保证自动绕出所有场景。测试保留了正面障碍陷入局部极小时停车且不碰撞的用例。可通过现有多点参考轨迹/上游全局规划接口引导。
- 本地移植使用静态点假设，不预测动态障碍速度。0.5 m 高度与平面模型不覆盖三维通行能力。
- 无真实小车测试；安装外参、base 原点、车体自反射、实际加减速、定位频率和控制时延仍需实车核对。
- 引擎现有 `finenav_util/ros/simple_action_server.hpp` 的 `~SimpleActionServer()` 对 `current_handle_` 无条件调用 `abort()`。导航已经成功后 Ctrl+C，会触发 `SUCCEEDED → ABORT` 非法状态转换并异常退出。该问题可在模拟链路复现，与 NeuPAN 求解无关；遵守不改框架的要求，本接入没有修改它。测试报告单独记录退出码，不把正常导航通过等同于正常退出。

## 来源

算法来源：[hanruihua/NeuPAN](https://github.com/hanruihua/NeuPAN)，Ruihua Han；C++ 核心现随整包放在 `FineNav/neupan_cpp_ws/src/neupan_cpp/libneupan`。模型结构来自原 Python 代码，二进制格式来自 C++ 项目的导出工具。本扩展与模型训练工具按 GPL-3.0-or-later 提供，原作者与第三方许可证保留；框架原文件保持其自身许可。
