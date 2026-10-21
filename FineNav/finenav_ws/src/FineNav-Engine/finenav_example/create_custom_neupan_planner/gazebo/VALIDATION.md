# 原仿真验证记录（2026-10-21）

环境：ROS 2 Humble、Gazebo Fortress 6.18.0、原 `finenav_simulator`，默认 `flat_small_scale_maze.world`、`fine_robot`、原 64 × 360 三维激光及原物理驱动。真值定位，无 FAST-LIO。导航引擎框架未修改。

- 构建：原引擎/示例 9 个包及 `finenav_simulator` 编译成功。
- C++：12 项测试全部通过，含新训练 0.7 × 0.6 m 模型的几何核对与 C++/PyTorch 导出一致性；见 `test/adapter_result.xml`。
- 神经模型：16,000 步训练，50,000 个独立验证样本，近场距离误差 P99 约 0.80 mm；模型 SHA256 见 `../models/training_report.json`。这只验证轮廓距离网络，不是导航成功率。
- 实际到点：原出生点 `(2,2)` → `(4,2,yaw=0.5)`，Action `SUCCEEDED (4)`，位置误差 0.05127 m，朝向误差 0.07600 rad；最后命令为零，Gazebo 实际速度归零。按真实里程计对原迷宫墙体进行包络几何检查，没有侵入 5 cm 净空。见 `test/navigation.json`。
- 墙内目标：目标 `(4,4)` 位于原迷宫墙体中；未误报成功，停在约 `(4.01,3.26)`。取消后命令及物理速度归零，墙体净空检查通过。但 Action 返回 `ABORTED (6)`，未满足 `CANCELED (5)` 契约，因此 `test/wall_and_cancel.json` 的总体 `passed` 为 false，不能称全部仿真测试通过。

取消状态原因可从原源码直接定位：`finenav_navigator/src/naviagtor.cpp` 的取消分支调用 `terminate_all()`，而 `finenav_util/include/finenav_util/ros/simple_action_server.hpp` 的该函数调用 `abort()`。本次未修改这两个文件。另有前期已记录的导航成功后 Ctrl+C 退出异常，也未通过修改框架处理。

运行中观察到定位器状态历史时间戳告警，以及较慢渲染时偶发点云过期告警；后者触发零速保护。没有通过放宽数据超时来隐藏告警。

测试范围是原平地迷宫中的局部导航、姿态调整与停车。墙体净空为原地图静态矩形与机器人平面包络的几何检查，不是全量物理接触传感器验证。尚未证明复杂迷宫自主通关、全局绕行、斜坡、动态障碍或实车效果。

## 统一车型及三维视图更新（2026-10-21）

全部入口现共用上级目录的 `planner.yaml` 和 `models/diff_0p7_0p6.bin`。默认高度配置为 0.6 m；原 0.4 × 0.4 m 暂定模型退出当前配置，早期验证记录保留并标为历史。扩大车型后重新验证了避障、旋转扫掠、超时停车和导出一致性，12 项 C++ 测试通过，见 `../test/current_robot_adapter_result.xml`。RViz 默认改为 Orbit 三维视角，以出生点 `(2,2,0.4)` 为观察中心；导航仍为平面规划。
