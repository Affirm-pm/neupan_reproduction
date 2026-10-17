# 本机验证记录（2026-10-17）

## 完成结果

- ROS 2 Humble 下 `finenav_example` 构建、安装成功。
- 清除 AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH / LD_LIBRARY_PATH / PYTHONPATH 后，仅 source `~/FineNav/finenav_ws/install/setup.bash` 可以找到 ros2、finenav_example 和全部插件运行库。
- C++：11 个测试，0 failure、0 error。见 `test/adapter_result.xml`。
- ROS 模拟输入：无点云、近障碍、点云超时、定位超时、错误点云 frame 均停车；数据恢复后继续；闭环到点和朝向控制通过。见 `test/smoke_result.json` 和 `test/smoke_engine.log`。
- 最后一次模拟闭环位置误差 0.001863 m、朝向误差 0.079692 rad，容差均为 0.08。模拟采用理想差速积分，这些数值不代表真实小车性能。
- 0.4 × 0.4 m 模型在 Python 中完成训练、导出；C++ 与 PyTorch 的 256 点输出一致性测试通过（最大允许差 1e-4）；加载器拒绝不匹配尺寸。
- 模型训练报告与模型 SHA-256 一致；工具 Python 语法、行为树 XML 和已有文件 diff 空白检查通过。

## 单目录整包搬迁验证（2026-10-17）

从源码压缩包解压到 `/tmp/finenav-relocation-check/FineNav`，清除工作区环境变量，不使用原目录的 build/install；通过包内 `tools/build.sh` 从零编译。

- 9 个 ROS 包全部构建成功；系统 Python、fmt 和 yaml-cpp，算法核心来自整包内并列的 `neupan_cpp_ws`。
- 11 项 C++ 测试全部通过（0 failure、0 error）。
- 仅 source 新路径的 install 后，动态库无缺失，不引用原工作区、外部 NeuPAN 或 Conda 路径。
- 在本地独立 ROS 域 188，模拟脚本的停车、数据恢复、闭环到点及朝向检查全部通过；位置误差 0.004624 m、朝向误差 0.074102 rad。结果保存在 `test/relocation_smoke_result.json`，仅代表模拟验证。
- 导航成功后的退出码仍为 -6，原因见下文；不声称正常退出或已经完成实车验证。
- 训练模型未改变，SHA-256 仍与 `models/training_report.json` 一致。

## 未解决的框架既有问题

ROS 链路的导航任务成功后，Ctrl+C 导致进程退出码 -6（SIGABRT）。错误：

```
goal_handle attempted invalid transition from state SUCCEEDED with event ABORT
```

源码位置：`finenav_util/include/finenav_util/ros/simple_action_server.hpp` 的 `~SimpleActionServer()`。析构时只判断 `current_handle_` 非空，随即对已成功的 goal 调用 `abort()`。本次未改该文件；这是“不修改导航引擎框架”约束下保留的已知问题。报告将正常导航结果与退出结果分别记录。

## 修改范围

已有文件只有以下三处（均在示例应用包）：

- `finenav_example/CMakeLists.txt`：加入新插件目录。
- `finenav_example/package.xml`：声明依赖及扩展许可。
- `finenav_example/getting_started/main.cpp`：从参数选择 WorldView 和行为树。

新增 `finenav_example/create_custom_neupan_planner/`；求解器离线源码和本机构建产物位于工作区 `neupan_dependencies/`，不参与 colcon 包发现。集成最初以原 `neupan_cpp_ws` 和 `NeuPAN` 为只读源码来源；打包改造后，C++ 源码已复制到整包内并列的 `FineNav/neupan_cpp_ws`，训练所需的原 ObsPointNet 模块也已随工具打包。历史日志中的旧路径仅为当时的验证记录。未修改驱动、定位或框架源文件。

## 仍需实车完成

精确车体足迹与运动参考点、传感器/base 高度外参、点云自反射、定位与控制时延、实际制动能力、动态障碍场景。正面障碍造成的局部极小可能使 NeuPAN 停住；本接入未增加全局搜索器。
