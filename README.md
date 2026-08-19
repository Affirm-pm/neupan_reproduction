# FineNav2D

## How to use

1. clone the repository
   
```shell
mkdir nav_ws && cd nav_ws/
git clone https://gitee.com/huigg-practice/FineNav2D.git
cd FineNav2D/
git submodule update --init --recursive 
```

2. build the project

```shell
cd ..
colcon build --symlink-install
```

3. launch

```shell
# Make sure you are in the root directory of the project
source install/setup.bash
```

## FineNav MPPI controller

`finenav_mppi_controller` 使用 Eigen 作为内部矩阵实现，不再依赖 xtensor 或 xsimd。构建前请先加载 ROS 2 Humble 和本机保存的依赖工作区：

```shell
source /opt/ros/humble/setup.bash
source /home/fins/Desktop/tl_expected/expected/install/setup.bash
source /home/fins/Desktop/generate_parameter_library/install/setup.bash
source /home/fins/Desktop/local_libs/BehaviorTreeCpp_v4.7.0/install/setup.bash
```

在仓库根目录构建 MPPI 包：

```shell
colcon build --symlink-install --packages-select finenav_mppi_controller
source install/setup.bash
```

运行 MPPI 单元测试时，优先使用当前工作区生成的库，避免系统中同名的旧库被动态链接器加载：

```shell
export LD_LIBRARY_PATH="$PWD/build/finenav_mppi_controller:$LD_LIBRARY_PATH"
colcon test --packages-select finenav_mppi_controller \
  --event-handlers console_direct+
colcon test-result --verbose
```

当前测试覆盖噪声生成、模型与 Eigen 数据形状、critic 评分、优化器轨迹积分以及 controller 规划流程。

如需构建并运行 Eigen benchmark：

```shell
colcon build --symlink-install --packages-select finenav_mppi_controller \
  --cmake-args -DBUILD_BENCHMARKS=ON
source install/setup.bash
build/finenav_mppi_controller/benchmark/mppi_practical_comparison \
  --benchmark_repetitions=15 \
  --benchmark_out_format=json \
  --benchmark_out=/tmp/eigen_practical.json
```

如需复现 Desktop 上的 Eigen/xtensor 实践型 A/B 数据，固定 CPU 并启用相同输入噪声：

```shell
source /home/fins/Desktop/nav_ws_map_test/install/setup.bash
taskset -c 2 env FINENAV_MPPI_BENCHMARK_DETERMINISTIC_NOISE=1 \
  LD_LIBRARY_PATH="$PWD/build/finenav_mppi_controller:$LD_LIBRARY_PATH" \
  build/finenav_mppi_controller/benchmark/mppi_practical_comparison \
  --benchmark_repetitions=15 \
  --benchmark_out=/home/fins/Desktop/finenav_engine_design_docs/eigen_benchmark_comparison/eigen_practical.json \
  --benchmark_out_format=json
```

benchmark 使用 `5000x56x2` 重负载、`3000x56x1` 兼容规模、open/inflated/mixed 三种场景，覆盖完整 Optimizer 流程和全部 10 个 Critic。输出中的 median/P95 仅用于同一台机器、同一组参数下的相对比较，不能直接外推到其他硬件或运行配置。xtensor 基线只保存在本地 `/tmp`，设计文档和 JSON 结果保存在 Desktop，不提交到远程仓库。

Example: Bring up FineNav2D with Livox Mid-360 and Fast-LIO

```shell
# Check your ethernet interface
ifconfig
# Set the static IP address of your ethernet interface
# The default ipv4 address is 192.168.1.50, you can change it in <PROJECT_DIR>/fn_bringup/config/MID360_config.yaml
sudo ifconfig <interface> <ip>
# Launch FineNav2D
ros2 launch fine_nav2d_bringup bringup_nav2_real.launch.py \
lidar_type:=livox \
enable_recorder:=false \
lio_type:=fast_lio \
enable_rviz:=true
```
If everything goes well, you will see the following result:
![expected_result.png](asset/expected_result.png)
