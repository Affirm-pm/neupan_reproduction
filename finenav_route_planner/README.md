# finenav_route_planner

FineNav-Engine 的栅格路径规划与局部绕障插件。

## 规划器

| 类型 | 注册名 / 参数命名空间 | 行为 |
|---|---|---|
| `AstarPathSearch` | `astar_planner` | 使用启发函数进行均匀栅格 A* 搜索 |
| `BfsPathSearch` | `bfs_planner` | 使用零启发式 Dijkstra 搜索，寻找累计代价最小的路径 |
| `DStarLitePathSearch` | `dstarlite_planner` | 保存 g/rhs 与优先队列，同步障碍变化后增量重规划 |
| `DetourPlanner` | `detour_planner` | 沿参考轨迹检测堵塞，并搜索可达参考点以生成二维绕行连接段 |

A*、Dijkstra、D* Lite 从机器人当前位置搜索到 `ctx.ref_traj` 的末位姿，支持二维 4/8 连通和三维 6/18/26 连通。`BfsPathSearch` 仅在所有边代价相等时等价于经典 BFS；对角步长会影响边代价。

Detour 固定使用 XY 平面的 8 连通搜索。它从距离机器人最近的参考点开始检查前方堵塞，按累计代价选择可达的重接点。搜索成功时输出机器人当前位置到重接点的连接段，再拼接参考轨迹后缀。参考轨迹为空时返回有效的空输出；步长非正、起点不可通行或搜索失败时透传原参考轨迹，由上层决定后续处理。

## WorldView

A*、Dijkstra、Detour 共用 [astar::IWorldView](include/finenav_route_planner/common/world_view.hpp)。

| 接口 | 要求 |
|---|---|
| `isWalkable(p)` | 判断世界坐标位置是否可通行，统一处理障碍物、未知区域和地图边界 |
| `getCost(p)` | 返回非负的额外通行代价；Detour 要求归一化到 `[0, 1]` |
| `getStepSize()` | 返回有限且大于零的栅格间距，单位 m |

输入位置使用 `WORLD_FIXED_FRAME`，单位为米。搜索栅格锚定世界原点，按 `floor(position / step)` 离散，单元中心为 `(index + 0.5) * step`。A*、Dijkstra、D* Lite 输出栅格中心位置，姿态使用单位四元数。Detour 连接段在 XY 平面生成，保留参考后缀中的原有位姿。

[DStarLitePathSearch::IWorldView](include/finenav_route_planner/dstarlite_path_search.hpp) 还需提供 `rows()`、`cols()` 和 `layers()`；`layers()` 默认返回 1。三个维度必须为正，起点和目标的栅格索引须满足 `0 <= x < cols`、`0 <= y < rows`、`0 <= z < layers`。

D* Lite 在同目标的后续规划中通过 `syncMap()` 扫描全图并同步通行状态变化。仅改变可通行格的 `getCost()` 值时，需要在下次规划前调用 `reset()` 重建搜索状态。地图分辨率改变时也需重建。

相邻栅格的几何步长为 `step_cost`（1、√2 或 √3），边代价为：

- A* / Dijkstra：`step_cost * (movement_weight + getCost(p))`。
- D* Lite：`step_cost * (movement_weight + terrain_weight * getCost(p))`。
- Detour：`step_cost * (1 + obstacle_cost_weight * getCost(p))`。

公共接口与搜索辅助代码位于 `include/finenav_route_planner/common/`，A* 与 D* Lite 的核心实现分别位于 `astar/` 和 `dstarlite/` 子目录。

## 参数

参数定义分别见 [astar.yaml](astar.yaml)、[bfs.yaml](bfs.yaml)、[dstarlite.yaml](dstarlite.yaml) 和 [detour.yaml](detour.yaml)，由 `generate_parameter_library` 生成。

### AstarPathSearch（命名空间 `astar_planner`）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `connectivity` | string | "8" | 栅格连通性：4/8（2D），6/18/26（3D） |
| `heuristic` | string | "Euclidean" | 启发函数：Euclidean / Manhattan / Chebyshev |
| `goal_tolerance` | double | 0.5 | 未到达目标时，可接受的已访问栅格到目标的最大距离（m） |
| `max_iterations` | int | 1000000 | 最大扩展节点数（-1 不限制） |
| `max_planning_time` | double | 5.0 | 最大规划时长（s） |
| `movement_weight` | double | 1.0 | 边代价中移动距离项的系数 |
| `greedy_weight` | double | 1.001 | 贪心启发项的权重 |

### BfsPathSearch（命名空间 `bfs_planner`）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `connectivity` | string | "8" | 栅格连通性：4/8（2D），6/18/26（3D） |
| `goal_tolerance` | double | 0.5 | 未到达目标时，可接受的已访问栅格到目标的最大距离（m） |
| `max_iterations` | int | 1000000 | 最大扩展节点数（-1 不限制） |
| `max_planning_time` | double | 5.0 | 最大规划时长（s） |
| `movement_weight` | double | 1.0 | 移动距离项的系数；越大越偏好短路径 |

### DStarLitePathSearch（命名空间 `dstarlite_planner`）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `connectivity` | string | "8" | 栅格连通性：4/8（2D），6/18/26（3D） |
| `heuristic` | string | "Diagonal" | 启发函数：Euclidean / Manhattan / Chebyshev / Diagonal |
| `goal_tolerance` | double | 0.5 | 原目标不可通行时的替代目标搜索范围（m），向上取整为栅格数并按 Manhattan 距离筛选 |
| `max_expansions` | int | 1000000 | 最大扩展节点数，非正数不限制 |
| `max_path_steps` | int | 10000 | 路径提取步数上限，负数不限制；达到上限仍未到目标时返回失败 |
| `max_planning_time` | double | 5.0 | 最大规划时长（s），非正数不限制 |
| `movement_weight` | double | 1.0 | 移动距离项的系数；越大，地形代价的相对影响越小 |
| `greedy_weight` | double | 1.0 | D* Lite 优先级中的启发项系数，大于 1 时更偏重启发引导 |
| `terrain_weight` | double | 1.0 | 地形代价乘子（边代价 = 步长 × (movement_weight + terrain_weight × cost)） |

### DetourPlanner（命名空间 `detour_planner`）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `scan_distance_m` | double | 5.0 | 沿参考轨迹向前扫描堵塞的最远距离（m） |
| `blockage_threshold` | double | 0.8 | 软堵塞：归一化代价（0-1）达到此值即触发绕行；`isWalkable()` 为 false 的路径点始终触发 |
| `reconnect_offset_m` | double | 1.0 | 重接候选起点沿参考轨迹越过堵塞点的距离（m），最长推进到轨迹终点 |
| `obstacle_cost_weight` | double | 1.0 | 绕行搜索中地图代价的系数（0 = 忽略代价，越大越避障） |
| `max_planning_time` | double | 0.1 | 单次绕行搜索的最大耗时（s） |

## 示例

在 ROS 2 参数文件中，将以下参数放在对应节点的 `ros__parameters` 下。四个规划器的参数命名空间相互独立，默认值如下：

```yaml
astar_planner:
  connectivity: "8"
  heuristic: "Euclidean"
  goal_tolerance: 0.5
  max_iterations: 1000000
  max_planning_time: 5.0
  movement_weight: 1.0
  greedy_weight: 1.001

bfs_planner:
  connectivity: "8"
  goal_tolerance: 0.5
  max_iterations: 1000000
  max_planning_time: 5.0
  movement_weight: 1.0

dstarlite_planner:
  connectivity: "8"
  heuristic: "Diagonal"
  goal_tolerance: 0.5
  max_expansions: 1000000
  max_path_steps: 10000
  max_planning_time: 5.0
  movement_weight: 1.0
  greedy_weight: 1.0
  terrain_weight: 1.0

detour_planner:
  scan_distance_m: 5.0
  blockage_threshold: 0.8
  reconnect_offset_m: 1.0
  obstacle_cost_weight: 1.0
  max_planning_time: 0.1
```

## 构建与验证

在已准备 FineNav-Engine 依赖的 ROS 2 工作区根目录执行。启用测试需要 `ament_cmake_gtest` 和 Google Benchmark：

```bash
colcon build --packages-up-to finenav_route_planner --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select finenav_route_planner --event-handlers console_direct+
colcon test-result --verbose
```

单元测试目标为 `astar_test`、`bfs_test`、`dstarlite_test` 和 `detour_test`。算法基准程序运行方式：

```bash
./build/finenav_route_planner/benchmark/route_planner_benchmark
```

评估程序直接调用 A* 和 D* Lite 核心算法。场景选择、地图和权重配置见 [eval_config.hpp](eval/eval_config.hpp)，修改后重新构建。默认运行空地图、障碍变化、地形权重扫描和随机障碍发现四组场景：

```bash
./build/finenav_route_planner/eval/route_planner_eval
```

程序自动调用 [plot.py](eval/plot.py) 绘图，需要 `python3`、NumPy、pandas 和 Matplotlib 3.5 或更新版本。默认图片写入 `/tmp/dstarlite_eval/`，输出目录可在 `eval_config.hpp` 中修改。绘图后会清理中间 CSV 和 JSON 文件。

## License

本包采用 [Mozilla Public License 2.0（MPL-2.0）](LICENSE)。
