# finenav_planner

规划组件集中维护在 [FineNav/finenav_planner](https://github.com/FineNav/finenav_planner)。

## 下载

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone https://github.com/FineNav/finenav_planner.git src/finenav_planner
```

## 组件目录

| ROS 包 | Planner 注册名 | 用途 |
| --- | --- | --- |
| `finenav_route_planner` | `astar_planner` | 栅格 A* 路径搜索。 |
| `finenav_route_planner` | `bfs_planner` | 使用零启发函数的 Dijkstra 栅格搜索。 |
| `finenav_route_planner` | `dstarlite_planner` | 障碍变化后的 D* Lite 增量重规划。 |
| `finenav_route_planner` | `detour_planner` | 检测参考路径堵塞并生成二维局部绕行路径。 |
| `finenav_mppi_SE2` | `finenav_mppi_SE2` | 面向地面机器人的 MPPI 局部规划。 |
| `finenav_rotate_shim` | `finenav_rotate_shim` | 路径跟踪前的航向对齐。 |

具体接口和参数见对应目录的 README：
[路径规划](https://github.com/FineNav/finenav_planner/tree/main/finenav_route_planner)、
[MPPI](https://github.com/FineNav/finenav_planner/tree/main/finenav_mppi_SE2)、
[航向对齐](https://github.com/FineNav/finenav_planner/tree/main/finenav_rotate_heading_aligner)。

## 示例 Planner

Getting Started 使用的是 `finenav_example` 自带的两个示例：

| Planner | 输入 → 输出 | 主要参数 |
| --- | --- | --- |
| `dummy_line_planner` | 机器人位姿 + 目标 → 直线位姿路径 | `path_resolution`：采样间距，默认 0.2 m。 |
| `dummy_follow_planner` | 机器人状态 + 路径 → 速度指令 | `linear_speed` 默认 0.5 m/s；`angular_speed` 增益默认 1.0；`lookahead_distance` 默认 1.0 m。 |

Dummy 跟踪器用于演示跟踪接入，没有终点朝向对齐或到点减速。
对应教程：[选择 Map 和 Planner](../choose-algorithms.md)、[组装控制 Pipeline](../cpp-application.md)。

需要扩展算法时，继续阅读[自定义 Planner](../custom-planner.md)。
