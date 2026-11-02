# 2. 选择 Map 和 Planner

根据环境表示、传感器输入和规划任务选择 Map 与 Planner。
本例使用一个 Map 和两个 Planner，将目标点转换为位姿路径，再生成底盘速度指令。

## 示例组件

| 职责 | 实现 / 注册名 | 用途 |
| --- | --- | --- |
| 地图 | `DummyMap` / `dummy_map` | 提供最小占据查询，所有位置均被视为没有障碍物。 |
| 路径规划 | `DummyLinePlanner` / `dummy_line_planner` | 机器人当前位姿 + 目标 → 直线位姿路径。 |
| 路径跟踪 | `DummyFollowPlanner` / `dummy_follow_planner` | 机器人状态 + 位姿路径 → 速度指令。 |

## 选择指南

| 检查事项 | 检查内容 |
| --- | --- |
| 环境表示 | Map 的地图类型适合当前场景，如二维栅格或三维体素。 |
| 传感器输入 | Map 支持项目使用的传感器数据。 |
| 规划任务 | Planner 满足路径规划、路径跟踪等任务需求。 |
| 输入与输出 | Planner 的输入与上游数据匹配，输出满足下一层需求。 |
| 最终输出 | 末层输出与底盘接口兼容；本例使用 `/cmd_vel`。 |

示例代码的 Map 和 Planner 组件源码在
[Map](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_map/include/dummy_map.hpp)、
[直线 Planner](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_planner/include/dummy_line_planner.hpp) 和
[跟踪 Planner](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_planner/include/dummy_follow_planner.hpp) 中。

其他组件见 [finenav_planner](plugins/planners.md) 和 [finenav_map](plugins/maps.md)。
