# 2. Choose Map and Planner

Choose a Map and Planners based on the environment representation, sensor inputs,
and planning tasks. This example uses one Map and two Planners to turn a goal
into a pose path, then into velocity commands.

## Example components

| Role | Implementation / registered name | Purpose |
| --- | --- | --- |
| Map | `DummyMap` / `dummy_map` | Supplies a minimal occupancy query; every position is unoccupied. |
| Path Planner | `DummyLinePlanner` / `dummy_line_planner` | Current robot pose + goal → straight pose path. |
| Tracking Planner | `DummyFollowPlanner` / `dummy_follow_planner` | Robot state + pose path → velocity command. |

## Selection guide

| Check item | Requirement |
| --- | --- |
| Environment representation | The Map type suits the environment, such as a 2D grid or 3D voxels. |
| Sensor inputs | The Map supports the project's sensor data. |
| Planning task | The Planner meets the requirements for tasks such as path planning or path tracking. |
| Inputs and outputs | The Planner accepts upstream data and produces the output required by the next layer. |
| Final output | The final output matches the chassis interface; this example uses `/cmd_vel`. |

The source code for the example's Map and Planner components is in
[Map](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_map/include/dummy_map.hpp),
[line Planner](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_planner/include/dummy_line_planner.hpp), and
[tracking Planner](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_planner/include/dummy_follow_planner.hpp).

For alternatives, see [finenav_planner](plugins/planners.md) and
[finenav_map](plugins/maps.md).
