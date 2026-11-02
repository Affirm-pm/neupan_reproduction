# finenav_planner

The planner collection is maintained in
[FineNav/finenav_planner](https://github.com/FineNav/finenav_planner).

## Download

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone https://github.com/FineNav/finenav_planner.git src/finenav_planner
```

## Component catalogue

| ROS package | Registered Planner | Purpose |
| --- | --- | --- |
| `finenav_route_planner` | `astar_planner` | Grid A* route search. |
| `finenav_route_planner` | `bfs_planner` | Dijkstra grid search with a zero heuristic. |
| `finenav_route_planner` | `dstarlite_planner` | D* Lite incremental replanning after obstacle changes. |
| `finenav_route_planner` | `detour_planner` | Detects blockages along a reference path and generates a local 2D detour. |
| `finenav_mppi_SE2` | `finenav_mppi_SE2` | MPPI local planning for ground robots. |
| `finenav_rotate_shim` | `finenav_rotate_shim` | Heading alignment before path tracking. |

Package READMEs describe their interfaces and parameters:
[route planners](https://github.com/FineNav/finenav_planner/tree/main/finenav_route_planner),
[MPPI](https://github.com/FineNav/finenav_planner/tree/main/finenav_mppi_SE2),
[heading alignment](https://github.com/FineNav/finenav_planner/tree/main/finenav_rotate_heading_aligner).

## Example Planners

Getting Started uses two separate examples bundled in `finenav_example`:

| Planner | Input → output | Main parameters |
| --- | --- | --- |
| `dummy_line_planner` | Robot pose + goal → straight pose path | `path_resolution` (m), default 0.2. |
| `dummy_follow_planner` | Robot state + path → velocity command | `linear_speed` (m/s) 0.5; `angular_speed` gain 1.0; `lookahead_distance` (m) 1.0. |

The dummy follower illustrates tracking; it has no final-heading alignment or
terminal braking. See [Choose Map and Planner](../choose-algorithms.md) and
[Assemble a control pipeline](../cpp-application.md).

Need another algorithm? Continue with [Custom Planner](../custom-planner.md).
