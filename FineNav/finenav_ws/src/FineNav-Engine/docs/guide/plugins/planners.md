# Planners

Planner plugins compute paths or motion commands.
Choose a Planner according to your robot's motion, navigation task, and
the queries available from your Map through a WorldView.

## Available entries

| Plugin | Input → Output | Repository |
| --- | --- | --- |
| `dummy_planner` | Goal poses → velocity commands | [FineNav-Engine](https://github.com/FineNav/FineNav-Engine/tree/81c8f277518fb9229eb5e78be43375227490ce39/finenav_example/create_custom_planner) |

## Dummy Planner

**Package:** `finenav_example`\
**Registered name:** `dummy_planner`\
**Example pairing:** `dummy_map` through `dummy_world_view`

This Planner uses the final pose in its input as the goal and generates
forward and angular velocity commands from the robot's current pose.
Its parameters include `linear_speed` and `angular_speed`.

The example demonstrates Planner integration and parameter configuration.
It does not perform obstacle avoidance or use the WorldView to check obstacles.

For installation and execution, follow the [Quickstart](../quickstart.md).
To connect a different Planner to your Map, see [WorldView integration](../worldview.md).
