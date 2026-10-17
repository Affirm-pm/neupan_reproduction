# Maps

Map plugins represent the environment and expose queries used by a WorldView.
Choose a Map according to your sensor inputs, environment representation,
and the information your Planner needs.

## Available entries

| Plugin | Purpose | Repository |
| --- | --- | --- |
| `dummy_map` | Minimal example Map for learning plugin integration. | [FineNav-Engine](https://github.com/FineNav/FineNav-Engine/tree/81c8f277518fb9229eb5e78be43375227490ce39/finenav_example/create_custom_map) |

## Dummy Map

**Package:** `finenav_example`\
**Registered name:** `dummy_map`\
**Example pairing:** `dummy_planner` through `dummy_world_view`

This Map exposes an occupancy query that always returns false and has no
observation sources. It provides an integration example for Map configuration
and WorldView queries.

It does not contain obstacle data. Use a Map with an appropriate environment
representation for navigation that requires obstacle information.

For installation and execution, follow the [Quickstart](../quickstart.md).
To connect another Map to a Planner, see [WorldView integration](../worldview.md).
