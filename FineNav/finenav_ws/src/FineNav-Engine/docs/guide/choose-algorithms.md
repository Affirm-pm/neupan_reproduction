# Choose Map and Planner

Choose components that fit your robot, sensor data, and navigation task.
You can use an existing Map and Planner together by writing a WorldView for that pair.

## What to check

| Choice | Check before using it |
| --- | --- |
| Map | Does it accept your environment data and expose the queries you need? |
| Planner | Does it support your robot's motion and produce the required path or commands? |
| WorldView | Can the selected Map answer every query in the Planner's `IWorldView` interface? |

For example, a Planner may ask whether a position is walkable, while a Map exposes
an occupancy query. Your WorldView connects these two operations.

If a suitable WorldView is already provided with the components, use its registered
name directly in your C++ application.

## Find a plugin

Browse [FineNav Plugins](plugins/index.md) for implemented components and repository links.
The [Maps](plugins/maps.md) and [Planners](plugins/planners.md) pages describe
their capabilities and example pairings. Use each plugin's installation instructions
when it is distributed in a separate repository.

Once you have chosen your components, [write a WorldView](worldview.md).
If you need a new algorithm or map representation, use the
[Planner](custom-planner.md) or [Map](custom-map.md) development guides.