# Custom Planner

Write a custom Planner when the available algorithms do not fit your robot or task.
Start from the
[Planner example](https://github.com/FineNav/FineNav-Engine/tree/81c8f277518fb9229eb5e78be43375227490ce39/finenav_example/create_custom_planner).

## Define what your algorithm needs

Declare an `IWorldView` interface containing the environment queries your algorithm
uses. Keep those queries independent of a particular Map representation.
Users implement that interface in a WorldView for their chosen Map.

Choose input and output profiles to match your algorithm. The example takes poses
as input and produces velocity commands:

```cpp
static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
static constexpr finenav::core::IOProfile OutputProfile{false, true, false, false};
```

## Implement the algorithm

The example's main methods are:

```cpp
using ConfigType = dummy_planner::Params;
using IWorldView = DummyWorldView;
using Context = finenav::core::PlanningContext<DummyPlanner>;
using Result = std::optional<finenav::core::OutputTrajectoryT<DummyPlanner>>;

void configure(const ConfigType& cfg);
void reset();
Result plan(const Context& ctx, const IWorldView& view);
```

Apply parameters in `configure()`, clear planning state in `reset()`, and compute
the output in `plan()` using the robot state, reference trajectory, and WorldView.
Return `std::nullopt` when no result is available.

## Package and use it

Define the parameter YAML, generate its parameter library, and build the Planner
as a shared library. Register it in a `.cpp` file:

```cpp
FINENAV_REGISTER_PLANNER(finenav::example::DummyPlanner, dummy_planner)
```

For your plugin, replace the example class and namespace with your own and keep
the registration namespace consistent with the YAML schema.
Use the example's CMake file as a build starting point.

Then [write a WorldView](worldview.md) for your Planner and chosen Map,
and load both plugins in your [C++ application](cpp-application.md).