# Custom Planner

Write a custom Planner when the available algorithms do not fit your robot or
task. Start from the
[Planner examples](https://github.com/FineNav/FineNav-Engine/tree/main/finenav_example/create_custom_planner).

## Define the contract

Define an `IWorldView` interface inside each Planner containing the environment
queries its algorithm uses. Each Planner owns its interface, even when two
algorithms currently need the same queries. Keep those queries independent of
a concrete Map type so a WorldView can adapt the selected Map later.

Choose input and output profiles for each pipeline stage. The layered example
uses pose input and pose output for path generation:

```cpp
static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};
```

The follower consumes that pose path and produces velocity commands:

```cpp
static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
static constexpr finenav::core::IOProfile OutputProfile{false, true, false, false};
```

## Implement the algorithm

Inside `DummyLinePlanner`, the contract is:

```cpp
class IWorldView {
  public:
    virtual ~IWorldView() = default;
    virtual bool isWalkable(const finenav::common::Position3D& position) const = 0;
};

using ConfigType = dummy_line_planner::Params;
using Context = finenav::core::PlanningContext<DummyLinePlanner>;
using Result = std::optional<finenav::core::OutputTrajectoryT<DummyLinePlanner>>;

void configure(const ConfigType& config);
void reset();
Result plan(const Context& context, const IWorldView& world_view);
```

Apply parameters in `configure()`, clear episode state in `reset()`, and compute
the output from `robot_state`, `ref_traj`, and the WorldView in `plan()`. Return
`std::nullopt` when valid input cannot produce a result.

The shared library is declared with `add_library(... SHARED ...)` and installed
to `lib`. The parent package must find `finenav_cmake` and declare it as a build
dependency. Refer to the example CMake files for the complete setup.

## Register and publish the plugin

Define the parameter YAML, generate its parameter library, and build the Planner
as a shared library. The YAML namespace, generated parameter namespace, and
registration name must match:

```cpp
FINENAV_REGISTER_PLANNER(
    finenav::example::DummyLinePlanner,
    dummy_line_planner)
```

Publish the library through the package's ament resource:

```cmake
finenav_register_algo_plugin(dummy_line_planner)
```

Then [write a WorldView](worldview.md) for the Planner and selected Map, and
assemble the registered components in a [C++ application](cpp-application.md).
