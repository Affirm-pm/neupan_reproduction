# 3. Write a WorldView

A WorldView bridges a Map and a Planner by converting map information into a form the Planner can use directly, such as traversal costs.
To combine your chosen Map and Planner plugins, implement a WorldView that connects them.

## Planner interface

A Planner defines an abstract `IWorldView` class to specify the interfaces that a user-written WorldView must provide. For example,
`DummyLinePlanner` declares the interface inside its class:

```cpp
class DummyLinePlanner {
  public:
    class IWorldView {
      public:
        virtual ~IWorldView() = default;
        virtual bool isWalkable(const finenav::common::Position3D& position) const = 0;
    };
    // ...
};
```

`DummyFollowPlanner` declares a separate `IWorldView` with the same query.
The `IWorldView` interfaces defined by different Planners are distinct types and must be implemented separately.
Users implement the required WorldView logic in concrete classes derived from the corresponding `IWorldView`, adapting map information into the query results each Planner needs.

## Implement a WorldView derived class

`DummyLineWorldView` derives from `DummyLinePlanner::IWorldView`, keeps a reference to the selected Map, and implements the walkability query required by the Planner.
The class is defined in namespace `finenav::example`:

```cpp
class DummyLineWorldView : public DummyLinePlanner::IWorldView {
  public:
    // Parameter type used by the derived class, generated from the YAML parameter definition.
    using ConfigType = dummy_line_world_view::Params;

    // Keep a read-only reference to the Map for subsequent queries.
    DummyLineWorldView(rclcpp::Node::SharedPtr, const DummyMap& map) : map_(map) {}

    // Accept adapter parameters. This example has no parameters to apply.
    void configure(const ConfigType&) {}

    // Prepare data before each planning call. Direct Map queries need no preprocessing here.
    void prepare(const finenav::core::PlanningContext<DummyLinePlanner>&) {}

    // Implement the Planner's interface by converting occupancy to walkability.
    bool isWalkable(const finenav::common::Position3D& position) const override {
        const bool occupied = map_.isOccupied(position);
        return !occupied;
    }

  private:
    const DummyMap& map_;
};
```

The conversion is straightforward: an occupied position is not walkable, while an unoccupied position is walkable.
For your own WorldView, implement each query declared by the corresponding `IWorldView` using data from the selected Map.
The class must also provide `configure()` and `prepare()`; add processing to these methods when your application needs parameter handling or data preparation.

## Type binding and plugin registration

The registration file binds each adapter to one Planner and the selected Map:

```cpp
FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyLineWorldView, dummy_line_world_view,
                           finenav::example::DummyLinePlanner, finenav::example::DummyMap)

FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyFollowWorldView, dummy_follow_world_view,
                           finenav::example::DummyFollowPlanner, finenav::example::DummyMap)
```

The registration name is what the application passes to `createControlLayer()`.
The parameter schema files are `dummy_line_world_view.yaml` and
`dummy_follow_world_view.yaml` beside the CMake file.
The build generates parameter code, compiles and installs the shared library, and registers it for Engine discovery. The key statements are:

```cmake
# Generate parameter code for both WorldViews.
generate_parameter_library(dummy_line_world_view_params dummy_line_world_view.yaml)
generate_parameter_library(dummy_follow_world_view_params dummy_follow_world_view.yaml)

# Compile the WorldView implementations into a shared library.
add_library(dummy_world_view SHARED src/dummy_world_view.cpp)

# Install the library and register it for Engine discovery.
install(TARGETS dummy_world_view LIBRARY DESTINATION lib)
finenav_register_algo_plugin(dummy_world_view)
```

See the complete [CMakeLists.txt](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/CMakeLists.txt)
for include paths, dependency links, and installation rules.

The relevant source code is available in the [adapter classes](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/include/dummy_world_view.hpp),
[registrations](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/src/dummy_world_view.cpp),
[CMake](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/create_custom_world_view/CMakeLists.txt).

Continue with [4. Assemble a control pipeline](cpp-application.md).
