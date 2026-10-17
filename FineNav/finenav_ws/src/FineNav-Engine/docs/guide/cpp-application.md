# Configure Your Application

Use `FineNavEngine` to load your plugins, assemble a WorldView, and run navigation.
Start with the supplied application and change the component names and parameters.

## Assemble navigation

This setup uses the example plugins and behavior tree:

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "finenav_engine_cpp/finenav_engine.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hello_finenav");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    engine->setDefaultBehaviorTree("GettingStartedNavigate");
    const bool ready =
        engine->loadAlgoPlugins({"finenav_example/lib"}) &&
        engine->createControlLayerByWorldView(
            "dummy_world_view", "my_layer",
            finenav::core::PeriodicPolicy{10.0}) &&
        engine->registerControlBranch("navigate", {"my_layer"});

    if (!ready) {
        RCLCPP_ERROR(node->get_logger(), "Navigation setup failed.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Replace `finenav_example/lib` with your installed plugin package directory and
`dummy_world_view` with your registered WorldView name.
Include the libraries for its Planner and Map in the loaded plugin directories.
The WorldView registration selects the component types for you.

The example runs planning at 10 Hz. Keep the branch name `navigate` when using
`GettingStartedNavigate`, because that tree refers to it.

## Set runtime parameters

Use the example
[parameter file](https://github.com/FineNav/FineNav-Engine/blob/81c8f277518fb9229eb5e78be43375227490ce39/finenav_example/getting_started/params/sim_localizer.yaml)
as a starting point:

| Setting | What to configure |
| --- | --- |
| `behavior_trees` | Include the directory containing `GettingStartedNavigate`. |
| Localization topics | Connect your robot's odometry or pose input. |
| `use_sim_time` | Enable for simulation; disable for a physical robot. |
| Component parameters | Set values from the selected plugins' YAML schemas. |

The example file uses the host node name `hello_finenav`. Update its namespace
if you rename that node. The default frames are `map` and `base_link`;
call `setFrameNames()` before assembling components if your robot uses other names.

## Send a goal

With the node spinning, use RViz's `/goal_pose` input or invoke the C++ API from
an application callback. Goal poses use the configured world frame:

```cpp
geometry_msgs::msg::Pose goal;
goal.position.x = 2.0;
goal.position.y = 1.0;
goal.orientation.w = 1.0;
engine->navigateTo({goal});

// Request cancellation when needed.
engine->cancelNavigation();
```

For executable build configuration, copy the
[getting-started CMake example](https://github.com/FineNav/FineNav-Engine/blob/81c8f277518fb9229eb5e78be43375227490ce39/finenav_example/getting_started/CMakeLists.txt).