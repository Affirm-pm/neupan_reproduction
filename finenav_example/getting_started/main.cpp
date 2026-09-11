// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Getting started: assemble a complete navigation stack using FineNavEngine.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine/finenav_engine.hpp"

using namespace finenav;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello_finenav");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    // 0. Set the default behavior tree (used by /goal_pose and navigateTo() without bt_id).
    engine->setDefaultBehaviorTree("GettingStartedNavigate");

    // 1. Load the algorithm .so's.  The planner, map and world_view libraries are
    //    deliberately NOT linked into this executable (see CMakeLists.txt) — a
    //    self-registering .so the executable references no symbol from would be
    //    dropped by --as-needed.  dlopen forces each registrar to fire, which
    //    populates the shared PlanRegistry + MapRegistry + WorldViewRegistry.
    //    "finenav_example/lib" resolves to <prefix>/lib and loads every *.so it finds.
    engine->loadAlgoPlugins({"finenav_example/lib"});

    // 2. Assemble a ControlLayer from a WorldView bridge by NAME.
    //    createLayerByWorldView resolves the planner + map TYPES the bridge declares
    //    back to their registered names ("dummy_planner" / "dummy_map"), reuses
    //    (or creates) the MapServer, builds the IWorldView and attaches it.  The
    //    end user never names a planner or a map — only the WorldView.
    engine->createLayerByWorldView("dummy_world_view", "my_layer", core::PeriodicPolicy{10.0});

    // 3. Declare the control pipeline as a named branch.  A branch is an
    //    ordered sequence of layer names; the first stage implicitly consumes
    //    _root (the navigation goal).  The branch name ("navigate") is what the
    //    behavior tree uses in <ExecuteControlPipelineBranch branch="..."/>.
    engine->wireBranch("navigate", {"my_layer"});

    // auto map = engine->getMapServer<example::DummyMap>>("dummy_map");

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
