// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine_cpp/finenav_engine.hpp"

using namespace finenav;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello_finenav");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    engine->setDefaultBehaviorTree("GettingStartedNavigate");

    const bool use_global_planner = node->declare_parameter("use_global_planner", true);
    if (!engine->loadAlgoPlugins({"finenav_example"}) ||
        (use_global_planner && !engine->createControlLayer(
            "global_grid_world_view", "global_layer", core::PeriodicPolicy{1.0})) ||
        !engine->createControlLayer(
            "neupan_world_view", "my_layer", core::PeriodicPolicy{10.0}) ||
        !engine->registerControlBranch("navigate", use_global_planner
            ? std::vector<std::string>{"global_layer", "my_layer"}
            : std::vector<std::string>{"my_layer"})) {
        RCLCPP_ERROR(node->get_logger(), "Failed to assemble the NEUPAN control branch.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
