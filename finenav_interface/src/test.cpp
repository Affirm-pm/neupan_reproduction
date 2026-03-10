// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.


#include "simpleTestMap.hpp"
#include "simpleTestPlanner.hpp"
#include "finenav_manager.hpp"

using namespace finenav;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("finenav_demo");

using GlobalPlanners = PlannerSet<SimpleTestPlanner>;
using LocalLayers = PlannerSet<SimpleTestPlanner>;
using ReactivePlanners = PlannerSet<SimpleTestPlanner>;

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("finenav_demo");

    auto finenav_ptr = std::make_shared<FineNavManager>(node);

    auto global_layer = finenav_ptr->createControlLayer<GlobalPlanners>("global_layer");
    auto local_layer = finenav_ptr->createControlLayer<LocalLayers>("local_layer");
    auto reactive_layer = finenav_ptr->createControlLayer<ReactivePlanners>("reactive_layer");

    finenav_ptr->registerPipeline(FineNavManager::PipelineIO{}, global_layer, local_layer, reactive_layer);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
