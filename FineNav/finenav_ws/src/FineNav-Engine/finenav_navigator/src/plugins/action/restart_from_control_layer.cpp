// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/restart_from_control_layer.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include "finenav_navigator/pipeline_controller.hpp"

namespace finenav::navigator {

BT::NodeStatus RestartFromControlLayer::tick() {
    PipelineController* pipeline = nullptr;
    if (!config().blackboard->get("_pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    std::string control_layer;
    if (!getInput("control_layer", control_layer) || control_layer.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    pipeline->restartFrom(control_layer);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::RestartFromControlLayer>("RestartFromControlLayer");
}
