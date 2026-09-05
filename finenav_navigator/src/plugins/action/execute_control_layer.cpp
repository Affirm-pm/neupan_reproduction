// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/execute_control_layer.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include "finenav_navigator/pipeline_controller.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"

namespace finenav::navigator {

BT::NodeStatus ExecuteControlLayer::onStart() {
    PipelineController* pipeline = nullptr;
    if (!config().blackboard->get("_pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }
    if (!getInput("layer", layer_)) return BT::NodeStatus::FAILURE;

    pipeline->startLayer(layer_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteControlLayer::onRunning() {
    using Status = finenav_interfaces::msg::TaskFeedback;

    PipelineController* pipeline = nullptr;
    if (!config().blackboard->get("_pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    auto fb = pipeline->feedback(layer_);
    if (fb && fb->status == Status::COMPLETED) return BT::NodeStatus::SUCCESS;
    if (fb && fb->status == Status::FAILED)    return BT::NodeStatus::FAILURE;

    return BT::NodeStatus::RUNNING;
}

void ExecuteControlLayer::onHalted() {
    PipelineController* pipeline = nullptr;
    if (config().blackboard->get("_pipeline", pipeline) && pipeline) {
        pipeline->stopLayer(layer_);
    }
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::ExecuteControlLayer>("ExecuteControlLayer");
}
