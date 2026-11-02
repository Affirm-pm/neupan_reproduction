// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/get_faulted_control_layer.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include "finenav_interfaces/msg/task_feedback.hpp"
#include "finenav_navigator/pipeline_controller.hpp"

namespace finenav::navigator {

using FeedbackMsg = finenav_interfaces::msg::TaskFeedback;

BT::PortsList GetFaultedControlLayer::providedPorts() {
    return {
        BT::OutputPort<std::string>("control_layer",
            "Name of the most-upstream FAULTED ControlLayer of the active branch"),
        BT::InputPort<PipelineController*>("pipeline", "{@_pipeline}",
            "PipelineController holding the control pipeline state"),
    };
}

BT::NodeStatus GetFaultedControlLayer::tick() {
    PipelineController* pipeline = nullptr;
    if (!getInput("pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    const auto& stages = pipeline->activeStages();
    if (stages.empty()) return BT::NodeStatus::FAILURE;

    // 故障沿管线向下游传播，故活跃支路上的故障层构成 [根因 .. leaf] 的一段后缀：
    // 顺序扫到的第一个 FAULTED 就是根因。
    for (const auto& s : stages) {
        auto fb = pipeline->feedback(s);
        if (fb && fb->status == FeedbackMsg::FAULTED) {
            setOutput("control_layer", s);
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::GetFaultedControlLayer>("GetFaultedControlLayer");
}
