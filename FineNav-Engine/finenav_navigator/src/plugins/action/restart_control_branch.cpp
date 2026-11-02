// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/restart_control_branch.hpp"

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "finenav_navigator/pipeline_controller.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

BT::PortsList RestartControlBranch::providedPorts() {
    return {
        BT::InputPort<PipelineController*>("pipeline", "{@_pipeline}",
            "PipelineController driving the control branches"),
        BT::InputPort<rclcpp::Node::SharedPtr>("node", "{@_node}",
            "Node used for error logging"),
        BT::InputPort<std::string>("branch", "Name of the pipeline branch to restart"),
    };
}

BT::NodeStatus RestartControlBranch::tick() {
    PipelineController* pipeline = nullptr;
    if (!getInput("pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    std::string branch;
    if (!getInput("branch", branch) || branch.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    if (!pipeline->hasBranch(branch)) {
        std::string candidates;
        for (const auto& name : pipeline->branchNames()) candidates += " " + name;
        rclcpp::Node::SharedPtr node;
        if (getInput("node", node) && node) {
            FINENAV_LOG_ERROR(node->get_logger(),
                "RestartControlBranch: unknown branch '{}'. Known branches:{}. No-op.",
                branch, candidates);
        }
        return BT::NodeStatus::SUCCESS;
    }

    pipeline->restartBranch(branch);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::RestartControlBranch>("RestartControlBranch");
}
