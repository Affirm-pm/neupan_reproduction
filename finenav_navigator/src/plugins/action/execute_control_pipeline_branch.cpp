// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/execute_control_pipeline_branch.hpp"

#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include "finenav_navigator/pipeline_controller.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

BT::NodeStatus ExecuteControlPipelineBranch::tick() {
    PipelineController* pipeline = nullptr;
    if (!config().blackboard->get("_pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    std::string branch;
    if (!getInput("branch", branch)) return BT::NodeStatus::FAILURE;

    if (!pipeline->hasBranch(branch)) {
        std::string candidates;
        for (const auto& name : pipeline->branchNames()) candidates += " " + name;
        rclcpp::Node::SharedPtr node;
        if (config().blackboard->get("_node", node) && node) {
            FINENAV_LOG_ERROR(node->get_logger(),
                "ExecuteControlPipelineBranch: unknown branch '{}'. Known branches:{}",
                branch, candidates);
        }
        return BT::NodeStatus::FAILURE;
    }

    pipeline->activateBranch(branch);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::ExecuteControlPipelineBranch>("ExecuteControlPipelineBranch");
}
