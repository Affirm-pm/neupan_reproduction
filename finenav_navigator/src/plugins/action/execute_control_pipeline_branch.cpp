// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/execute_control_pipeline_branch.hpp"

#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "finenav_interfaces/msg/task_feedback.hpp"
#include "finenav_navigator/pipeline_controller.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

using FeedbackMsg = finenav_interfaces::msg::TaskFeedback;

BT::NodeStatus ExecuteControlPipelineBranch::onStart() {
    PipelineController* pipeline = nullptr;
    if (!config().blackboard->get("_pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("branch", branch_)) return BT::NodeStatus::FAILURE;

    if (!pipeline->hasBranch(branch_)) {
        std::string candidates;
        for (const auto& name : pipeline->branchNames()) candidates += " " + name;
        rclcpp::Node::SharedPtr node;
        if (config().blackboard->get("_node", node) && node) {
            FINENAV_LOG_ERROR(node->get_logger(),
                "ExecuteControlPipelineBranch: unknown branch '{}'. Known branches:{}",
                branch_, candidates);
        }
        return BT::NodeStatus::FAILURE;
    }

    pipeline->restartBranch(branch_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteControlPipelineBranch::onRunning() {
    PipelineController* pipeline = nullptr;
    if (!config().blackboard->get("_pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    const auto stages = pipeline->branchStages(branch_);
    if (stages.empty()) return BT::NodeStatus::FAILURE;

    // 任一 layer FAILED → FAILURE；否则一直 RUNNING。
    // 分支的"完成"不由这里判定 —— 由 task_master（IsGoalReached 等条件）下达 STOP，
    // 或在 BT 侧用其它条件节点决定何时把本节点 hal 掉（onHalted → stopBranch）。
    for (const auto& s : stages) {
        auto fb = pipeline->feedback(s);
        if (fb && fb->status == FeedbackMsg::FAILED) return BT::NodeStatus::FAILURE;
    }

    // 其余 → RUNNING（含 feedback 未发布的预热态）。
    return BT::NodeStatus::RUNNING;
}

void ExecuteControlPipelineBranch::onHalted() {
    PipelineController* pipeline = nullptr;
    if (config().blackboard->get("_pipeline", pipeline) && pipeline) {
        pipeline->stopBranch(branch_);
    }
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::ExecuteControlPipelineBranch>("ExecuteControlPipelineBranch");
}
