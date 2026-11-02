// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/execute_control_branch.hpp"

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "finenav_interfaces/msg/task_feedback.hpp"
#include "finenav_navigator/pipeline_controller.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

using FeedbackMsg = finenav_interfaces::msg::TaskFeedback;

BT::PortsList ExecuteControlBranch::providedPorts() {
    return {
        BT::InputPort<PipelineController*>("pipeline", "{@_pipeline}",
            "PipelineController driving the control branches"),
        BT::InputPort<rclcpp::Node::SharedPtr>("node", "{@_node}",
            "Node used for error logging"),
        BT::InputPort<std::string>("branch", "Name of the pipeline branch to activate"),
    };
}

BT::NodeStatus ExecuteControlBranch::onStart() {
    PipelineController* pipeline = nullptr;
    if (!getInput("pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("branch", branch_)) return BT::NodeStatus::FAILURE;

    if (!pipeline->hasBranch(branch_)) {
        std::string candidates;
        for (const auto& name : pipeline->branchNames()) candidates += " " + name;
        rclcpp::Node::SharedPtr node;
        if (getInput("node", node) && node) {
            FINENAV_LOG_ERROR(node->get_logger(),
                "ExecuteControlBranch: unknown branch '{}'. Known branches:{}",
                branch_, candidates);
        }
        return BT::NodeStatus::FAILURE;
    }

    pipeline->activateBranch(branch_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteControlBranch::onRunning() {
    PipelineController* pipeline = nullptr;
    if (!getInput("pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    const auto stages = pipeline->branchStages(branch_);
    if (stages.empty()) return BT::NodeStatus::FAILURE;

    // 任一 layer FAULTED → FAILURE；否则一直 RUNNING。
    // 分支的"完成"不由这里判定 —— 由 task_master（IsGoalReached 等条件）下达 STOP，
    // 或在 BT 侧用其它条件节点决定何时把本节点 hal 掉（onHalted → 条件清停）。
    for (const auto& s : stages) {
        auto fb = pipeline->feedback(s);
        if (fb && fb->status == FeedbackMsg::FAULTED) {
            return BT::NodeStatus::FAILURE;
        }
    }

    // 其余 → RUNNING（含 feedback 未发布的预热态）。
    return BT::NodeStatus::RUNNING;
}

void ExecuteControlBranch::onHalted() {
    PipelineController* pipeline = nullptr;
    if (getInput("pipeline", pipeline) && pipeline) {
        // 仅当本支路仍是活跃支路时才整管清停（取消/抢占）。若已被 activateBranch
        // 切走（多支路切换，新支路 onStart 先于本 onHalted），停止已由集合差完成，
        // 这里什么都不做，以免误停共享 stage / 新支路。
        if (pipeline->isActive(branch_)) {
            pipeline->stopAll();
        }
    }
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::ExecuteControlBranch>("ExecuteControlBranch");
}
