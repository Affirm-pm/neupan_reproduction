// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief ExecuteControlPipelineBranch — 一键激活一条 control_pipeline 支路（非阻塞）。
 *
 * 从黑盒读取 `_pipeline`，端口 `branch` 为支路名，调用 activateBranch(branch)：
 * 集合差激活整条支路（停旧起新、共享 stage 保持运行）。SyncAction，
 * fire-and-forget —— 激活后立即 SUCCESS，不阻塞等待完成；「到达目标」由 BT
 * 条件节点（IsGoalReached 等）轮询判断。未知支路名 → FAILURE（候选名列日志）。
 */
class ExecuteControlPipelineBranch : public BT::SyncActionNode {
public:
    ExecuteControlPipelineBranch(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("branch", "Name of the pipeline branch to activate")};
    }

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
