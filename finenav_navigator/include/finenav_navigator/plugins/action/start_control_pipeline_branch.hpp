// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief StartControlPipelineBranch — 一键启动一条 control_pipeline 支路（非阻塞）。
 *
 * 从黑盒读取 `_pipeline`，端口 `to` 为目标 leaf 名，调用 startBranch(to)：
 * 停 off-path 层 + 按拓扑序启动 on-path 层。SyncAction，立即返回 SUCCESS，不监控完成。
 */
class StartControlPipelineBranch : public BT::SyncActionNode {
public:
    StartControlPipelineBranch(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("to", "Target leaf of the branch to activate")};
    }

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
