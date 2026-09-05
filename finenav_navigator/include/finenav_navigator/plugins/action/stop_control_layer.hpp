// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief StopControlLayer — 停止单个 control layer。
 */
class StopControlLayer : public BT::SyncActionNode {
public:
    StopControlLayer(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("layer", "ControlLayer to stop")};
    }

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
