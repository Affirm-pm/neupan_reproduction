// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

class StopChassisMotion : public BT::SyncActionNode {
public:
    StopChassisMotion(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
