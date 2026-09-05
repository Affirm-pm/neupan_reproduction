// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief StartControlLayer — 手动启动单个 control layer 并监控其完成。
 *
 * 端口 `layer`（层名）。onStart 启动层；onRunning 轮询该层反馈，
 * COMPLETED→SUCCESS / FAILED→FAILURE；onHalted 停止该层。StatefulActionNode。
 */
class StartControlLayer : public BT::StatefulActionNode {
public:
    StartControlLayer(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("layer", "ControlLayer name to run")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string layer_;
};

}  // namespace finenav::navigator
