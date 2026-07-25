// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <behaviortree_cpp/action_node.h>

#include "finenav_util/ros/task_channel.hpp"

namespace finenav::navigator {

class ComputePlan : public BT::StatefulActionNode {
public:
    ComputePlan(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("layer",       "ControlLayer name, e.g. 'mppi_layer'"),
            BT::InputPort<geometry_msgs::msg::Pose>("goal", "{goal}"),
            BT::InputPort<std::string>("planner_id", "{planner_id}"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    bool taskChanged(const geometry_msgs::msg::Pose& goal, const std::string& planner_id) const;
    std::string              layer_;
    geometry_msgs::msg::Pose last_goal_{};
    std::string              last_planner_id_;
};

}  // namespace finenav::navigator
