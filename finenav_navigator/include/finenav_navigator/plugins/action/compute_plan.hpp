// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <behaviortree_cpp/action_node.h>

#include "finenav_util/ros/task_channel.hpp"

namespace finenav::navigator {

class ComputePlan : public BT::SyncActionNode {
public:
    ComputePlan(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("layer",       "ControlLayer name, e.g. 'mppi_layer'"),
            BT::InputPort<geometry_msgs::msg::Pose>("goal", "{goal}"),
            BT::InputPort<std::string>("planner_id", "{planner_id}"),
        };
    }

    BT::NodeStatus tick() override;

private:
    bool goalChanged(const geometry_msgs::msg::Pose& a, double eps = 1e-4) const;
    geometry_msgs::msg::Pose last_sent_goal_{};
};

}  // namespace finenav::navigator
