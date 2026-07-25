// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav::navigator {

/**
 * @brief IsStuck — detects whether the robot has not moved enough within a time window.
 *
 * Blackboard input:
 *   - robot_pose (geometry_msgs::msg::Pose)
 *
 * Ports:
 *   - dist_threshold (double, default 0.05 m): Minimum displacement within window.
 *   - timeout_s      (double, default 5.0  s): Time window for stuck detection.
 *
 * Returns SUCCESS when stuck, FAILURE when moving normally.
 */
class IsStuck : public BT::ConditionNode {
public:
    IsStuck(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    bool   ref_set_{false};
    double ref_x_{0.0}, ref_y_{0.0};
    std::chrono::steady_clock::time_point window_start_;
    std::chrono::steady_clock::time_point last_tick_time_;
};

}  // namespace finenav::navigator
