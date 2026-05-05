// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav {

/**
 * @brief IsGoalReached — 检查机器人当前位置是否已到达目标。
 *
 * Blackboard 输入:
 *   - robot_pose (geometry_msgs::msg::Pose): 当前机器人位姿（由 Navigator 每 tick 更新）
 *   - goal       (geometry_msgs::msg::Pose): 导航目标位姿
 *
 * 输入端口:
 *   - tolerance  (double, 默认 0.25 m): 到达判定的距离阈值
 *
 * 返回 SUCCESS 表示已到达，FAILURE 表示尚未到达。
 */
class IsGoalReached : public BT::ConditionNode {
public:
    IsGoalReached(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Pose>("robot_pose", "{robot_pose}", "Current robot pose"),
            BT::InputPort<geometry_msgs::msg::Pose>("goal",       "{goal}",       "Goal pose"),
            BT::InputPort<double>("tolerance", 0.25, "Goal tolerance [m]"),
        };
    }

    BT::NodeStatus tick() override {
        geometry_msgs::msg::Pose robot_pose, goal;
        double tolerance = 0.25;

        if (!getInput("robot_pose", robot_pose) || !getInput("goal", goal)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("tolerance", tolerance);

        const double dx = robot_pose.position.x - goal.position.x;
        const double dy = robot_pose.position.y - goal.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        return (dist < tolerance) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

} // namespace finenav

