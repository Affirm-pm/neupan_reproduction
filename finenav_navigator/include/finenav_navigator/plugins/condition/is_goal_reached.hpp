// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav {

/**
 * @brief IsGoalReached — 检查机器人是否已到达目标（位置 + 速度 + 角度三重判定）。
 *
 * Blackboard 输入:
 *   - robot_pose  (geometry_msgs::msg::Pose):  当前机器人位姿
 *   - robot_twist (geometry_msgs::msg::Twist): 当前机器人速度
 *   - goal        (geometry_msgs::msg::Pose):  导航目标位姿
 *
 * 输入端口:
 *   - tolerance        (double, 默认 0.25 m):   到达判定的距离阈值
 *   - yaw_tolerance    (double, 默认 0.0 rad):  到达判定的 yaw 角度阈值（绝对值）
 *                      设为 0.0 可禁用角度检查（仅判断位置＋速度）
 *   - speed_tolerance  (double, 默认 0.1 m/s):  到达判定的合速度阈值
 *                      设为 0.0 可禁用速度检查（仅判断位置）
 *
 * 仅当位置距离 < tolerance 且合速度 < speed_tolerance 且 |Δyaw| < yaw_tolerance 时返回 SUCCESS。
 */
class IsGoalReached : public BT::ConditionNode {
public:
    IsGoalReached(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Pose> ("robot_pose",      "{robot_pose}",  "Current robot pose"),
            BT::InputPort<geometry_msgs::msg::Twist>("robot_twist",     "{robot_twist}", "Current robot twist"),
            BT::InputPort<geometry_msgs::msg::Pose> ("goal",            "{goal}",        "Goal pose"),
            BT::InputPort<double>("tolerance",       0.25, "Goal position tolerance [m]"),
            BT::InputPort<double>("yaw_tolerance",   0.0,  "Goal yaw tolerance [rad]; set 0 to disable"),
            BT::InputPort<double>("speed_tolerance", 0.1,  "Goal speed tolerance [m/s]; set 0 to disable"),
        };
    }

    BT::NodeStatus tick() override {
        geometry_msgs::msg::Pose robot_pose, goal;
        double tolerance       = 0.25;
        double yaw_tolerance   = 0.0;
        double speed_tolerance = 0.1;

        if (!getInput("robot_pose", robot_pose) || !getInput("goal", goal)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("tolerance",       tolerance);
        getInput("yaw_tolerance",   yaw_tolerance);
        getInput("speed_tolerance", speed_tolerance);

        // 位置检查
        const double dx   = robot_pose.position.x - goal.position.x;
        const double dy   = robot_pose.position.y - goal.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist >= tolerance) {
            return BT::NodeStatus::FAILURE;
        }

        // 速度检查（speed_tolerance <= 0 或 robot_twist 不可用时跳过）
        geometry_msgs::msg::Twist robot_twist;
        if (speed_tolerance > 0.0 && getInput("robot_twist", robot_twist)) {
            const double vx    = robot_twist.linear.x;
            const double vy    = robot_twist.linear.y;
            const double speed = std::sqrt(vx * vx + vy * vy);
            if (speed >= speed_tolerance) {
                return BT::NodeStatus::FAILURE;
            }
        }

        // 角度检查（yaw_tolerance <= 0 时跳过）
        if (yaw_tolerance > 0.0) {
            const double robot_yaw = quaternionToYaw(robot_pose.orientation);
            const double goal_yaw  = quaternionToYaw(goal.orientation);
            const double yaw_diff  = std::abs(normalizeAngle(robot_yaw - goal_yaw));
            if (yaw_diff >= yaw_tolerance) {
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    /// Convert quaternion to yaw (rotation around Z)
    static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny, cosy);
    }

    /// Normalize angle to [-π, π]
    static double normalizeAngle(double angle) {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

} // namespace finenav

