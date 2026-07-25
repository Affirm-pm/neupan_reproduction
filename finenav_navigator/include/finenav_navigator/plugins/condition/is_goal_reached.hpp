// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav::navigator {

/**
 * @brief Per-axis goal-reached checker (2D/3D).
 *
 * Each axis is controlled by its own tolerance port.  A tolerance of 0.0
 * disables that axis entirely, so it never blocks SUCCESS.
 *
 * Ports:
 *   xy_tolerance     (double, default 0.25 m)   Horizontal distance threshold.
 *   z_tolerance      (double, default 0.0  m)   Vertical   distance threshold.
 *   yaw_tolerance    (double, default 0.0  rad) Yaw   angle threshold.
 *   roll_tolerance   (double, default 0.0  rad) Roll  angle threshold.
 *   pitch_tolerance  (double, default 0.0  rad) Pitch angle threshold.
 *   speed_tolerance  (double, default 0.1 m/s)  Linear speed threshold.
 *
 * Blackboard inputs:
 *   robot_pose   (geometry_msgs::msg::Pose)
 *   robot_twist  (geometry_msgs::msg::Twist)  — optional, used only for speed.
 *   goal         (geometry_msgs::msg::Pose)
 *
 * Returns SUCCESS when all enabled checks pass simultaneously.
 */
class IsGoalReached : public BT::ConditionNode {
public:
    IsGoalReached(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    static void quaternionToRPY(const geometry_msgs::msg::Quaternion& q,
                                double& roll, double& pitch, double& yaw);
    static double normalizeAngle(double angle);
};

}  // namespace finenav::navigator
