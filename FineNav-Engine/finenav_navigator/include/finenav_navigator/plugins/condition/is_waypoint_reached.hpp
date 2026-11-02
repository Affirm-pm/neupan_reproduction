// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav::navigator {

/**
 * @brief IsWaypointReached — position-only check against the k-th waypoint.
 *
 * Reads the full `goal_poses[]` vector and judges the robot's **position**
 * against goal_poses[waypoint_index]. Unlike IsGoalReached (which checks the
 * terminal goal .back()), this node targets an intermediate/front waypoint and
 * performs **no orientation and no speed checks** — only XY distance plus an
 * optional Z distance.
 *
 * Ports:
 *   goal_poses      (std::vector<geometry_msgs::msg::Pose>,
 *                    default "{goal_poses}")
 *   waypoint_index  (int,    default 0)   Index into goal_poses to check.
 *   robot_pose      (geometry_msgs::msg::Pose, default "{robot_pose}")
 *   xy_tolerance    (double, default 0.25 m)   Horizontal distance threshold.
 *   z_tolerance     (double, default 0.0  m)   Vertical   distance threshold.
 *
 * Returns FAILURE when `waypoint_index` is out of range for `goal_poses`
 * (or when the goal_poses / robot_pose inputs are unavailable).
 * Returns SUCCESS when the robot is within tolerance of the k-th waypoint.
 */
class IsWaypointReached : public BT::ConditionNode {
public:
    IsWaypointReached(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
