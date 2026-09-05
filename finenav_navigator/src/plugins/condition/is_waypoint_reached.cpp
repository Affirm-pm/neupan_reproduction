// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/condition/is_waypoint_reached.hpp"

#include <cmath>
#include <vector>
#include <behaviortree_cpp/bt_factory.h>

namespace finenav::navigator {

IsWaypointReached::IsWaypointReached(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsWaypointReached::providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Pose>>(
            "goal_poses", "{goal_poses}", "Goal poses; waypoint_index selects one"),
        BT::InputPort<int>("waypoint_index", 0, "Index of the waypoint to check"),
        BT::InputPort<geometry_msgs::msg::Pose>("robot_pose", "{robot_pose}", "Current robot pose"),

        BT::InputPort<double>("xy_tolerance", 0.25, "Horizontal distance [m]; 0 = skip"),
        BT::InputPort<double>("z_tolerance",  0.0,  "Vertical   distance [m]; 0 = skip"),
    };
}

BT::NodeStatus IsWaypointReached::tick() {
    geometry_msgs::msg::Pose robot_pose;
    std::vector<geometry_msgs::msg::Pose> goal_poses;
    int waypoint_index = 0;

    if (!getInput("robot_pose", robot_pose) ||
        !getInput("goal_poses", goal_poses) ||
        goal_poses.empty() ||
        !getInput("waypoint_index", waypoint_index)) {
        return BT::NodeStatus::FAILURE;
    }

    // 越界（含负数与超上限）一律视为未到达。
    if (waypoint_index < 0 ||
        waypoint_index >= static_cast<int>(goal_poses.size())) {
        return BT::NodeStatus::FAILURE;
    }

    double xy_tol = 0.25;
    double z_tol  = 0.0;
    getInput("xy_tolerance", xy_tol);
    getInput("z_tolerance",  z_tol);

    const auto& wp = goal_poses[static_cast<size_t>(waypoint_index)];

    // ── Position (XY) ────────────────────────────────────────────────────
    const double dx = robot_pose.position.x - wp.position.x;
    const double dy = robot_pose.position.y - wp.position.y;

    if (xy_tol > 0.0) {
        const double xy_dist = std::sqrt(dx * dx + dy * dy);
        if (xy_dist >= xy_tol) return BT::NodeStatus::FAILURE;
    }

    // ── Position (Z, optional) ───────────────────────────────────────────
    if (z_tol > 0.0) {
        const double dz = std::abs(robot_pose.position.z - wp.position.z);
        if (dz >= z_tol) return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::IsWaypointReached>("IsWaypointReached");
}
