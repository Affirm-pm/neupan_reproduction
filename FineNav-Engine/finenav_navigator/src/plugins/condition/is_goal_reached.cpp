// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/condition/is_goal_reached.hpp"

#include <cmath>
#include <vector>
#include <behaviortree_cpp/bt_factory.h>

namespace finenav::navigator {

IsGoalReached::IsGoalReached(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsGoalReached::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Pose>("robot_pose",  "{@robot_pose}",  "Current robot pose"),
        BT::InputPort<geometry_msgs::msg::Twist>("robot_twist", "{@robot_twist}", "Current robot twist"),
        BT::InputPort<std::vector<geometry_msgs::msg::Pose>>(
            "goal_poses", "{@goal_poses}", "Goal poses; last element is the terminal goal"),

        BT::InputPort<double>("xy_tolerance",    0.25, "Horizontal distance [m]; 0 = skip"),
        BT::InputPort<double>("z_tolerance",     0.0,  "Vertical   distance [m]; 0 = skip"),
        BT::InputPort<double>("yaw_tolerance",   0.0,  "Yaw   angle [rad];      0 = skip"),
        BT::InputPort<double>("roll_tolerance",  0.0,  "Roll  angle [rad];      0 = skip"),
        BT::InputPort<double>("pitch_tolerance", 0.0,  "Pitch angle [rad];      0 = skip"),
        BT::InputPort<double>("speed_tolerance", 0.1,  "Linear speed [m/s];     0 = skip"),
    };
}

BT::NodeStatus IsGoalReached::tick() {
    geometry_msgs::msg::Pose robot_pose;
    std::vector<geometry_msgs::msg::Pose> goal_poses;
    if (!getInput("robot_pose", robot_pose) || !getInput("goal_poses", goal_poses) ||
        goal_poses.empty()) {
        return BT::NodeStatus::FAILURE;
    }
    // 判定目标是航点序列的最后一个点（终点）。
    const auto& goal = goal_poses.back();

    double xy_tol = 0.25, z_tol  = 0.0;
    double yaw_tol = 0.0, roll_tol = 0.0, pitch_tol = 0.0;
    double speed_tol = 0.1;

    getInput("xy_tolerance",    xy_tol);
    getInput("z_tolerance",     z_tol);
    getInput("yaw_tolerance",   yaw_tol);
    getInput("roll_tolerance",  roll_tol);
    getInput("pitch_tolerance", pitch_tol);
    getInput("speed_tolerance", speed_tol);

    // ── Position ──────────────────────────────────────────────────────
    const double dx = robot_pose.position.x - goal.position.x;
    const double dy = robot_pose.position.y - goal.position.y;

    if (xy_tol > 0.0) {
        const double xy_dist = std::sqrt(dx * dx + dy * dy);
        if (xy_dist >= xy_tol) return BT::NodeStatus::FAILURE;
    }
    if (z_tol > 0.0) {
        const double dz = std::abs(robot_pose.position.z - goal.position.z);
        if (dz >= z_tol) return BT::NodeStatus::FAILURE;
    }

    // ── Speed ─────────────────────────────────────────────────────────
    if (speed_tol > 0.0) {
        geometry_msgs::msg::Twist robot_twist;
        if (getInput("robot_twist", robot_twist)) {
            const double vx = robot_twist.linear.x;
            const double vy = robot_twist.linear.y;
            const double vz = robot_twist.linear.z;
            const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (speed >= speed_tol) return BT::NodeStatus::FAILURE;
        }
    }

    // ── Orientation ───────────────────────────────────────────────────
    if (yaw_tol > 0.0 || roll_tol > 0.0 || pitch_tol > 0.0) {
        double robot_roll, robot_pitch, robot_yaw;
        quaternionToRPY(robot_pose.orientation, robot_roll, robot_pitch, robot_yaw);

        double goal_roll, goal_pitch, goal_yaw;
        quaternionToRPY(goal.orientation, goal_roll, goal_pitch, goal_yaw);

        if (yaw_tol   > 0.0 && std::abs(normalizeAngle(robot_yaw   - goal_yaw))   >= yaw_tol)   return BT::NodeStatus::FAILURE;
        if (roll_tol  > 0.0 && std::abs(normalizeAngle(robot_roll  - goal_roll))  >= roll_tol)  return BT::NodeStatus::FAILURE;
        if (pitch_tol > 0.0 && std::abs(normalizeAngle(robot_pitch - goal_pitch)) >= pitch_tol) return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

void IsGoalReached::quaternionToRPY(const geometry_msgs::msg::Quaternion& q,
                                    double& roll, double& pitch, double& yaw) {
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    // roll  (x-axis rotation)
    const double sinr = 2.0 * (w * x + y * z);
    const double cosr = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr, cosr);
    // pitch (y-axis rotation)
    const double sinp = 2.0 * (w * y - z * x);
    pitch = (std::abs(sinp) >= 1.0)
                ? std::copysign(M_PI_2, sinp)
                : std::asin(sinp);
    // yaw   (z-axis rotation)
    const double siny = 2.0 * (w * z + x * y);
    const double cosy = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny, cosy);
}

double IsGoalReached::normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::IsGoalReached>("IsGoalReached");
}
