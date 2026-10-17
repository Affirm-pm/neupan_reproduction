// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_planner.hpp"

#include <cmath>

#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {

DummyPlanner::DummyPlanner() = default;

void DummyPlanner::configure(const ConfigType& cfg) {
    // Use the framework-injected logger (bound to the ControlLayer's node)
    RCLCPP_INFO(logger_,
        "Params updated: linear_speed=%.2f -> %.2f, angular_speed=%.2f -> %.2f",
        linear_speed_, cfg.linear_speed, angular_speed_, cfg.angular_speed);

    linear_speed_  = cfg.linear_speed;
    angular_speed_ = cfg.angular_speed;
}

void DummyPlanner::reset() {}

DummyPlanner::Result DummyPlanner::plan(const Context& ctx,const IWorldView& map_view) {

    using Trajectory = finenav::core::Trajectory<OutputProfile>; // TODO：这个写法不错，很漂亮

    if (ctx.ref_traj.poses.empty()) return std::nullopt;
    const auto& goal = ctx.ref_traj.poses.back();
    double dx = goal.position.x - ctx.robot_state.pose.position.x;
    double dy = goal.position.y - ctx.robot_state.pose.position.y;

    double desired_yaw = std::atan2(dy, dx);

    const auto& q = ctx.robot_state.pose.orientation;
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double current_yaw = std::atan2(siny, cosy);

    double yaw_error = desired_yaw - current_yaw;
    while (yaw_error >  M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear_speed_ * std::max(0.0, 1.0 - std::abs(yaw_error) / 1.5);
    cmd.angular.z = angular_speed_ * yaw_error;

    Trajectory result;
    result.twists.push_back(cmd);
    return result;
}

}  // namespace finenav::example

// One-line registration: parameter wiring + plugin self-registration.
// "dummy_planner" must match the top-level key in dummy_planner.yaml.
FINENAV_REGISTER_PLANNER(finenav::example::DummyPlanner, dummy_planner)
