// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/chassis_cmd_vel.hpp"

#include <geometry_msgs/msg/twist.hpp>

namespace finenav::navigator {

ChassisCmdVel::ChassisCmdVel(rclcpp::Node::SharedPtr node,
                             const std::string& topic) {
    pub_ = node->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
}

void ChassisCmdVel::publish(const finenav_interfaces::msg::Trajectory& traj) {
    if (traj.twists.empty()) return;
    pub_->publish(traj.twists[0]);
}

}  // namespace finenav::navigator
