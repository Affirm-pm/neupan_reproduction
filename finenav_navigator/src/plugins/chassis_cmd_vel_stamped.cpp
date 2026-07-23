// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/chassis_cmd_vel_stamped.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace finenav::navigator {

ChassisCmdVelStamped::ChassisCmdVelStamped(rclcpp::Node::SharedPtr node,
                                           const std::string& topic)
    : node_(std::move(node)) {
    pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic, 10);
}

void ChassisCmdVelStamped::publish(const finenav_interfaces::msg::Trajectory& traj) {
    if (traj.twists.empty()) return;
    geometry_msgs::msg::TwistStamped msg;
    msg.twist = traj.twists[0];
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";
    pub_->publish(msg);
}

}  // namespace finenav::navigator
