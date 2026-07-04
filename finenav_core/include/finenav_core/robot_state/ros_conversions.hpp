// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_interfaces/msg/robot_state.hpp"

namespace finenav::core {

/// Convert Eigen RobotState → ROS message format.
inline void toROSMsg(const RobotState& src, finenav_interfaces::msg::RobotState& dst) {
    dst.pose.position.x    = src.position.x();
    dst.pose.position.y    = src.position.y();
    dst.pose.position.z    = src.position.z();
    dst.pose.orientation.w = src.orientation.w();
    dst.pose.orientation.x = src.orientation.x();
    dst.pose.orientation.y = src.orientation.y();
    dst.pose.orientation.z = src.orientation.z();
    dst.twist.linear.x     = src.linear_velocity.x();
    dst.twist.linear.y     = src.linear_velocity.y();
    dst.twist.linear.z     = src.linear_velocity.z();
    dst.twist.angular.x    = src.angular_velocity.x();
    dst.twist.angular.y    = src.angular_velocity.y();
    dst.twist.angular.z    = src.angular_velocity.z();
}

}  // namespace finenav::core
