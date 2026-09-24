/*
 * neupan_cpp_ros: ROS2 wrapper for libneupan, the C++ port of NeuPAN.
 *
 * Message helpers shared by neupan_node and astar_global_node.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>

namespace neupan_ros {

inline double quatToYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.z * q.z + q.y * q.y));
}

inline geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

inline geometry_msgs::msg::PoseStamped planarPose(
    const std_msgs::msg::Header& header, double x, double y, double yaw) {
  geometry_msgs::msg::PoseStamped ps;
  ps.header = header;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation = yawToQuat(yaw);
  return ps;
}

}  // namespace neupan_ros
