// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Shared test infrastructure for finenav_core/robot_state unit tests.

#pragma once

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "finenav_core/robot_state/robot_state.hpp"

/// seconds → rclcpp::Time (ROS-clock integer-second stamp).
inline rclcpp::Time ts(int32_t seconds) { return rclcpp::Time(seconds, 0, RCL_ROS_TIME); }
/// seconds with sub-second fraction (e.g. 1.5 s).
inline rclcpp::Time tf(double seconds)
{
    auto s  = static_cast<int32_t>(seconds);
    auto ns = static_cast<uint32_t>((seconds - s) * 1e9);
    return rclcpp::Time(s, ns, RCL_ROS_TIME);
}

/// Thin wrapper that promotes the protected feed/reset methods for testing.
class TestProvider : public finenav::core::RobotStateProvider {
public:
    using finenav::core::RobotStateProvider::RobotStateProvider;
    using finenav::core::RobotStateProvider::pushState;
    using finenav::core::RobotStateProvider::resetHistory;
    using finenav::core::RobotStateProvider::setHistoryDuration;
};

inline finenav::core::RobotState makeState(double x, double y, double yaw)
{
    finenav::core::RobotState s;
    s.position         = {x, y, 0.0};
    s.orientation      = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    s.linear_velocity  = {0.1, 0.0, 0.0};
    s.angular_velocity = {0.0, 0.0, 0.05};
    return s;
}
