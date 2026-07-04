// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cstdint>
#include "finenav_core/plan/trajectory.hpp"
#include "finenav_interfaces/msg/trajectory.hpp"

namespace finenav::core {

consteval uint32_t ProfileToMsgMask(const IOProfile& p) {
    uint32_t mask = 0;
    if (p.pose)  mask |= 1;
    if (p.vel)   mask |= 2;
    if (p.accel) mask |= 4;
    if (p.time)  mask |= 8;
    return mask;
}

// TODO: func: checkConsistency

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

template <IOProfile Config>
void toROSMsg(const Trajectory<Config>& src,
              finenav_interfaces::msg::Trajectory& dst)
{
    dst.valid_fields = ProfileToMsgMask(Config);
    if constexpr (Config.pose)  dst.poses = src.poses;
    if constexpr (Config.vel)   dst.twists = src.twists;
    if constexpr (Config.accel) dst.accels = src.accels;
    if constexpr (Config.time)  dst.time_from_start = src.times;
}

template <IOProfile Config>
void toROSMsg(Trajectory<Config>&& src,
              finenav_interfaces::msg::Trajectory& dst)
{
    dst.valid_fields = ProfileToMsgMask(Config);
    if constexpr (Config.pose)  dst.poses = std::move(src.poses);
    if constexpr (Config.vel)   dst.twists = std::move(src.twists);
    if constexpr (Config.accel) dst.accels = std::move(src.accels);
    if constexpr (Config.time)  dst.time_from_start = std::move(src.times);
}

template <IOProfile Config>
void fromROSMsg(const finenav_interfaces::msg::Trajectory& src,
                Trajectory<Config>& dst)
{
    uint32_t required = ProfileToMsgMask(Config);
    if ((src.valid_fields & required) != required) {
        throw std::runtime_error("Input trajectory message is missing required fields.");
    }
    if constexpr (Config.pose)  dst.poses = src.poses;
    if constexpr (Config.vel)   dst.twists = src.twists;
    if constexpr (Config.accel) dst.accels = src.accels;
    if constexpr (Config.time)  dst.times = src.time_from_start;
}

template <IOProfile Config>
void fromROSMsg(finenav_interfaces::msg::Trajectory&& src,
                Trajectory<Config>& dst)
{
    uint32_t required = ProfileToMsgMask(Config);
    if ((src.valid_fields & required) != required) {
        throw std::runtime_error("Input trajectory message is missing required fields.");
    }
    if constexpr (Config.pose)  dst.poses = std::move(src.poses);
    if constexpr (Config.vel)   dst.twists = std::move(src.twists);
    if constexpr (Config.accel) dst.accels = std::move(src.accels);
    if constexpr (Config.time)  dst.times = std::move(src.time_from_start);
}

} // namespace finenav::core
