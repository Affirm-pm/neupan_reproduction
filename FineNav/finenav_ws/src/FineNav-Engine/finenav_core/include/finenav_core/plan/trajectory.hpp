// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>

#include "builtin_interfaces/msg/duration.hpp"

namespace finenav::core {

struct IOProfile {
    bool pose = false;
    bool vel  = false;
    bool accel= false;
    bool time = false;
};

namespace detail {

// Per-field storage blocks.  Each is empty when its field is disabled, so a
// Trajectory only pays for the fields its IOProfile turns on.

template <bool Enable> struct PoseMixin;
template <> struct PoseMixin<true> { std::vector<geometry_msgs::msg::Pose> poses; };
template <> struct PoseMixin<false> {};

template <bool Enable> struct VelMixin;
template <> struct VelMixin<true> { std::vector<geometry_msgs::msg::Twist> twists; };
template <> struct VelMixin<false> {};

template <bool Enable> struct AccelMixin;
template <> struct AccelMixin<true> { std::vector<geometry_msgs::msg::Accel> accels; };
template <> struct AccelMixin<false> {};

template <bool Enable> struct TimeMixin;
template <> struct TimeMixin<true> { std::vector<builtin_interfaces::msg::Duration> times; };
template <> struct TimeMixin<false> {};

}  // namespace detail

/// A planner's trajectory: a sequence of waypoints with compile-time field
/// selection.  Only the fields enabled in the `Profile` exist at runtime —
/// a planner that does not need velocity/acceleration pays no memory cost.
template <IOProfile Profile>
struct Trajectory :
    public detail::PoseMixin<Profile.pose>,
    public detail::VelMixin<Profile.vel>,
    public detail::AccelMixin<Profile.accel>,
    public detail::TimeMixin<Profile.time>
{};

/// Alias for a planner's output trajectory type.
template <typename PlannerT>
using OutputTrajectoryT = Trajectory<PlannerT::OutputProfile>;

/// Alias for a planner's input (reference) trajectory type.
template <typename PlannerT>
using InputTrajectoryT = Trajectory<PlannerT::InputProfile>;

} // namespace finenav::core


