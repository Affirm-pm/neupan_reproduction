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
