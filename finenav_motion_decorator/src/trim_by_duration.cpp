// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_motion_decorator/trim_by_duration.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "finenav_core/common/algo_register_macros.hpp"

namespace finenav::motion_decorator {

namespace {

double duration_to_sec(const builtin_interfaces::msg::Duration& d) {
    return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

}  // namespace

TrimByDuration::TrimByDuration() = default;

void TrimByDuration::configure(const ConfigType& cfg) {
    max_duration_sec_ = cfg.max_duration_sec;
}

void TrimByDuration::reset() {}

TrimByDuration::Result TrimByDuration::plan(const Context& ctx) {
    using Trajectory = finenav::core::Trajectory<OutputProfile>;

    Trajectory result;
    result.poses = ctx.ref_traj.poses;
    result.times = ctx.ref_traj.times;

    if (result.times.empty()) {
        return result;  // nothing to trim.
    }

    // Find the first index whose time_from_start exceeds the limit.
    std::size_t keep = result.times.size();
    for (std::size_t i = 0; i < result.times.size(); ++i) {
        if (duration_to_sec(result.times[i]) > max_duration_sec_) {
            keep = i;
            break;
        }
    }

    // Silently no-op: all points are already within the limit.
    if (keep < result.times.size()) {
        result.poses.resize(keep);
        result.times.resize(keep);
    }

    return result;
}

}  // namespace finenav::motion_decorator

FINENAV_REGISTER_PLANNER(finenav::motion_decorator::TrimByDuration, trim_by_duration)
