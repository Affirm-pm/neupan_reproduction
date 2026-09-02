// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_motion_decorator/trim_by_count.hpp"

#include "finenav_core/common/algo_register_macros.hpp"

namespace finenav::motion_decorator {

TrimByCount::TrimByCount() = default;

void TrimByCount::configure(const ConfigType& cfg) {
    max_points_ = static_cast<std::size_t>(cfg.max_points);
}

void TrimByCount::reset() {}

TrimByCount::Result TrimByCount::plan(const Context& ctx) {
    using Trajectory = finenav::core::Trajectory<OutputProfile>;

    Trajectory result;
    result.poses = ctx.ref_traj.poses;

    // Silently no-op: already within the limit or trajectory is empty.
    if (result.poses.size() <= max_points_) {
        return result;
    }

    result.poses.resize(max_points_);
    return result;
}

}  // namespace finenav::motion_decorator

FINENAV_REGISTER_PLANNER(finenav::motion_decorator::TrimByCount, trim_by_count)
