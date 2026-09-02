// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_motion_decorator/trim_by_distance.hpp"

#include <cmath>

#include "finenav_core/common/algo_register_macros.hpp"

namespace finenav::motion_decorator {

TrimByDistance::TrimByDistance() = default;

void TrimByDistance::configure(const ConfigType& cfg) {
    max_distance_m_ = cfg.max_distance_m;
}

void TrimByDistance::reset() {}

TrimByDistance::Result TrimByDistance::plan(const Context& ctx) {
    using Trajectory = finenav::core::Trajectory<OutputProfile>;

    Trajectory result;
    result.poses = ctx.ref_traj.poses;

    if (result.poses.size() < 2) {
        return result;  // 0 or 1 point — no arc-length to compute, nothing to trim.
    }

    // Accumulate arc-length; stop at the first point that would exceed the limit.
    double accumulated = 0.0;
    std::size_t keep = result.poses.size();  // default: keep all

    for (std::size_t i = 1; i < result.poses.size(); ++i) {
        const auto& prev = result.poses[i - 1].position;
        const auto& curr = result.poses[i].position;

        const double dx = curr.x - prev.x;
        const double dy = curr.y - prev.y;
        const double dz = curr.z - prev.z;
        accumulated += std::sqrt(dx * dx + dy * dy + dz * dz);

        if (accumulated > max_distance_m_) {
            keep = i;  // keep points [0, i), i.e. up to but not including i.
            break;
        }
    }

    // Silently no-op: total path is within the limit.
    if (keep < result.poses.size()) {
        result.poses.resize(keep);
    }

    return result;
}

}  // namespace finenav::motion_decorator

FINENAV_REGISTER_PLANNER(finenav::motion_decorator::TrimByDistance, trim_by_distance)
