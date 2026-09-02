// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_motion_decorator/seek_to_nearest_point.hpp"

#include <cstddef>
#include <limits>

#include "finenav_core/common/algo_register_macros.hpp"

namespace finenav::motion_decorator {

SeekToNearestPoint::SeekToNearestPoint() = default;

void SeekToNearestPoint::configure(const ConfigType& cfg) {
    use_2d_ = cfg.use_2d;
}

void SeekToNearestPoint::reset() {}

SeekToNearestPoint::Result SeekToNearestPoint::plan(const Context& ctx) {
    using Trajectory = finenav::core::Trajectory<OutputProfile>;

    Trajectory result;
    result.poses = ctx.ref_traj.poses;

    if (result.poses.empty()) {
        return result;
    }

    const auto& ref = ctx.robot_state.pose.position;

    std::size_t nearest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < result.poses.size(); ++i) {
        const auto& p = result.poses[i].position;
        const double dx = p.x - ref.x;
        const double dy = p.y - ref.y;
        const double dz = use_2d_ ? 0.0 : (p.z - ref.z);
        const double dist_sq = dx * dx + dy * dy + dz * dz;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = i;
        }
    }

    // Silently no-op: nearest point is already the first waypoint.
    if (nearest_idx == 0) {
        return result;
    }

    result.poses.erase(
        result.poses.begin(),
        result.poses.begin() + static_cast<std::ptrdiff_t>(nearest_idx));
    return result;
}

}  // namespace finenav::motion_decorator

FINENAV_REGISTER_PLANNER(finenav::motion_decorator::SeekToNearestPoint, seek_to_nearest_point)
