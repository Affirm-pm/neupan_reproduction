// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_motion_decorator/trim_by_aabb.hpp"

#include "finenav_core/common/algo_register_macros.hpp"

namespace finenav::motion_decorator {

TrimByAABB::TrimByAABB() = default;

void TrimByAABB::configure(const ConfigType& cfg) {
    min_x_ = cfg.min_x;
    min_y_ = cfg.min_y;
    min_z_ = cfg.min_z;
    max_x_ = cfg.max_x;
    max_y_ = cfg.max_y;
    max_z_ = cfg.max_z;
}

void TrimByAABB::reset() {}

TrimByAABB::Result TrimByAABB::plan(const Context& ctx) {
    using Trajectory = finenav::core::Trajectory<OutputProfile>;

    Trajectory result;
    result.poses = ctx.ref_traj.poses;

    // Find the first waypoint outside the bounding box; keep [0, keep).
    std::size_t keep = result.poses.size();  // default: keep all
    for (std::size_t i = 0; i < result.poses.size(); ++i) {
        const auto& p = result.poses[i].position;
        const bool inside = (p.x >= min_x_ && p.x <= max_x_) &&
                            (p.y >= min_y_ && p.y <= max_y_) &&
                            (p.z >= min_z_ && p.z <= max_z_);
        if (!inside) {
            keep = i;
            break;
        }
    }

    // Silently no-op: all waypoints are inside the box.
    if (keep < result.poses.size()) {
        result.poses.resize(keep);
    }

    return result;
}

}  // namespace finenav::motion_decorator

FINENAV_REGISTER_PLANNER(finenav::motion_decorator::TrimByAABB, trim_by_aabb)
