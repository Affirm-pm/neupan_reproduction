// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_motion_decorator/trim_by_aabb_params.hpp"

namespace finenav::motion_decorator {

/**
 * @brief Map-less planner that trims a trajectory by discarding all waypoints
 *        that lie outside an axis-aligned bounding box (AABB).
 *
 * The planner scans the trajectory from the front and retains only the
 * contiguous prefix of waypoints that are contained within the box.
 * The first waypoint found outside the box — and all following waypoints —
 * are removed.
 *
 * The box is configured entirely via parameters (min_* / max_*), so it needs no
 * map.  Default extents are huge (±1e9), making the planner a no-op until the
 * box is narrowed via parameters.
 *
 * - Silently no-ops if all waypoints are inside the box.
 */
class TrimByAABB {
public:
    using ConfigType = trim_by_aabb::Params;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    TrimByAABB();
    void configure(const ConfigType& cfg);
    void reset();

    using Context = finenav::core::PlanningContext<TrimByAABB>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<TrimByAABB>>;

    Result plan(const Context& ctx);

private:
    double min_x_{-1e9}, min_y_{-1e9}, min_z_{-1e9};
    double max_x_{1e9},  max_y_{1e9},  max_z_{1e9};
};

}  // namespace finenav::motion_decorator
