// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_motion_decorator/trim_by_count_params.hpp"

namespace finenav::motion_decorator {

/**
 * @brief Map-less planner that trims a trajectory to at most `max_points`
 *        waypoints, counting from the beginning of the path.
 *
 * - Silently no-ops when the trajectory already has <= max_points waypoints.
 */
class TrimByCount {
public:
    using ConfigType = trim_by_count::Params;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    TrimByCount();
    void configure(const ConfigType& cfg);
    void reset();

    using Context = finenav::core::PlanningContext<TrimByCount>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<TrimByCount>>;

    Result plan(const Context& ctx);

private:
    std::size_t max_points_{100};
};

}  // namespace finenav::motion_decorator
