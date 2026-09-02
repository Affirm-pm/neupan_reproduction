// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_motion_decorator/trim_by_duration_params.hpp"

namespace finenav::motion_decorator {

/**
 * @brief Map-less planner that trims a trajectory to only waypoints whose
 *        time_from_start is strictly less than or equal to `max_duration_sec`.
 *
 * Consumes and produces both pose and time fields; they are trimmed consistently.
 *
 * - Silently no-ops if the trajectory is already within the limit.
 */
class TrimByDuration {
public:
    using ConfigType = trim_by_duration::Params;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, true};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, true};

    TrimByDuration();
    void configure(const ConfigType& cfg);
    void reset();

    using Context = finenav::core::PlanningContext<TrimByDuration>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<TrimByDuration>>;

    Result plan(const Context& ctx);

private:
    double max_duration_sec_{10.0};
};

}  // namespace finenav::motion_decorator
