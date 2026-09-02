// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_motion_decorator/trim_by_distance_params.hpp"

namespace finenav::motion_decorator {

/**
 * @brief Map-less planner that trims a trajectory so that the cumulative
 *        arc-length along the path does not exceed `max_distance_m`.
 *
 * Distance is computed as the sum of consecutive Euclidean distances between
 * pose positions (x, y, z).
 *
 * - Silently no-ops if the total path length is already within the limit.
 */
class TrimByDistance {
public:
    using ConfigType = trim_by_distance::Params;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    TrimByDistance();
    void configure(const ConfigType& cfg);
    void reset();

    using Context = finenav::core::PlanningContext<TrimByDistance>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<TrimByDistance>>;

    Result plan(const Context& ctx);

private:
    double max_distance_m_{5.0};
};

}  // namespace finenav::motion_decorator
