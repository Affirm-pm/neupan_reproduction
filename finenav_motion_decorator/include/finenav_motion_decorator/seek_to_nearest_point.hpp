// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_motion_decorator/seek_to_nearest_point_params.hpp"

namespace finenav::motion_decorator {

/**
 * @brief Map-less planner that advances the trajectory start to the waypoint
 *        nearest to the robot's current position, discarding all preceding points.
 *
 * "Seek" mirrors file-seek semantics: the read head is repositioned to the
 * nearest entry rather than truncating the tail.
 *
 * The reference position is taken from ctx.robot_state (the robot's live pose),
 * so it needs no map and no goal — only the input reference trajectory.
 *
 * Nearest-point search is performed in the XY plane by default (use_2d = true),
 * which is more robust on uneven terrain.
 */
class SeekToNearestPoint {
public:
    using ConfigType = seek_to_nearest_point::Params;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    SeekToNearestPoint();
    void configure(const ConfigType& cfg);
    void reset();

    using Context = finenav::core::PlanningContext<SeekToNearestPoint>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<SeekToNearestPoint>>;

    Result plan(const Context& ctx);

private:
    bool use_2d_{true};
};

}  // namespace finenav::motion_decorator
