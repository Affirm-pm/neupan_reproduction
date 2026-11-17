// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <cmath>

#include "finenav_core/plugin_planner.hpp"
#include "finenav_route_planner/astar_params.hpp"

#include "finenav_route_planner/astar/astar_algorithm.hpp"
#include "finenav_route_planner/common/world_view.hpp"

namespace finenav {
using finenav::common::Position3D;
using finenav::common::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::InputTrajectoryT;
using finenav::core::OutputTrajectoryT;
using finenav::core::Trajectory;

/// Uniform-grid A* planner.
class AstarPathSearch {
public:
    AstarPathSearch() = default;

    // FineNav framework metadata
    using ConfigType = astar_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose  = true,   // goal = ref_traj.poses.back()
        .vel   = false,
        .accel = false,
        .time  = false
    };
    static constexpr IOProfile OutputProfile = {
        .pose  = true,
        .vel   = false,
        .accel = false,
        .time  = false
    };

    using IWorldView = astar::IWorldView;

    using Context = PlanningContext<AstarPathSearch>;
    using Result  = std::optional<OutputTrajectoryT<AstarPathSearch>>;

    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IWorldView& world_view);

private:
    astar::Connectivity connectivity_    = astar::Connectivity::N8;
    astar::Heuristic    heuristic_       = astar::Heuristic::Euclidean;
    double goal_tolerance_m_  = 0.5;
    int    max_iterations_    = 1000000;
    double max_planning_time_ = 5.0;
    double movement_weight_   = 1.0;
    double greedy_weight_     = 1.001;
};

} // namespace finenav

