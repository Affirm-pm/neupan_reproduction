// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <cmath>

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_route_planner/astar_params.hpp"

#include "astar_algorithm.hpp"
#include "imap_view.hpp"

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::InputTrajectoryT;
using finenav::core::OutputTrajectoryT;
using finenav::core::Trajectory;

// =============================================================
// AstarPathSearch
//
// FineNav planner that performs uniform-grid A* path search.
// =============================================================

class AstarPathSearch {
public:
    AstarPathSearch() = default;

    // ── FineNav framework metadata ────────────────────────────

    using ConfigType = astar_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose  = false,
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

    using IMapView = astar::IMapView;

    using Context = PlanningContext<AstarPathSearch>;
    using Result  = std::optional<OutputTrajectoryT<AstarPathSearch>>;

    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IMapView& map_view);

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

FINENAV_REGISTER_ALGO_CONFIG(finenav::AstarPathSearch, astar_planner)

