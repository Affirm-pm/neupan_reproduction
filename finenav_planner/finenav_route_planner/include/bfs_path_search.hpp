// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include "finenav_core/concepts.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_route_planner/bfs_params.hpp"

#include "astar_algorithm.hpp"   // BFS is A* with Heuristic::Zero
#include "imap_view.hpp"

namespace finenav {

// =============================================================
// BfsPathSearch
//
// FineNav planner that finds the minimum-cost grid path without
// a directional heuristic (Dijkstra / weighted BFS).
//
// Internally this is AStarAlgorithm with Heuristic::Zero:
//   h(n) = 0  →  f(n) = g(n)  →  pure cost-based expansion.
//
// On a uniform-cost map this is equivalent to classic BFS and
// finds the fewest-hops path.  On a weighted map it finds the
// minimum traversal-cost path (same as Dijkstra).
//
// getCost() IS called, so map views should return meaningful values.
// =============================================================

class BfsPathSearch {
public:
    BfsPathSearch() = default;

    using ConfigType = bfs_planner::Params;

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

    using Context = PlanningContext<BfsPathSearch>;
    using Result  = std::optional<OutputDataT<BfsPathSearch>>;

    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IMapView& map_view);

private:
    // All fields mirror AstarPathSearch except heuristic_ is always Zero.
    astar::Connectivity connectivity_    = astar::Connectivity::N8;
    double goal_tolerance_m_  = 0.5;
    int    max_iterations_    = 1000000;
    double max_planning_time_ = 5.0;
    double movement_weight_   = 1.0;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::BfsPathSearch, bfs_planner)
