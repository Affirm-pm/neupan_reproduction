// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_route_planner/bfs_params.hpp"

#include "finenav_route_planner/astar_algorithm.hpp"   // BFS is A* with Heuristic::Zero
#include "finenav_route_planner/imap_view.hpp"

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::InputTrajectoryT;
using finenav::core::OutputTrajectoryT;
using finenav::core::Trajectory;

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

    using IMapView = astar::IMapView;

    using Context = PlanningContext<BfsPathSearch>;
    using Result  = std::optional<OutputTrajectoryT<BfsPathSearch>>;

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
