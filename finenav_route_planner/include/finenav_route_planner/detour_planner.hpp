// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include "finenav_core/plugin_planner.hpp"
#include "finenav_route_planner/detour_params.hpp"

#include "finenav_route_planner/common/world_view.hpp"   // astar::IWorldView — shared interface

namespace finenav {

using finenav::common::Position3D;
using finenav::common::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::OutputTrajectoryT;

// =============================================================
// DetourPlanner
//
// Re-anchors a reference trajectory at the robot and generates
// grid-search detour paths around dynamic obstacles.
//
// Designed to sit between a global route planner and MPPI in
// the FineNav pipeline.
//
// Output contract: the returned path ALWAYS starts at the robot
// pose.  The robot may have been displaced from the reference
// (manual override, tracking error) — every cycle an
// obstacle-aware connector re-attaches it to the reference.
//
// Flow in plan():
//   1. Scan the reference path forward from the waypoint nearest
//      to the robot (up to scan_distance_m).  A waypoint is
//      "blocked" when !isWalkable OR cost >= blockage_threshold.
//   2. Candidate set:
//        clear   → all waypoints from the nearest one onward
//        blocked → all waypoints at least reconnect_offset_m past
//                  the blocked waypoint
//   3. Obstacle-aware BFS (Dijkstra h=0, same primitive as
//      RoutePathSearch::snapToRoute) finds the nearest REACHABLE
//      candidate — ones still inside an obstacle are skipped
//      automatically.
//   4. Output = connector (robot → candidate) + reference suffix.
//   5. No candidate reachable → warn + pass the reference through
//      (upper layers handle the stuck condition).
//
// IWorldView cost contract:
//   getCost() MUST return normalised costs in [0.0, 1.0]
//   (e.g. raw TerrainAnalyzer cost / 254.0).
//   isWalkable() returns false for lethal obstacles.
// =============================================================

class DetourPlanner {
public:
    DetourPlanner() = default;

    // ── FineNav framework metadata ────────────────────────────

    using ConfigType = detour_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose  = true,
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

    using Context = PlanningContext<DetourPlanner>;
    using Result  = std::optional<OutputTrajectoryT<DetourPlanner>>;

    void configure(const ConfigType& params);
    void reset() {}

    /// Main entry point.
    /// Returns a trajectory that starts at the robot pose: an obstacle-aware
    /// connector onto the reference, detouring around blocked segments.
    Result plan(const Context& ctx, const IWorldView& world_view);

private:
    /// Scan the reference path forward from waypoint `scan_start` (up to
    /// scan_distance_m) and return the index of the first blocked waypoint
    /// (!isWalkable OR cost >= blockage_threshold).
    std::optional<size_t> findBlockage(
        const Context& ctx, const IWorldView& world_view, size_t scan_start) const;

    // ── parameters ────────────────────────────────────────────
    double scan_distance_m_    = 5.0;   // forward scan for blockage detection
    double blockage_threshold_ = 0.8;   // soft blockage: cost >= this triggers a detour
    double reconnect_offset_m_ = 1.0;   // rejoin the path at least this far past the blockage
    double terrain_weight_     = 1.0;   // terrain cost multiplier in the search edge weight
    double max_planning_time_  = 0.1;
};

} // namespace finenav
