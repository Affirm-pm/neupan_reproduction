// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// =============================================================
// snap_search_utils
//
// Shared "snap" primitives used by RoutePathSearch and
// DetourPlanner: obstacle-aware Dijkstra (h = 0) searches on the
// occupancy map that snap a world pose to the nearest REACHABLE
// member of a candidate point set, plus the world-frame connector
// path construction.
//
// Pattern:
//   1. Build a cell → candidate map with CellGrid::toCell and
//      track the max Chebyshev depth to bound the search box.
//   2. makeSnapSearch(...).findPathToAny(origin, is_goal)
//   3. buildConnector(pose, *path, grid, exact_end)
// =============================================================

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "finenav_route_planner/astar/astar_algorithm.hpp"
#include "finenav_route_planner/common/world_view.hpp"   // astar::IWorldView, finenav::common::Position3D

namespace finenav::snap {

using finenav::common::Position3D;

/// World ↔ grid-cell conversion at the map resolution.
struct CellGrid {
    double step;

    astar::Vec3 toCell(double x, double y) const {
        return {static_cast<int>(std::floor(x / step)),
                static_cast<int>(std::floor(y / step)), 0};
    }
    Position3D centre(const astar::Vec3& c) const {
        return {(c.x + 0.5) * step, (c.y + 0.5) * step, 0.0};
    }
};

inline int chebyshev(const astar::Vec3& a, const astar::Vec3& b)
{
    return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
}

/// Dijkstra (h=0) engine on the real occupancy map, bounded to a box of
/// `max_depth` cells around `origin` so an unreachable goal set cannot
/// trigger an unbounded expansion.
inline astar::AStarAlgorithm makeSnapSearch(
    const astar::IWorldView& map_view,
    const CellGrid&          grid,
    const astar::Vec3&       origin,
    int                      max_depth,
    double                   terrain_weight,
    double                   max_planning_time)
{
    auto is_walkable = [mv = &map_view, grid, origin, max_depth](const astar::Vec3& c) {
        return chebyshev(c, origin) <= max_depth &&
               mv->isWalkable(grid.centre(c));
    };
    // The new AStarAlgorithm has no setTerrainWeight(); fold the weight into
    // the cost callback so the edge weight stays
    // step * (movement_weight + terrain_weight * map_cost).
    auto get_cost = [mv = &map_view, grid, terrain_weight](const astar::Vec3& c) {
        return terrain_weight * mv->getCost(grid.centre(c));
    };

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::Connectivity::N8);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setMaxIterations(-1);
    algo.setMaxPlanningTime(max_planning_time);
    return algo;
}

/// Connector = exact start pose, then the grid path's cell centres
/// (cells[0] is the cell containing the pose and is skipped), then the
/// exact snap point.
inline std::vector<Position3D> buildConnector(
    const geometry_msgs::msg::Pose& pose,
    const std::vector<astar::Vec3>& cells,
    const CellGrid&                 grid,
    const Position3D&               end)
{
    std::vector<Position3D> conn;
    conn.reserve(cells.size() + 1);
    conn.push_back({pose.position.x, pose.position.y, 0.0});
    for (size_t i = 1; i < cells.size(); ++i)
        conn.push_back(grid.centre(cells[i]));
    if (conn.size() > 1)
        conn.back() = end;
    else
        conn.push_back(end);
    return conn;
}

} // namespace finenav::snap
