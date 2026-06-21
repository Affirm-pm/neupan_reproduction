// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <vector>

#include "route/route_graph.hpp"

namespace route {

// =============================================================
// findRoute
//
// Dijkstra's shortest-path on a route Graph, followed by
// polyline densification to produce a dense pose list.
//
// Parameters:
//   graph        — route graph built by loadFromJson().
//   start_index  — index in graph.nodes for the start node.
//   goal_index   — index in graph.nodes for the goal node.
//   path_density — interpolation step in metres.
//                  Pass 0 (or negative) to return only node centres.
//
// Returns:
//   A list of [x, y] world-frame points along the route,
//   or nullopt if no path exists (disconnected graph).
// =============================================================

using RoutePoses = std::vector<std::pair<double, double>>;

std::optional<RoutePoses> findRoute(
    const Graph& graph,
    size_t       start_index,
    size_t       goal_index,
    double       path_density = 0.05);

}  // namespace route

