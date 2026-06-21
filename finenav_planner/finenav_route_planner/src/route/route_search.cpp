// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "route/route_search.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace route {

namespace {

// Append interpolated points from (x0,y0) to (x1,y1) at step `density`.
// The start point is only appended if it differs from the last point in `out`.
void densifySegment(
    double x0, double y0, double x1, double y1,
    double density,
    RoutePoses& out)
{
    auto append = [&](double x, double y) {
        if (out.empty() || std::hypot(out.back().first - x, out.back().second - y) > 1e-9)
            out.emplace_back(x, y);
    };

    const double dist = std::hypot(x1 - x0, y1 - y0);
    if (dist < 1e-9) { append(x0, y0); return; }

    const int    n    = std::max(1, static_cast<int>(std::ceil(dist / density)));
    const double step = dist / n;
    const double ux   = (x1 - x0) / dist;
    const double uy   = (y1 - y0) / dist;

    for (int i = 0; i < n; ++i)
        append(x0 + ux * step * i, y0 + uy * step * i);
}

}  // anonymous namespace

std::optional<RoutePoses> findRoute(
    const Graph& graph,
    size_t       start_index,
    size_t       goal_index,
    double       path_density)
{
    const size_t N = graph.nodes.size();
    if (start_index >= N || goal_index >= N) return std::nullopt;

    if (start_index == goal_index)
        return RoutePoses{{graph.nodes[start_index].x, graph.nodes[start_index].y}};

    // ── Dijkstra ──────────────────────────────────────────────────────────
    constexpr double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    std::vector<size_t> prev(N, SIZE_MAX);
    std::vector<size_t> prev_edge(N, SIZE_MAX);   // which Neighbor index was taken

    using QEntry = std::pair<double, size_t>;     // (cost, node_index)
    std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> pq;

    dist[start_index] = 0.0;
    pq.emplace(0.0, start_index);

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == goal_index) break;

        for (size_t k = 0; k < graph.adj[u].size(); ++k) {
            const Neighbor& nb = graph.adj[u][k];
            const double nd = d + nb.cost;
            if (nd < dist[nb.to_index]) {
                dist[nb.to_index] = nd;
                prev[nb.to_index]      = u;
                prev_edge[nb.to_index] = k;
                pq.emplace(nd, nb.to_index);
            }
        }
    }

    if (dist[goal_index] == INF) return std::nullopt;

    // ── Reconstruct edge sequence (from start to goal) ────────────────────
    // edges_fwd[i] = (from_node_index, neighbor_index_in_adj[from])
    std::vector<std::pair<size_t, size_t>> edges_fwd;
    for (size_t cur = goal_index; cur != start_index; ) {
        size_t p = prev[cur];
        edges_fwd.emplace_back(p, prev_edge[cur]);
        cur = p;
    }
    std::reverse(edges_fwd.begin(), edges_fwd.end());

    // ── Build output ──────────────────────────────────────────────────────
    RoutePoses result;

    if (path_density <= 0.0) {
        // Return only the node centres along the route.
        result.emplace_back(graph.nodes[start_index].x, graph.nodes[start_index].y);
        for (auto [fi, k] : edges_fwd) {
            const size_t ti = graph.adj[fi][k].to_index;
            result.emplace_back(graph.nodes[ti].x, graph.nodes[ti].y);
        }
        return result;
    }

    // Densify each edge's polyline.
    for (auto [fi, k] : edges_fwd) {
        const auto& pl = graph.adj[fi][k].polyline;
        for (size_t i = 0; i + 1 < pl.size(); ++i)
            densifySegment(pl[i].first, pl[i].second,
                           pl[i+1].first, pl[i+1].second,
                           path_density, result);
    }

    // Guarantee the exact goal position is the last point.
    const auto& gn = graph.nodes[goal_index];
    if (result.empty() ||
        std::hypot(result.back().first - gn.x, result.back().second - gn.y) > 1e-9)
        result.emplace_back(gn.x, gn.y);

    return result;
}

}  // namespace route

