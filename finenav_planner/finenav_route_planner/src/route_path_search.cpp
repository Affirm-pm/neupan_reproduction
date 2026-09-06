// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/route_path_search.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>

#include <geometry_msgs/msg/pose.hpp>

#include "finenav_core/common/algo_register_macros.hpp"

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────
//
// Called whenever a dynamic parameter changes.
// Reloads the graph only when graph_file actually changes.

void RoutePathSearch::configure(const ConfigType& params)
{
    path_density_             = params.path_density;
    min_approach_angle_deg_   = params.min_approach_angle_deg;
}

// ── plan ───────────────────────────────────────────────────────────────────

RoutePathSearch::Result RoutePathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    const auto* graph_ptr = map_view.getGraph();
    if (!graph_ptr || graph_ptr->nodes.empty()) return std::nullopt;
    if (ctx.ref_traj.poses.empty()) return std::nullopt;
    const auto& orig_graph = *graph_ptr;

    // Obstacle-aware BFS to snap start and goal to nearest reachable
    // graph points (nodes or intermediate path points).
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = ctx.robot_state.pose.position.x;
    start_pose.position.y = ctx.robot_state.pose.position.y;
    start_pose.position.z = ctx.robot_state.pose.position.z;
    start_pose.orientation.w = ctx.robot_state.pose.orientation.w;
    start_pose.orientation.x = ctx.robot_state.pose.orientation.x;
    start_pose.orientation.y = ctx.robot_state.pose.orientation.y;
    start_pose.orientation.z = ctx.robot_state.pose.orientation.z;
    const auto start_snap = findNearestGraphPoint(orig_graph, start_pose, map_view);
    if (!start_snap.isNode() && !start_snap.isPath()) return std::nullopt;

    const auto goal_snap =
        findNearestGraphPoint(orig_graph, ctx.ref_traj.poses.back(), map_view);
    if (!goal_snap.isNode() && !goal_snap.isPath()) return std::nullopt;

    // ── Build mutable working graph ──────────────────────────────────────
    // We may need to insert virtual nodes and split edges for path-snapped
    // entry/exit points.  Work on a copy so the cached graph is unaffected.
    route::Graph graph = orig_graph;   // deep copy

    size_t start_node_idx = SIZE_MAX;
    size_t goal_node_idx  = SIZE_MAX;

    // Helper: convert a SnapResult into an effective node index in `graph`.
    // For node snaps the original index is still valid (we only append).
    // For path snaps we insert a virtual node and split the edge.
    auto resolveSnap = [&](const SnapResult& snap, bool is_start) -> size_t {
        if (snap.isNode()) {
            return snap.node_index;
        }
        // Path snap — insert virtual node and split the edge
        const size_t vi = insertVirtualNode(graph, snap.snap_point);
        splitEdgeForSnap(graph, snap, vi);
        return vi;
    };

    // Process snaps in order of pt_idx (decreasing) when they land on the
    // same edge, so that earlier splits don't invalidate later indices.
    // Easiest: process start first, then goal.  splitEdgeForSnap searches
    // by to_index and polyline bounds so it is robust to index shifts.
    start_node_idx = resolveSnap(start_snap, true);
    goal_node_idx  = resolveSnap(goal_snap, false);

    // ── Dijkstra on the (possibly modified) topological graph ────────────
    auto route_opt = route::findRoute(graph, start_node_idx, goal_node_idx, path_density_);
    if (!route_opt || route_opt->empty()) return std::nullopt;

    OutputTrajectoryT<RoutePathSearch> output;
    output.poses.reserve(start_snap.connector_path.size()
                       + goal_snap.connector_path.size()
                       + route_opt->size());

    auto appendPose = [&](double x, double y) {
        geometry_msgs::msg::Pose p;
        p.position.x    = x;
        p.position.y    = y;
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    };

    auto isDegenerate = [](const std::vector<Position3D>& conn) -> bool {
        if (conn.size() < 2) return true;
        const double d = (conn.front() - conn.back()).norm();
        return d < 1e-6;
    };

    // ── Start connector (robot_pose → snap_point) ──
    if (!isDegenerate(start_snap.connector_path)) {
        for (const auto& pt : start_snap.connector_path)
            appendPose(pt.x(), pt.y());
    }

    // ── Topological route ──
    // Skip the first route point if it coincides with the last connector point.
    bool skip_first_route = false;
    if (!isDegenerate(start_snap.connector_path) && !output.poses.empty()) {
        const auto& last_conn = output.poses.back().position;
        const auto& first_route_x = (*route_opt)[0].first;
        const auto& first_route_y = (*route_opt)[0].second;
        skip_first_route = (std::hypot(last_conn.x - first_route_x,
                                        last_conn.y - first_route_y) < 1e-6);
    }

    for (size_t i = (skip_first_route ? 1 : 0); i < route_opt->size(); ++i)
        appendPose((*route_opt)[i].first, (*route_opt)[i].second);

    // ── Goal connector (snap_point → final ref pose), reversed ──
    if (!isDegenerate(goal_snap.connector_path)) {
        const auto& route_end = output.poses.back().position;
        const auto& goal_last = goal_snap.connector_path.back();  // snap point
        const bool route_ends_at_snap =
            std::hypot(route_end.x - goal_last.x(),
                       route_end.y - goal_last.y()) < 1e-6;

        const size_t start_i = route_ends_at_snap
            ? goal_snap.connector_path.size() - 2
            : goal_snap.connector_path.size() - 1;

        for (size_t i = start_i; i != SIZE_MAX; --i)
            appendPose(goal_snap.connector_path[i].x(),
                       goal_snap.connector_path[i].y());
    }

    return output;
}

// ── findNearestGraphPoint ───────────────────────────────────────────────────
//
// Runs AStarAlgorithm::findPathToAny (h=0, Dijkstra) on the real occupancy
// map.  The search expands from the cell containing `pose` outward until it
// finds a cell that contains at least one route graph node OR intermediate
// point on an edge polyline (a "path point").
//
// Using the real map means the connector is obstacle-aware: a point that can
// only be reached by going through a wall will not be selected.
//
// Cell size = map_view.getStepSize() (map resolution) so the BFS is in the
// same cell space as AstarPathSearch / BfsPathSearch.
//
// Returns a SnapResult with the entry info and the world-frame connector
// path from `pose` to that entry point.

namespace {
struct PathPointInfo {
    size_t                from_node;
    size_t                to_node;
    size_t                edge_idx;   // index in graph.adj[from_node]
    size_t                pt_idx;     // index in the edge's polyline
    finenav::Position3D   world_pos;
};
}  // anonymous namespace

RoutePathSearch::SnapResult RoutePathSearch::findNearestGraphPoint(
    const route::Graph&             graph,
    const geometry_msgs::msg::Pose& pose,
    const IMapView&                 map_view) const
{
    const double step = map_view.getStepSize();

    auto toCell = [step](double x, double y) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / step)),
            static_cast<int>(std::floor(y / step)),
            0
        };
    };
    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return { (c.x + 0.5) * step, (c.y + 0.5) * step, 0.0 };
    };

    const astar::Vec3 origin = toCell(pose.position.x, pose.position.y);

    // Build cell → node-index map
    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_nodes;
    int max_depth = 0;
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        const astar::Vec3 c = toCell(graph.nodes[i].x, graph.nodes[i].y);
        cell_nodes[c].push_back(i);
        max_depth = std::max(max_depth,
            std::max(std::abs(c.x - origin.x), std::abs(c.y - origin.y)));
    }

    // Build cell → path-point-info map from intermediate polyline points.
    // Skip the first (0) and last (size-1) points — those coincide with nodes.
    std::unordered_map<astar::Vec3, std::vector<PathPointInfo>, astar::Vec3Hash> cell_path_pts;
    for (size_t fi = 0; fi < graph.adj.size(); ++fi) {
        for (size_t ei = 0; ei < graph.adj[fi].size(); ++ei) {
            const auto& pl = graph.adj[fi][ei].polyline;
            if (pl.size() <= 2) continue;  // no intermediate points
            const size_t to_idx = graph.adj[fi][ei].to_index;
            for (size_t pi = 1; pi + 1 < pl.size(); ++pi) {
                const astar::Vec3 c = toCell(pl[pi].first, pl[pi].second);
                cell_path_pts[c].push_back({
                    fi, to_idx, ei, pi,
                    {pl[pi].first, pl[pi].second, 0.0}
                });
                max_depth = std::max(max_depth,
                    std::max(std::abs(c.x - origin.x), std::abs(c.y - origin.y)));
            }
        }
    }

    // Obstacle-aware lambdas — use real map data
    auto is_walkable = [&](const astar::Vec3& c) {
        return c.z == 0 &&
               std::abs(c.x - origin.x) <= max_depth &&
               std::abs(c.y - origin.y) <= max_depth &&
               map_view.isWalkable(cellCentre(c));
    };
    auto get_cost = [&](const astar::Vec3& c) {
        return map_view.getCost(cellCentre(c));
    };
    auto is_goal = [&](const astar::Vec3& c) {
        return cell_nodes.contains(c) || cell_path_pts.contains(c);
    };

    // ── Angle filter for path-point snapping ─────────────────────────────
    // Rejects goal cells whose path-point candidates all have approach angles
    // below min_approach_angle_deg_, causing the Dijkstra to continue past them.
    // Nodes are always accepted (no angle check for nodes).
    astar::AStarAlgorithm::GoalFilterFn accept_goal = nullptr;
    if (min_approach_angle_deg_ > 0.0) {
        accept_goal = [&](const astar::Vec3& c) -> bool {
            // Nodes are always acceptable
            if (cell_nodes.contains(c)) return true;
            // For path-point cells, check whether any candidate passes
            if (cell_path_pts.contains(c)) {
                for (const auto& ppi : cell_path_pts.at(c)) {
                    // approach direction: robot pose → candidate point
                    const double adx = ppi.world_pos.x() - pose.position.x;
                    const double ady = ppi.world_pos.y() - pose.position.y;
                    const double ad_norm = std::hypot(adx, ady);
                    if (ad_norm < 1e-9) continue;
                    // path tangent: polyline[pt+1] − polyline[pt−1]
                    const auto& pl = graph.adj[ppi.from_node][ppi.edge_idx].polyline;
                    const double tdx = pl[ppi.pt_idx + 1].first  - pl[ppi.pt_idx - 1].first;
                    const double tdy = pl[ppi.pt_idx + 1].second - pl[ppi.pt_idx - 1].second;
                    const double td_norm = std::hypot(tdx, tdy);
                    if (td_norm < 1e-9) continue;
                    const double cos_angle = (adx * tdx + ady * tdy) / (ad_norm * td_norm);
                    const double angle_deg =
                        std::acos(std::clamp(cos_angle, -1.0, 1.0)) * 180.0 / M_PI;
                    if (angle_deg >= min_approach_angle_deg_) return true;
                }
                return false;   // no path point in this cell passes
            }
            return false;   // neither node nor path point (should not happen)
        };
    }

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::Connectivity::N8);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setMaxIterations(-1);
    algo.setMaxPlanningTime(5.0); // TODO: 后面这些应该也成为动态参数

    const auto path = algo.findPathToAny(origin, is_goal, accept_goal);
    if (!path || path->empty()) return {};

    const astar::Vec3& goal_cell = path->back();

    SnapResult result;

    // ── Find the geometrically closest candidate among both nodes and path points ──
    double best_dist = std::numeric_limits<double>::infinity();
    bool   found_node = false;

    // Check nodes in the goal cell
    if (cell_nodes.contains(goal_cell)) {
        for (const size_t idx : cell_nodes.at(goal_cell)) {
            const double d = std::hypot(graph.nodes[idx].x - pose.position.x,
                                        graph.nodes[idx].y - pose.position.y);
            if (d < best_dist ||
                (std::abs(d - best_dist) < 1e-9 &&
                 graph.nodes[idx].id < graph.nodes[result.node_index].id))
            {
                result.node_index = idx;
                best_dist = d;
                found_node = true;
            }
        }
    }

    // Check path points in the goal cell
    if (cell_path_pts.contains(goal_cell)) {
        for (const auto& ppi : cell_path_pts.at(goal_cell)) {
            // ── Angle filter: skip path points whose approach angle is too small ──
            if (min_approach_angle_deg_ > 0.0) {
                const double adx = ppi.world_pos.x() - pose.position.x;
                const double ady = ppi.world_pos.y() - pose.position.y;
                const double ad_norm = std::hypot(adx, ady);
                if (ad_norm > 1e-9) {
                    const auto& pl = graph.adj[ppi.from_node][ppi.edge_idx].polyline;
                    const double tdx = pl[ppi.pt_idx + 1].first  - pl[ppi.pt_idx - 1].first;
                    const double tdy = pl[ppi.pt_idx + 1].second - pl[ppi.pt_idx - 1].second;
                    const double td_norm = std::hypot(tdx, tdy);
                    if (td_norm > 1e-9) {
                        const double cos_angle = (adx * tdx + ady * tdy) / (ad_norm * td_norm);
                        const double angle_deg =
                            std::acos(std::clamp(cos_angle, -1.0, 1.0)) * 180.0 / M_PI;
                        if (angle_deg < min_approach_angle_deg_) continue;  // skip this candidate
                    }
                }
            }

            const double d = std::hypot(ppi.world_pos.x() - pose.position.x,
                                        ppi.world_pos.y() - pose.position.y);
            if (d < best_dist) {
                // If switching from node to path point, reset node_index
                if (found_node) {
                    result.node_index = SIZE_MAX;
                    found_node = false;
                }
                result.edge_from  = ppi.from_node;
                result.edge_to    = ppi.to_node;
                result.edge_idx   = ppi.edge_idx;
                result.pt_idx     = ppi.pt_idx;
                result.snap_point = ppi.world_pos;
                best_dist = d;
            }
        }
    }

    // ── Build world-frame connector path (pose → snap_point) ──────────
    std::vector<Position3D> connector;
    connector.reserve(path->size() + 1);

    // First point: exact continuous pose
    connector.push_back({pose.position.x, pose.position.y, 0.0});

    // Interior grid cells → cell centres (skip origin if it coincides)
    const bool skip_first = (path->size() >= 2 && (*path)[0] == origin);
    for (size_t i = (skip_first ? 1 : 0); i < path->size(); ++i) {
        connector.push_back(cellCentre((*path)[i]));
    }

    // Last point: replace with the exact snap point position
    if (result.isNode()) {
        connector.back() = {graph.nodes[result.node_index].x,
                           graph.nodes[result.node_index].y, 0.0};
    } else {
        connector.back() = result.snap_point;
    }

    result.connector_path = std::move(connector);
    return result;
}

// ── insertVirtualNode ───────────────────────────────────────────────────────
//
// Appends a virtual node with no id (id_index is untouched).
// Returns the index of the new node in graph.nodes.

size_t RoutePathSearch::insertVirtualNode(
    route::Graph&     graph,
    const Position3D& pos) const
{
    const size_t idx = graph.nodes.size();
    graph.nodes.push_back({/*id=*/0, pos.x(), pos.y(), /*frame=*/""});
    graph.adj.resize(graph.nodes.size());
    return idx;
}

// ── splitEdgeForSnap ────────────────────────────────────────────────────────
//
// Finds ANY directed edge in graph.adj whose polyline contains a point
// matching snap.snap_point, then replaces it with two sub-edges:
//   src → virtual_idx   (polyline up to and including the snap point)
//   virtual_idx → dst   (polyline from the snap point onward)
//
// If a reverse edge (dst → src) exists, it is split similarly.
//
// Searches through ALL adjacency lists — robust to prior splits on the same
// original edge that may have moved the target polyline to a virtual node.

void RoutePathSearch::splitEdgeForSnap(
    route::Graph&     graph,
    const SnapResult& snap,
    size_t            virtual_idx) const
{
    const double kTol = 1e-6;

    // ── Find and split the forward edge (the one containing snap_point) ──
    bool found = false;
    for (size_t si = 0; si < graph.adj.size() && !found; ++si) {
        for (size_t ei = 0; ei < graph.adj[si].size() && !found; ++ei) {
            auto& nb = graph.adj[si][ei];
            if (nb.polyline.size() < 2) continue;

            // Search for the snap point in this polyline
            for (size_t pi = 0; pi < nb.polyline.size(); ++pi) {
                const double dx = nb.polyline[pi].first  - snap.snap_point.x();
                const double dy = nb.polyline[pi].second - snap.snap_point.y();
                if (std::hypot(dx, dy) > kTol) continue;

                // Found it — split at index pi
                const size_t src = si;
                const size_t dst = nb.to_index;

                // ── Sub-polylines ──
                std::vector<std::pair<double, double>> pl_first(
                    nb.polyline.begin(), nb.polyline.begin() + pi + 1);
                std::vector<std::pair<double, double>> pl_second(
                    nb.polyline.begin() + pi, nb.polyline.end());

                // ── Proportional cost ──
                double len_first = 0.0, len_second = 0.0;
                for (size_t i = 1; i < pl_first.size(); ++i)
                    len_first += std::hypot(pl_first[i].first - pl_first[i-1].first,
                                            pl_first[i].second - pl_first[i-1].second);
                for (size_t i = 1; i < pl_second.size(); ++i)
                    len_second += std::hypot(pl_second[i].first - pl_second[i-1].first,
                                             pl_second[i].second - pl_second[i-1].second);
                const double total_len = len_first + len_second;
                const double cost_first  = (total_len > 1e-9)
                    ? nb.cost * (len_first / total_len) : nb.cost * 0.5;
                const double cost_second = (total_len > 1e-9)
                    ? nb.cost * (len_second / total_len) : nb.cost * 0.5;

                // Replace original edge with first sub-edge (src → virtual)
                nb.to_index = virtual_idx;
                nb.polyline = std::move(pl_first);
                nb.cost     = cost_first;

                // Add second sub-edge (virtual → dst)
                graph.adj[virtual_idx].push_back(
                    {dst, cost_second, std::move(pl_second)});

                found = true;

                // ── Split reverse edge (dst → src) if it exists ──
                for (auto& rev_nb : graph.adj[dst]) {
                    if (rev_nb.to_index != src) continue;
                    if (rev_nb.polyline.size() < 2) continue;

                    // Find the snap point in the reverse polyline
                    for (size_t rpi = 0; rpi < rev_nb.polyline.size(); ++rpi) {
                        const double rdx = rev_nb.polyline[rpi].first  - snap.snap_point.x();
                        const double rdy = rev_nb.polyline[rpi].second - snap.snap_point.y();
                        if (std::hypot(rdx, rdy) > kTol) continue;

                        std::vector<std::pair<double, double>> rpl_first(
                            rev_nb.polyline.begin(), rev_nb.polyline.begin() + rpi + 1);
                        std::vector<std::pair<double, double>> rpl_second(
                            rev_nb.polyline.begin() + rpi, rev_nb.polyline.end());

                        double rlen_first = 0.0, rlen_second = 0.0;
                        for (size_t i = 1; i < rpl_first.size(); ++i)
                            rlen_first += std::hypot(rpl_first[i].first - rpl_first[i-1].first,
                                                     rpl_first[i].second - rpl_first[i-1].second);
                        for (size_t i = 1; i < rpl_second.size(); ++i)
                            rlen_second += std::hypot(rpl_second[i].first - rpl_second[i-1].first,
                                                      rpl_second[i].second - rpl_second[i-1].second);
                        const double rtotal = rlen_first + rlen_second;
                        const double rcost_first  = (rtotal > 1e-9)
                            ? rev_nb.cost * (rlen_first / rtotal) : rev_nb.cost * 0.5;
                        const double rcost_second = (rtotal > 1e-9)
                            ? rev_nb.cost * (rlen_second / rtotal) : rev_nb.cost * 0.5;

                        rev_nb.to_index = virtual_idx;
                        rev_nb.polyline = std::move(rpl_first);
                        rev_nb.cost     = rcost_first;

                        graph.adj[virtual_idx].push_back(
                            {src, rcost_second, std::move(rpl_second)});
                        break;
                    }
                    break;
                }
                break;
            }
        }
    }
}

} // namespace finenav

FINENAV_REGISTER_PLANNER(finenav::RoutePathSearch, route_planner)

