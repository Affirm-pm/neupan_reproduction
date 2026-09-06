// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_route_planner/route_params.hpp"

#include "finenav_route_planner/astar_algorithm.hpp"
#include "finenav_route_planner/imap_view.hpp"
#include "finenav_route_planner/route_imap_view.hpp"
#include "finenav_route_planner/route/route_graph.hpp"
#include "finenav_route_planner/route/route_search.hpp"

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::InputTrajectoryT;
using finenav::core::OutputTrajectoryT;
using finenav::core::Trajectory;

// =============================================================
// RoutePathSearch
//
// FineNav planner for topological-graph route planning.
//
// From the user's perspective this planner takes the same IMapView
// as AstarPathSearch / BfsPathSearch.  The occupancy map is used to
// guide obstacle-aware BFS when snapping the robot's current pose
// and the goal pose to the nearest reachable route graph point
// (either a node or an intermediate point on an edge polyline).
//
// The route graph (GeoJSON) is loaded from the /geojson topic;
// it is cached across planning calls and reloaded when the topic
// publishes new data.
//
// Planning flow in plan():
//   1. Use cached route graph.
//   2. AStarAlgorithm::findPathToAny (h=0 + map isWalkable/getCost)
//      → find the nearest REACHABLE graph point (node OR path point)
//        to ctx.robot_state.pose.
//   3. Same search for the goal — ctx.ref_traj.poses.back().
//   4. For path-point snaps: insert virtual nodes + split edges on a
//      mutable graph copy.
//   5. Dijkstra on the (possibly modified) route graph (route::findRoute).
//   6. Return the densified pose list.
// =============================================================

class RoutePathSearch {
public:
    RoutePathSearch() = default;

    using ConfigType = route_planner::Params;

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

    // Same map interface as AstarPathSearch and BfsPathSearch.
    // isWalkable / getCost are used during the BFS that snaps poses to
    // the nearest route node so that the connector path avoids obstacles.
    using IMapView = finenav::IRouteMapView;

    using Context = PlanningContext<RoutePathSearch>;
    using Result  = std::optional<OutputTrajectoryT<RoutePathSearch>>;

    // Loads (or reloads) the graph when graph_file param changes.
    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IMapView& map_view);

    // Result of snapping a world pose to the nearest reachable route graph point
    // (either a node or an intermediate point on an edge polyline).
    // Carries the entry point info and the obstacle-aware grid path (in world
    // frame) that connects the pose to that point.
    struct SnapResult {
        // Node snapping — valid when isNode() is true.
        size_t node_index{SIZE_MAX};

        // Path snapping — valid when isPath() is true.
        // The snap landed on the edge graph.adj[edge_from][edge_idx] at an
        // intermediate polyline point.  snap_point holds the world coordinates.
        size_t      edge_from{SIZE_MAX};
        size_t      edge_to{SIZE_MAX};       // target node of the edge
        size_t      edge_idx{SIZE_MAX};      // index in graph.adj[edge_from]
        size_t      pt_idx{SIZE_MAX};        // index in the edge's polyline
        Position3D  snap_point{};           // world coordinates of the entry point

        std::vector<Position3D> connector_path;   // world-frame, pose → snap_point

        bool isNode() const { return node_index != SIZE_MAX; }
        bool isPath() const { return edge_from != SIZE_MAX; }
    };

private:
    // Obstacle-aware BFS (Dijkstra h=0 on the occupancy map) to snap a
    // world pose to the nearest reachable route graph point (node or path).
    // Returns a SnapResult with both the entry info and the world-frame
    // connector path from `pose` to that point.
    SnapResult findNearestGraphPoint(
        const route::Graph&             graph,
        const geometry_msgs::msg::Pose& pose,
        const IMapView&                 map_view) const;

    // Insert a temporary virtual node into a mutable graph at the given
    // world position.  Returns the index of the new node.
    // Does NOT register the node in id_index (virtual nodes have no id).
    size_t insertVirtualNode(
        route::Graph&         graph,
        const Position3D&     pos) const;

    // Split the edge that a path-snapped SnapResult landed on.
    // Replaces the original directed edge (graph.adj[edge_from][edge_idx])
    // with two sub-edges: edge_from → virtual_idx  and  virtual_idx → edge_to.
    // If a reverse edge (edge_to → edge_from) exists, it is also split.
    void splitEdgeForSnap(
        route::Graph&         graph,
        const SnapResult&     snap,
        size_t                virtual_idx) const;

    // ── Parameters ────────────────────────────────────────────
    double path_density_ = 0.05;
    double min_approach_angle_deg_ = 0.0;
};

} // namespace finenav

