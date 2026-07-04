// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>
#include <mutex>
#include <std_msgs/msg/string.hpp>

#include "finenav_core/concepts.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_route_planner/route_params.hpp"

#include "astar_algorithm.hpp"
#include "imap_view.hpp"
#include "route/route_graph.hpp"
#include "route/route_search.hpp"

namespace finenav {

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
//   3. Same search for ctx.goal_pose.
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

    // Same map interface as AstarPathSearch and BfsPathSearch.
    // isWalkable / getCost are used during the BFS that snaps poses to
    // the nearest route node so that the connector path avoids obstacles.
    using IMapView = astar::IMapView;

    using Context = PlanningContext<RoutePathSearch>;
    using Result  = std::optional<OutputDataT<RoutePathSearch>>;

    // Loads (or reloads) the graph when graph_file param changes.
    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IMapView& map_view);

    /**
     * @brief 从 GeoJSON 格式的字符串更新缓存的路网图
     * @param geojson_str GeoJSON 格式的数据内容
     */
    void updateGraph(const std::string& geojson_str);

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

    // ── Cached graph & Mutex ──────────────────────────────────
    mutable std::unique_ptr<std::mutex> graph_mutex_ = std::make_unique<std::mutex>();
    std::optional<route::Graph> cached_graph_;

    // ── Parameters ────────────────────────────────────────────
    double path_density_ = 0.05;
    double min_approach_angle_deg_ = 0.0;
};

} // namespace finenav

//TODO: 不用 FINENAV_REGISTER_ALGO_CONFIG(finenav::RoutePathSearch, route_planner) 的做法有待商榷
namespace finenav::core {

template <>
struct AlgoConfigurator<finenav::RoutePathSearch> {
    static constexpr std::string_view name = "route_planner";
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr node, finenav::RoutePathSearch& algo) {
        auto listener = std::make_shared<route_planner::ParamListener>(node, "route_planner");
        algo.configure(listener->get_params());
        listener->setUserCallback([&algo](const auto& params) {
            algo.configure(params);
        });

        struct LifetimeKeeper {
            std::shared_ptr<route_planner::ParamListener> listener;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
        };

        auto keeper = std::make_shared<LifetimeKeeper>();
        keeper->listener = listener;
        keeper->sub = node->create_subscription<std_msgs::msg::String>(
            "/geojson",
            rclcpp::QoS(1).transient_local(),
            [&algo](const std_msgs::msg::String::SharedPtr msg) {
                algo.updateGraph(msg->data);
            }
        );

        return keeper;
    }
};

} // namespace finenav::core
