// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// RoutePathSearch (topological route planning) test suite.
//
// Covers:
//   - GeoJSON graph loading via configure() (valid / missing / malformed)
//   - Dijkstra routing on the topological graph (reachable / disconnected /
//     fixed-cost-override route selection / start==goal snapping)
//   - Obstacle-aware pose-to-node snapping (findNearestNodeIndex), including
//     skipping a geometrically-closer but unreachable node
//   - Connector paths: obstacle-aware grid paths from robot/goal pose to
//     nearest route node are stitched into the output
//   - Output shape: world-frame poses, densification, exact endpoints

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "finenav_route_planner/route_path_search.hpp"
#include "grid_map.hpp"

namespace {

namespace fs = std::filesystem;

// ── RAII temp GeoJSON file ───────────────────────────────────────────────
// RoutePathSearch loads its graph from a file path (graph_file param), so
// each fixture is materialised on disk for the duration of the test.
class TempFile {
public:
    explicit TempFile(const std::string& content)
        : path_((fs::temp_directory_path() /
                 ("route_planner_test_" + std::to_string(counter_++) + ".geojson")).string())
    {
        std::ofstream f(path_);
        f << content;
    }
    ~TempFile() { std::error_code ec; fs::remove(path_, ec); }

    const std::string& path() const { return path_; }

private:
    std::string path_;
    static inline int counter_ = 0;
};

// ── Test map view (same adaptor pattern as bfs_test.cpp) ────────────────
// Wraps the dependency-free astar::GridMap as an astar::IMapView so the
// obstacle-aware snapping inside RoutePathSearch can be exercised.
class TestMapView : public astar::IMapView {
public:
    TestMapView(const astar::GridMap& g, double step_size)
        : grid_(g), step_(step_size) {}

    bool isWalkable(const finenav::Position3D& p) const override {
        return grid_.isWalkableCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }
    double getCost(const finenav::Position3D& p) const override {
        return grid_.getCostCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }
    double getStepSize() const override { return step_; }

private:
    const astar::GridMap& grid_;
    double step_;
};

// ── helpers ──────────────────────────────────────────────────────────────

finenav::RoutePathSearch makePlanner(const std::string& graph_file, double path_density = 0.05)
{
    finenav::RoutePathSearch planner;
    route_planner::Params p;
    p.path_density = path_density;
    planner.configure(p);

    if (!graph_file.empty()) {
        std::ifstream f(graph_file);
        if (f.is_open()) {
            std::ostringstream buf;
            buf << f.rdbuf();
            planner.updateGraph(buf.str());
        }
    }
    return planner;
}

// PlanningContext stores const references into owned storage, so the owned
// RobotState / ref_traj must outlive every plan() call.  Keep them in a bundle
// that converts to `const PlanningContext&`; the bundle is a temporary living
// for the whole plan() call, so call sites (`planner.plan(makeCtx(...), view)`)
// stay unchanged.
struct RouteTestCtx {
    finenav::core::RobotState robot_state;
    finenav::core::InputTrajectoryT<finenav::RoutePathSearch> ref_traj;
    finenav::PlanningContext<finenav::RoutePathSearch> ctx{robot_state, ref_traj};

    RouteTestCtx(double sx, double sy, double gx, double gy)
    {
        robot_state.position.x() = sx;
        robot_state.position.y() = sy;
        robot_state.orientation.w() = 1.0;
        ref_traj.poses.resize(1);
        ref_traj.poses[0].position.x    = gx;
        ref_traj.poses[0].position.y    = gy;
        ref_traj.poses[0].orientation.w = 1.0;
    }

    operator const finenav::PlanningContext<finenav::RoutePathSearch>&() const
    {
        return ctx;
    }
};

RouteTestCtx
makeCtx(double sx, double sy, double gx, double gy)
{
    return RouteTestCtx{sx, sy, gx, gy};
}

double dist(double x0, double y0, double x1, double y1)
{
    return std::hypot(x1 - x0, y1 - y0);
}

double pathLength(const std::vector<geometry_msgs::msg::Pose>& poses)
{
    double len = 0.0;
    for (size_t i = 1; i < poses.size(); ++i) {
        len += dist(poses[i - 1].position.x, poses[i - 1].position.y,
                    poses[i].position.x,     poses[i].position.y);
    }
    return len;
}

// Surround cell (cx, cy) with obstacles on all 8 neighbours, isolating it
// from the rest of an otherwise-open N8-connected grid.
void enclose(astar::GridMap& grid, int cx, int cy)
{
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
            if (dx != 0 || dy != 0)
                grid.setObstacle(cx + dx, cy + dy, true);
}

// ── GeoJSON fixtures ─────────────────────────────────────────────────────

// Linear chain  A(0,0) -- B(5,0) -- C(10,0), bidirectional edges.
const char* kLinearGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0,"frame":"A"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5,0]},"properties":{"id":1,"frame":"B"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,0]},"properties":{"id":2,"frame":"C"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[5,0]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[5,0],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[5,0],[10,0]]},"properties":{"id":102,"startid":1,"endid":2}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,0],[5,0]]},"properties":{"id":103,"startid":2,"endid":1}}
  ]
}
)json";

// Two disjoint chains: {0,1} near the origin, {2,3} far away — no edge joins them.
const char* kDisconnectedGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5,0]},"properties":{"id":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[20,0]},"properties":{"id":2}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[25,0]},"properties":{"id":3}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[5,0]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[5,0],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[20,0],[25,0]]},"properties":{"id":102,"startid":2,"endid":3}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[25,0],[20,0]]},"properties":{"id":103,"startid":3,"endid":2}}
  ]
}
)json";

// 0 --(fixed cost 100, overridable=false)--> 1   [direct, geometrically length 10]
// 0 -> 2 -> 1 with default (geometric-length) costs: 8 + sqrt(10^2+8^2) ~= 20.8
// The cheaper route must go through node 2 despite being geometrically longer.
const char* kCostOverrideGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,0]},"properties":{"id":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,8]},"properties":{"id":2}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[10,0]]},"properties":{"id":100,"startid":0,"endid":1,"cost":100.0,"overridable":false}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[0,8]]},"properties":{"id":101,"startid":0,"endid":2}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,8],[10,0]]},"properties":{"id":102,"startid":2,"endid":1}}
  ]
}
)json";

// Two nodes: P sits in a pocket that obstacles will isolate, Q sits in the open.
const char* kSnapGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[2.5,2.5]},"properties":{"id":0,"frame":"P_enclosed"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[6.5,0.5]},"properties":{"id":1,"frame":"Q_open"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[2.5,2.5],[6.5,0.5]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[6.5,0.5],[2.5,2.5]]},"properties":{"id":101,"startid":1,"endid":0}}
  ]
}
)json";

// Single isolated node — used to test the "no node reachable at all" failure path.
const char* kIsolatedNodeGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.5,5.5]},"properties":{"id":0}}
  ]
}
)json";

// Two nodes A(0,0) and B(10,0) connected by a dense polyline edge
// with intermediate points at every 2 m.  This enables path-point snapping.
const char* kDensePathGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0,"frame":"A"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,0]},"properties":{"id":1,"frame":"B"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[2,0],[4,0],[6,0],[8,0],[10,0]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,0],[8,0],[6,0],[4,0],[2,0],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}}
  ]
}
)json";

// Three-node chain A(0,0) — B(10,0) — C(20,0) with dense polylines.
// Used to test U-turn prevention: robot near mid-path should enter the
// graph at the nearest path point and go forward toward the goal instead
// of backtracking to a node.
const char* kThreeNodeChainGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0,"frame":"A"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,0]},"properties":{"id":1,"frame":"B"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[20,0]},"properties":{"id":2,"frame":"C"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[2,0],[4,0],[6,0],[8,0],[10,0]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,0],[8,0],[6,0],[4,0],[2,0],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,0],[12,0],[14,0],[16,0],[18,0],[20,0]]},"properties":{"id":102,"startid":1,"endid":2}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[20,0],[18,0],[16,0],[14,0],[12,0],[10,0]]},"properties":{"id":103,"startid":2,"endid":1}}
  ]
}
)json";

// A single edge with a dense polyline: A(0,0) → B(10,10) via intermediate points.
// Tests path snapping on a diagonal edge.
const char* kDiagonalDenseGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,10]},"properties":{"id":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[2,2],[4,4],[6,6],[8,8],[10,10]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,10],[8,8],[6,6],[4,4],[2,2],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}}
  ]
}
)json";

}  // namespace

// =========================================================================
// configure() / graph loading
// =========================================================================

TEST(RoutePathSearch, ConfigureFails_GracefullyOnMissingFile)
{
    auto planner = makePlanner("/nonexistent/path/to/graph.geojson");
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, ConfigureFails_GracefullyOnMalformedJson)
{
    TempFile bad("{ this is not valid GeoJSON ]");
    auto planner = makePlanner(bad.path());
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, PlanFails_WhenGraphFileEmpty)
{
    auto planner = makePlanner("");   // default_value of graph_file
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, ConfigureSucceeds_LoadsValidGraph)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path());
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    // A loaded, non-empty graph must let plan() proceed past the
    // "no graph cached" early-out.
    EXPECT_TRUE(planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view).has_value());
}

// =========================================================================
// Dijkstra routing over the topological graph
// =========================================================================

TEST(RoutePathSearch, PlanSucceeds_LinearGraph_EndpointsNearRobotAndGoal)
{
    // The output now includes obstacle-aware connector paths:
    //   robot (0.5,0.5) → node A (0,0) → route → node C (10,0) → goal (9.5,0.5)
    // First point should be near the robot pose, last near the goal pose.
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // First point near robot (0.5, 0.5), not near node A (0, 0)
    EXPECT_NEAR(result->poses.front().position.x, 0.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);

    // Last point near goal (9.5, 0.5), not node C (10, 0)
    EXPECT_NEAR(result->poses.back().position.x, 9.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);

    // The route graph interior still passes through the expected nodes
    // (verify by checking that node A (0,0) and node C (10,0) appear)
    bool has_node_a = false, has_node_c = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 0.0, p.position.y - 0.0) < 0.01) has_node_a = true;
        if (std::hypot(p.position.x - 10.0, p.position.y - 0.0) < 0.01) has_node_c = true;
    }
    EXPECT_TRUE(has_node_a);
    EXPECT_TRUE(has_node_c);
}

TEST(RoutePathSearch, PlanFails_DisconnectedGraph_NoRouteBetweenComponents)
{
    TempFile graph(kDisconnectedGraph);
    auto planner = makePlanner(graph.path());
    astar::GridMap grid(40, 10);
    TestMapView view(grid, 1.0);

    // Start snaps into the {0,1} component, goal into the {2,3} component —
    // Dijkstra on the graph must find no path and return nullopt.
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 22.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, PlanPrefersLowerCost_OverFixedExpensiveDirectEdge)
{
    TempFile graph(kCostOverrideGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.0);  // node-centres only
    astar::GridMap grid(20, 20);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 10.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // The fixed-cost direct edge (cost=100) is far more expensive than the
    // detour via node 2 (~20.8), so the route must visit node 2.
    // Output now includes connectors: robot→node0, route, node1→goal.
    // Verify the node sequence: node 0 → node 2 → node 1 appears in order.
    ASSERT_GE(result->poses.size(), 3u);

    // Find the positions of the three nodes in the output
    auto findNode = [&](double x, double y) -> std::optional<size_t> {
        for (size_t i = 0; i < result->poses.size(); ++i)
            if (std::hypot(result->poses[i].position.x - x,
                           result->poses[i].position.y - y) < 1e-6)
                return i;
        return std::nullopt;
    };

    auto i0 = findNode(0.0, 0.0);
    auto i2 = findNode(0.0, 8.0);
    auto i1 = findNode(10.0, 0.0);
    ASSERT_TRUE(i0.has_value());
    ASSERT_TRUE(i2.has_value());
    ASSERT_TRUE(i1.has_value());
    EXPECT_LT(*i0, *i2);
    EXPECT_LT(*i2, *i1);
}

TEST(RoutePathSearch, Plan_SingleNodeGraph_StartEqualsGoal_ConnectorsBothSides)
{
    // Both start and goal snap to the same node.  The output now includes
    // connectors on both sides: robot → node → goal.
    TempFile graph(kIsolatedNodeGraph);
    auto planner = makePlanner(graph.path());
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(5.6, 5.6, 5.4, 5.4), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // First point is the robot pose
    EXPECT_NEAR(result->poses.front().position.x, 5.6, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 5.6, 1e-6);

    // Last point is the goal pose
    EXPECT_NEAR(result->poses.back().position.x, 5.4, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 5.4, 1e-6);

    // The node (5.5, 5.5) appears somewhere in the middle
    bool has_node = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 5.5, p.position.y - 5.5) < 1e-6)
            has_node = true;
    }
    EXPECT_TRUE(has_node);
}

// =========================================================================
// Output shape: densification + world coordinates
// =========================================================================

TEST(RoutePathSearch, Plan_Densification_SpacingMatchesPathDensity)
{
    TempFile graph(kLinearGraph);
    constexpr double kDensity = 1.0;
    auto planner = makePlanner(graph.path(), kDensity);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 3u);

    // Interior segment spacing should track the configured density closely.
    // Skip the start/goal connector segments: those have non-zero y (robot
    // at y=0.5) and/or diagonal steps (~0.707 m), while route edges lie on y=0.
    for (size_t i = 1; i + 1 < result->poses.size(); ++i) {
        const auto& a = result->poses[i - 1].position;
        const auto& b = result->poses[i].position;
        // Connector segments have y ≠ 0 or x outside the route extent [0, 10].
        const bool a_on_route = (std::abs(a.y) < 1e-6 && a.x >= 0.0 && a.x <= 10.0);
        const bool b_on_route = (std::abs(b.y) < 1e-6 && b.x >= 0.0 && b.x <= 10.0);
        if (!a_on_route || !b_on_route) continue;
        const double seg = dist(a.x, a.y, b.x, b.y);
        EXPECT_NEAR(seg, kDensity, 0.2);
    }
}

TEST(RoutePathSearch, Plan_ZeroDensity_IncludesNodeCentresAndConnectors)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.0);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // With zero density, route graph contributes node centres only: A(0,0), B(5,0), C(10,0).
    // Connectors: robot(0.5,0.5)→A(0,0) and C(10,0)→goal(9.5,0.5).
    // Verify the node centres appear in order.
    auto findNode = [&](double x, double y) -> std::optional<size_t> {
        for (size_t i = 0; i < result->poses.size(); ++i)
            if (std::hypot(result->poses[i].position.x - x,
                           result->poses[i].position.y - y) < 1e-4)
                return i;
        return std::nullopt;
    };

    auto iA = findNode(0.0, 0.0);
    auto iB = findNode(5.0, 0.0);
    auto iC = findNode(10.0, 0.0);
    ASSERT_TRUE(iA.has_value());
    ASSERT_TRUE(iB.has_value());
    ASSERT_TRUE(iC.has_value());
    EXPECT_LT(*iA, *iB);
    EXPECT_LT(*iB, *iC);
}

TEST(RoutePathSearch, Plan_OutputPoses_HaveValidOrientation)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), 0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    for (const auto& pose : result->poses) {
        // RoutePathSearch packs an identity orientation; a higher layer
        // assigns headings later. Check it is at least a valid quaternion.
        const double n = std::hypot(std::hypot(pose.orientation.x, pose.orientation.y),
                                    std::hypot(pose.orientation.z, pose.orientation.w));
        EXPECT_NEAR(n, 1.0, 1e-6);
    }
}

TEST(RoutePathSearch, Plan_TotalLength_CoversRobotToGoal)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), 0.25);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // Path covers robot(0.5,0.5) → node A(0,0) → node B(5,0) → node C(10,0) → goal(9.5,0.5).
    // Minimum straight-line distance from robot to goal = 9.0 m,
    // but the path backtracks from robot to node A then forward again.
    // robot→A ≈ 0.71, A→B→C = 10.0, C→goal ≈ 0.71 → total ≈ 11.4 m.
    const double len = pathLength(result->poses);
    EXPECT_GT(len, 9.0);
    EXPECT_LT(len, 12.0);
}

// =========================================================================
// Obstacle-aware pose -> route-node snapping (findNearestNodeIndex)
// =========================================================================

TEST(RoutePathSearch, Plan_SnapsToReachableNode_IncludesConnectorPath)
{
    // Node P (2.5,2.5) is enclosed by obstacles — geometrically closer but
    // unreachable.  Both robot and goal snap to node Q (6.5,0.5).
    // The output must include the connector path from robot to Q,
    // which proves the obstacle-aware search worked.
    TempFile graph(kSnapGraph);
    auto planner = makePlanner(graph.path());

    // Open N8-connected grid except node P's cell (2,2), which is walled in.
    astar::GridMap grid(12, 12);
    enclose(grid, 2, 2);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.6, 0.6, 0.7, 0.7), view);
    ASSERT_TRUE(result.has_value());

    // Output now includes connectors — at least 2 points (start connector + goal connector).
    ASSERT_GE(result->poses.size(), 2u);

    // First point is robot pose
    EXPECT_NEAR(result->poses.front().position.x, 0.6, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.6, 1e-6);

    // Last point is goal pose
    EXPECT_NEAR(result->poses.back().position.x, 0.7, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.7, 1e-6);

    // Node Q (6.5, 0.5) appears somewhere (the reachable node)
    bool has_q = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 6.5, p.position.y - 0.5) < 1e-4)
            has_q = true;
    }
    EXPECT_TRUE(has_q);

    // Node P (2.5, 2.5) must NOT appear (it's unreachable)
    bool has_p = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 2.5, p.position.y - 2.5) < 1e-4)
            has_p = true;
    }
    EXPECT_FALSE(has_p);
}

TEST(RoutePathSearch, PlanFails_WhenNoNodeIsReachableFromRobot)
{
    TempFile graph(kIsolatedNodeGraph);
    auto planner = makePlanner(graph.path());

    // Wall the only node's cell off completely — the obstacle-aware search
    // can never find a cell containing a graph node.
    astar::GridMap grid(12, 12);
    enclose(grid, 5, 5);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 0.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, PlanFails_WhenStartCellItselfIsBlocked)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path());

    astar::GridMap grid(12, 12);
    grid.setObstacle(0, 0, true);   // robot starts inside an obstacle cell
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view).has_value());
}

// =========================================================================
// Path-point snapping (new: snap to intermediate polyline points)
// =========================================================================

TEST(RoutePathSearch, Plan_SnapsToPathPoint_WhenRobotNearPathMidpoint)
{
    // Robot at (5, 1) — near the midpoint of the A(0,0)→B(10,0) dense edge.
    // With path-snapping, BFS should find the point (4,0) or (6,0) on the
    // edge's polyline and snap there, rather than going all the way to node A or B.
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    // Robot at (5, 1), goal at (5.5, 0.5) — both near the path midpoint
    auto result = planner.plan(makeCtx(5.0, 1.0, 5.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // First point should be near robot pose
    EXPECT_NEAR(result->poses.front().position.x, 5.0, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 1.0, 1e-6);

    // Last point near goal
    EXPECT_NEAR(result->poses.back().position.x, 5.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);

    // The total path length should be short — the robot enters the graph
    // at a nearby path point and exits nearby, avoiding long detours to nodes.
    const double len = pathLength(result->poses);
    // Direct distance robot→goal ≈ 0.7 m.  Connectors should be short
    // because both snap near the path midpoint.
    EXPECT_LT(len, 5.0);   // well under what node-only snapping would give (~10 m)

    // Intermediate poses should include a point on the path (y ≈ 0)
    bool has_path_point = false;
    for (const auto& p : result->poses) {
        if (std::abs(p.position.y) < 1e-4 && p.position.x > 0.0 && p.position.x < 10.0)
            has_path_point = true;
    }
    EXPECT_TRUE(has_path_point);
}

TEST(RoutePathSearch, Plan_ShortConnector_WhenRobotNearDensePathFarFromNodes)
{
    // Robot at (5, 0.5) — geometrically near the path midpoint, far from nodes.
    // Goal at (9.5, 0.5) — near node B.
    // Without path-snapping, robot would walk to node A (0,0) or B (10,0) first.
    // With path-snapping, it should enter at the nearest path point (~5,0).
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(5.0, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 3u);

    // First point near robot
    EXPECT_NEAR(result->poses.front().position.x, 5.0, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);

    // The connector should be short: robot goes to a path point near (5, 0),
    // not all the way to node A at (0, 0) or node B at (10, 0).
    // Verify that a point near (5, 0) appears early in the path.
    bool has_midpoint_entry = false;
    for (size_t i = 0; i < std::min(size_t(5), result->poses.size()); ++i) {
        const auto& p = result->poses[i];
        if (std::hypot(p.position.x - 5.0, p.position.y) < 1.0)
            has_midpoint_entry = true;
    }
    EXPECT_TRUE(has_midpoint_entry);

    // Total path should be significantly shorter than node-only snapping
    // (which would be ~0.7m robot→A + 10m A→B + 0.7m B→goal ≈ 11.4m)
    const double len = pathLength(result->poses);
    EXPECT_LT(len, 8.0);
}

TEST(RoutePathSearch, Plan_ThreeNodeChain_RobotEntersAtMidPath_GoesForward)
{
    // Graph: A(0,0) — B(10,0) — C(20,0).
    // Robot at (8, 2), goal near C at (18, 0.5).
    // With path-snapping: robot enters the A-B edge at ~8,0, goes forward
    // through B to C, then to goal.  No backtracking toward A.
    // Without path-snapping: BFS might find node A or B first, potentially
    // causing a detour.
    TempFile graph(kThreeNodeChainGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(22, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(8.0, 2.0, 18.0, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 3u);

    // First point near robot
    EXPECT_NEAR(result->poses.front().position.x, 8.0, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 2.0, 1e-6);

    // Last point near goal
    EXPECT_NEAR(result->poses.back().position.x, 18.0, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);

    // The path should visit node B (10, 0) somewhere in the middle
    bool has_node_b = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 10.0, p.position.y) < 1e-4)
            has_node_b = true;
    }
    EXPECT_TRUE(has_node_b);

    // The non-connector portion of the path should go forward (x increasing).
    // Check that after the point with y ≈ 0 (entry onto the graph), x-coordinates
    // are monotonically non-decreasing (no backtracking toward A at x=0).
    bool on_graph = false;
    double prev_x = -std::numeric_limits<double>::infinity();
    for (const auto& p : result->poses) {
        if (std::abs(p.position.y) < 1e-4) on_graph = true;  // entered the graph
        if (on_graph) {
            EXPECT_GE(p.position.x, prev_x - 1e-6)
                << "Backtracking detected after entering graph at x=" << p.position.x;
            prev_x = p.position.x;
        }
    }

    // Crucially, the path must NOT backtrack to node A (x=0).
    for (const auto& p : result->poses) {
        EXPECT_GT(p.position.x, 1.0) << "Path should not go back to node A at x=0";
    }

    // Total length should be reasonable: ~2.8m robot→path + ~10m path + 0.5m goal
    const double len = pathLength(result->poses);
    EXPECT_GT(len, 8.0);
    EXPECT_LT(len, 16.0);
}

TEST(RoutePathSearch, Plan_SnapsToNode_WhenNodeIsCloserThanAnyPathPoint)
{
    // Robot at (0.5, 0.5) — geometrically closer to node A(0,0) than to any
    // intermediate path point.  Should still snap to the node (existing behavior).
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // Node A (0, 0) should appear — robot snaps to it
    bool has_node_a = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 0.0, p.position.y - 0.0) < 1e-4)
            has_node_a = true;
    }
    EXPECT_TRUE(has_node_a);
}

TEST(RoutePathSearch, Plan_DiagonalDenseEdge_SnapsToPathPoint)
{
    // Robot at (5.5, 4.5) — near the diagonal path between (0,0) and (10,10).
    // Goal at (9.5, 9.5) — near the end node.
    // Path-snapping should enter the graph at the nearest diagonal path point.
    TempFile graph(kDiagonalDenseGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(5.5, 4.5, 9.5, 9.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // First point near robot
    EXPECT_NEAR(result->poses.front().position.x, 5.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 4.5, 1e-6);

    // Last point near goal
    EXPECT_NEAR(result->poses.back().position.x, 9.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 9.5, 1e-6);

    // Should include points on the diagonal (y ≈ x)
    bool has_diag_point = false;
    for (const auto& p : result->poses) {
        if (std::abs(p.position.x - p.position.y) < 0.2 &&
            p.position.x > 0.0 && p.position.x < 10.0)
            has_diag_point = true;
    }
    EXPECT_TRUE(has_diag_point);
}

TEST(RoutePathSearch, PlanFails_WhenNoGraphPointIsReachable)
{
    // Same as PlanFails_WhenNoNodeIsReachableFromRobot — but with a graph that
    // also has path points.  All are walled off, so the search must still fail.
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path());

    // Wall off the entire graph area
    astar::GridMap grid(12, 12);
    for (int x = 0; x < 11; ++x)
        for (int y = 0; y < 2; ++y)
            grid.setObstacle(x, y, true);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(5.0, 5.0, 10.0, 5.0), view).has_value());
}
