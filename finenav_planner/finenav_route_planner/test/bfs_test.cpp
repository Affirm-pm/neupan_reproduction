// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// BfsPathSearch / Dijkstra-mode test suite.
//
// BfsAlgorithm no longer exists as a separate class.
// "BFS" is now AStarAlgorithm configured with Heuristic::Zero,
// which degrades A* to Dijkstra (= BFS on uniform-cost grids).
// All tests use this unified algorithm directly or via BfsPathSearch.

#include <gtest/gtest.h>
#include <cmath>

#include "astar_algorithm.hpp"
#include "bfs_path_search.hpp"
#include "grid_map.hpp"

// ── helpers ──────────────────────────────────────────────────────────────

// Build a Dijkstra-mode (h=0) AStarAlgorithm over a GridMap.
inline astar::AStarAlgorithm makeDijkstra(const astar::GridMap& g)
{
    auto is_walkable = [g](const astar::Vec3& c) {
        return g.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [g](const astar::Vec3& c) {
        return g.getCostCell(c.x, c.y);
    };
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setHeuristic(astar::Heuristic::Zero);
    return algo;
}

// ── Algorithm tests (via AStarAlgorithm + Heuristic::Zero) ──────────────

TEST(BfsAlgorithm2D, FindsPath_OpenSpace)
{
    auto algo = makeDijkstra(astar::GridMap(20, 20));
    algo.setConnectivity(astar::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(),  (astar::Vec3{5, 5, 0}));
}

TEST(BfsAlgorithm2D, PathLength_Orthogonal)
{
    auto algo = makeDijkstra(astar::GridMap(20, 20));
    algo.setConnectivity(astar::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 0, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 6u);
}

TEST(BfsAlgorithm2D, Diagonal_AllowedReducesPathLength)
{
    astar::GridMap grid(20, 20);
    auto ortho = makeDijkstra(grid);
    ortho.setConnectivity(astar::Connectivity::N4);
    auto path_ortho = ortho.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_ortho.has_value());

    auto diag = makeDijkstra(grid);
    diag.setConnectivity(astar::Connectivity::N8);
    auto path_diag = diag.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_diag.has_value());

    EXPECT_LT(path_diag->size(), path_ortho->size());
}

TEST(BfsAlgorithm2D, PathAvoidsObstacles)
{
    astar::GridMap grid(10, 10);
    for (int y = 0; y <= 8; ++y) grid.setObstacle(3, y, true);

    auto algo = makeDijkstra(grid);
    algo.setConnectivity(astar::Connectivity::N4);

    auto path = algo.findPath({0, 5, 0}, {6, 5, 0});
    ASSERT_TRUE(path.has_value());
    for (const auto& c : *path) {
        EXPECT_FALSE(c.x == 3 && c.y <= 8)
            << "Path passes through obstacle at (" << c.x << "," << c.y << ")";
    }
}

TEST(BfsAlgorithm2D, StartEqualsGoal)
{
    auto algo = makeDijkstra(astar::GridMap(10, 10));
    auto path = algo.findPath({3, 3, 0}, {3, 3, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (astar::Vec3{3, 3, 0}));
}

TEST(BfsAlgorithm2D, GoalBlocked_ReturnsNullopt)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    auto algo = makeDijkstra(grid);
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(BfsAlgorithm2D, MaxIterations_TerminatesEarly)
{
    auto algo = makeDijkstra(astar::GridMap(1000, 1000));
    algo.setMaxIterations(10);
    algo.setGoalTolerance(0.0);
    (void)algo.findPath({0, 0, 0}, {999, 999, 0});
    SUCCEED();
}

TEST(BfsAlgorithm2D, MaxIterationsZero_ReturnsNulloptForDifferentGoal)
{
    auto algo = makeDijkstra(astar::GridMap(10, 10));
    algo.setMaxIterations(0);
    algo.setGoalTolerance(0.0);
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {1, 0, 0}).has_value());
}

// findPathToAny is now a method of AStarAlgorithm (h forced to 0 internally)
TEST(BfsAlgorithm2D, FindPathToAny_ReturnsStartWhenStartIsGoal)
{
    auto algo = makeDijkstra(astar::GridMap(10, 10));
    auto path = algo.findPathToAny({3, 3, 0}, [](const astar::Vec3& c) {
        return c == astar::Vec3{3, 3, 0};
    });
    ASSERT_TRUE(path.has_value());
    ASSERT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (astar::Vec3{3, 3, 0}));
}

TEST(BfsAlgorithm2D, FindPathToAny_StopsAtFirstReachableGoal)
{
    auto algo = makeDijkstra(astar::GridMap(20, 20));
    algo.setConnectivity(astar::Connectivity::N8);

    auto path = algo.findPathToAny({0, 0, 0}, [](const astar::Vec3& c) {
        return c == astar::Vec3{3, 0, 0} || c == astar::Vec3{8, 0, 0};
    });
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->back(), (astar::Vec3{3, 0, 0}));
    EXPECT_EQ(path->size(), 4u);
}

TEST(BfsAlgorithm3D, FindsPath_OpenVolume)
{
    auto is_walkable = [](const astar::Vec3&) { return true; };
    auto get_cost    = [](const astar::Vec3&) { return 0.0; };
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setConnectivity(astar::Connectivity::N26);

    auto path = algo.findPath({0, 0, 0}, {3, 3, 3});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(),  (astar::Vec3{3, 3, 3}));
}

// Weighted-cost test: Dijkstra must prefer low-cost detour over shorter obstacle-adjacent path
TEST(BfsAlgorithm2D, PrefersLowerCostPath_WeightedGrid)
{
    // 5-wide grid. Direct path (x=2 column) has high cost cells.
    // Detour (x=0 column) is longer but cheaper.
    astar::GridMap grid(5, 6);
    grid.setCost(2, 1, 100.0);
    grid.setCost(2, 2, 100.0);
    grid.setCost(2, 3, 100.0);

    auto algo = makeDijkstra(grid);
    algo.setConnectivity(astar::Connectivity::N4);
    algo.setGoalTolerance(0.0);

    auto path = algo.findPath({2, 0, 0}, {2, 5, 0});
    ASSERT_TRUE(path.has_value());

    // The path should avoid x=2, y=1..3 (high-cost cells)
    for (const auto& c : *path) {
        EXPECT_FALSE(c.x == 2 && c.y >= 1 && c.y <= 3)
            << "Dijkstra took high-cost shortcut at (" << c.x << "," << c.y << ")";
    }
}

// ── BfsPathSearch integration tests ─────────────────────────────────────

class TestMapView : public finenav::BfsPathSearch::IMapView {
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

inline finenav::BfsPathSearch makePlanner()
{
    finenav::BfsPathSearch planner;
    bfs_planner::Params p;
    p.connectivity      = "4";
    p.goal_tolerance    = 0.6;
    p.max_iterations    = 1000000;
    p.max_planning_time = 5.0;
    p.movement_weight   = 1.0;
    planner.configure(p);
    return planner;
}

inline finenav::PlanningContext<finenav::BfsPathSearch>
makeCtx(double sx, double sy, double gx, double gy)
{
    finenav::PlanningContext<finenav::BfsPathSearch> ctx;
    ctx.robot_state.position.x() = sx;
    ctx.robot_state.position.y() = sy;
    ctx.goal_pose.position.x = gx;
    ctx.goal_pose.position.y = gy;
    return ctx;
}

TEST(BfsPathSearch, PlanSucceeds_OpenGrid)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();
    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_GE(result->poses.size(), 1u);
}

TEST(BfsPathSearch, PlanFails_GoalBlocked)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view).has_value());
}

TEST(BfsPathSearch, PlanFails_InvalidStepSize)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 0.0);
    auto planner = makePlanner();
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view).has_value());
}

TEST(BfsPathSearch, OutputPoses_AreWorldCoordinates)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();
    auto result = planner.plan(makeCtx(0.5, 0.5, 3.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    for (const auto& pose : result->poses) {
        EXPECT_GE(pose.position.x, 0.0);
        EXPECT_LT(pose.position.x, 10.0);
        EXPECT_GE(pose.position.y, 0.0);
        EXPECT_LT(pose.position.y, 10.0);
    }
}
