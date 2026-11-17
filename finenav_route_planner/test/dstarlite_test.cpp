// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/**
 * @file dstarlite_test.cpp
 * @brief D* Lite 算法单元测试 — 覆盖核心引擎 + 增量 replan + PlannerConcept 集成
 *
 * @details
 * ==========================================================================
 * 测试总览
 * ==========================================================================
 * 本文件使用 Google Test 验证 DStarLiteAlgorithm 核心引擎和
 * DStarLitePathSearch 包装器的正确性。D* Lite 与 A* 的关键区别在于
 * **增量 replan**——环境局部变化时只修复受影响区域而非全量重搜索，
 * 因此本测试套件的重心放在障碍物变化后的 replan 行为验证上。
 *
 * 被测对象:
 *   - DStarLiteAlgorithm (dstarlite:: 命名空间, 无 ROS 依赖)
 *   - DStarLitePathSearch (finenav:: 命名空间, PlannerConcept 实现)
 *
 * 测试分层:
 *   -# 核心引擎 — 基础寻路、障碍处理、增量 replan (add/remove/syncMap)、
 *      搜索限制、goal_tolerance、3D 支持
 *   -# PlannerConcept 集成 — TestMapView + PlanningContext + plan(),
 *      replan、reset 生命周期
 *
 * ==========================================================================
 * 测试环境
 * ==========================================================================
 * 测试框架:     Google Test (GTest)
 * 依赖:         finenav_route_planner, common/grid_map.hpp (测试工具)
 * ROS 依赖:     无（核心引擎层）, 集成层需 ROS 2 头文件
 * 构建:         colcon build --packages-select finenav_route_planner
 *
 * 运行:
 * @code
 *   cd ~/FineNav_ws
 *   colcon test --packages-select finenav_route_planner
 *   # 或直接运行:
 *   ./build/finenav_route_planner/test/dstarlite_test
 *   # 仅核心引擎:
 *   ./build/finenav_route_planner/test/dstarlite_test --gtest_filter="DStarLiteAlgorithm.*"
 *   # 仅集成层:
 *   ./build/finenav_route_planner/test/dstarlite_test --gtest_filter="DStarLitePathSearch.*"
 *   # 仅 replan 相关:
 *   ./build/finenav_route_planner/test/dstarlite_test --gtest_filter="*Replan*:*Adapts*:*Remove*:*Sync*"
 * @endcode
 *
 * ==========================================================================
 * 测试详细说明
 * ==========================================================================
 *
 * --- Layer 1: DStarLiteAlgorithm (核心引擎) ---
 *
 * 基础寻路:
 *   FindsPath_OpenSpace          — 20×20 空网格, N8, (0,0)→(5,5)
 *   StartEqualsGoal              — 起点=终点 → 单 cell 路径
 *   Diagonal_ShorterPath         — N8 路径 < N4 路径 (对角线缩短)
 *   PathIsContinuous_N4          — 相邻 cell 曼哈顿距离 = 1
 *
 * 障碍处理:
 *   GoalBlocked_ReturnsEmpty     — 终点被占据 → extractPath() 返回空
 *   PathAvoidsObstacles          — x=3 垂直墙, 路径不穿越
 *
 * 增量 replan (★ D* Lite 核心竞争力):
 *   Replan_AdaptsToNewObstacle   — 初始规划成功 → setObstacle 添加墙 →
 *                                    replan() → 新路径绕开障碍
 *   RemoveObstacle_OpensNewPath  — 全墙阻断 → removeObstacle 开缺口 →
 *                                    replan() → 新路径穿过缺口
 *   SyncMap_DetectsChanges       — 外部修改 GridMap → syncMap() 检测变更 →
 *                                    replan() 避开新障碍
 *        @note 此测试验证 D* Lite 的"地图同步"机制:
 *              engine 通过回调函数读取 GridMap,
 *              当 GridMap 被外部直接修改时 engine 不知情;
 *              syncMap() 重新扫描所有节点, 将 rhs/g 值不一致的节点入队
 *
 * 搜索限制:
 *   MaxExpansions_TerminatesEarly — 1000² 网格, max_expansions=50 → 不挂死
 *
 * Goal tolerance:
 *   GoalTolerance_NearGoalSuccess — 终点及 4 邻居被占据, tolerance=2 →
 *                                    在距离 ≤2 的 cell 终止
 *
 * 3D:
 *   ThreeD_FindsPath_OpenVolume  — 5×5×5 立方体, N26, (0,0,0)→(3,3,3)
 *
 * --- Layer 2: DStarLitePathSearch 集成 ---
 *
 *   PlanSucceeds_OpenGrid        — 10×10 开放网格, goal_tolerance=0.6
 *   PlanFails_GoalBlocked         — 终点及 8 邻居全占据 → nullopt
 *   PlanFails_InvalidStepSize     — step_size=0 → nullopt
 *   OutputPoses_AreWorldCoordinates — 所有输出位姿在 grid 范围内
 *   ReplanAfterObstacleChange    — 初始规划 → grid.setObstacle →
 *                                    同一 planner 实例再次 plan() → 避开新障碍
 *   Reset_ClearsEngineState       — plan() → reset() → 新 goal 的 plan() 成功
 *        @note Reset 测试验证 PlannerConcept 的 reset() 契约:
 *              planner 实例可在收到新目标时被重置,
 *              清除内部 D* Lite 引擎的 rhs/g/key 状态
 *
 * ==========================================================================
 * D* Lite 特有的测试关注点
 * ==========================================================================
 *   -# setObstacle / removeObstacle 是 D* Lite 的原生接口,
 *      直接修改引擎内部节点状态并更新 priority queue;
 *      这与 syncMap (通过回调重新发现变化) 是两条不同的变更路径
 *   -# extractPath() 在目标不可达时返回空 vector (不是 nullopt),
 *      与 A* 的 findPath 返回 std::optional 不同 — 这是历史 API 差异
 *   -# DStarLiteAlgorithm 的构造函数需要 rows/cols 尺寸参数
 *      (不像 AStarAlgorithm 从 lambda 推断), 因为引擎需预分配
 *      全部节点的 rhs/g 值存储
 *   -# TestMapView 实现了 DStarLitePathSearch::IMapView 的全部接口:
 *      isWalkable + getCost + getStepSize (继承自 finenav::IMapView)
 *      + rows() + cols() + layers() (D* Lite 特有, 用于引擎初始化)
 *
 * @author Jinghui Wang (wangjinghui@sjtu.edu.cn)
 * @author IWIN-FINS Lab, Shanghai Jiao Tong University
 * @date 2026-07-19
 */

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "finenav_route_planner/dstarlite/dstarlite_algorithm.hpp"
#include "finenav_route_planner/dstarlite_path_search.hpp"
#include "grid_map.hpp"

// ═══════════════════════════════════════════════════════════════════
// Shared helpers
// ═══════════════════════════════════════════════════════════════════

// Build a DStarLiteAlgorithm over a GridMap using lambda callbacks.
inline dstarlite::DStarLiteAlgorithm makeEngine(
    const astar::GridMap& g,
    astar::Connectivity  conn = astar::Connectivity::N8,
    astar::Heuristic     heur = astar::Heuristic::Diagonal)
{
    auto is_walkable = [g](const astar::Vec3& c) {
        return g.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [g](const astar::Vec3& c) {
        return g.getCostCell(c.x, c.y);
    };
    return dstarlite::DStarLiteAlgorithm(g.getHeight(), g.getWidth(),
                                 is_walkable, get_cost, conn, heur);
}

// ═══════════════════════════════════════════════════════════════════
// Group 1: Core DStarLiteAlgorithm tests (grid-space, no IMapView)
// ═══════════════════════════════════════════════════════════════════

// ── Basic paths ──────────────────────────────────────────────────

TEST(DStarLiteAlgorithm, FindsPath_OpenSpace)
{
    auto engine = makeEngine(astar::GridMap(20, 20));
    engine.initialize({0, 0, 0}, {5, 5, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path.back(),  (astar::Vec3{5, 5, 0}));
}

TEST(DStarLiteAlgorithm, StartEqualsGoal)
{
    auto engine = makeEngine(astar::GridMap(10, 10));
    engine.initialize({3, 3, 0}, {3, 3, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.size(), 1u);
    EXPECT_EQ(path.front(), (astar::Vec3{3, 3, 0}));
}

TEST(DStarLiteAlgorithm, Diagonal_ShorterPath)
{
    auto ortho = makeEngine(astar::GridMap(20, 20), astar::Connectivity::N4);
    ortho.initialize({0, 0, 0}, {5, 5, 0});
    ortho.computeShortestPath();
    auto path_ortho = ortho.extractPath();

    auto diag = makeEngine(astar::GridMap(20, 20), astar::Connectivity::N8);
    diag.initialize({0, 0, 0}, {5, 5, 0});
    diag.computeShortestPath();
    auto path_diag = diag.extractPath();

    ASSERT_FALSE(path_ortho.empty());
    ASSERT_FALSE(path_diag.empty());
    EXPECT_LT(path_diag.size(), path_ortho.size());
}

TEST(DStarLiteAlgorithm, PathIsContinuous_N4)
{
    auto engine = makeEngine(astar::GridMap(10, 10), astar::Connectivity::N4);
    engine.initialize({1, 1, 0}, {5, 5, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_GE(path.size(), 2u);
    for (size_t i = 1; i < path.size(); ++i) {
        int diff = std::abs(path[i].x - path[i-1].x)
                 + std::abs(path[i].y - path[i-1].y)
                 + std::abs(path[i].z - path[i-1].z);
        EXPECT_EQ(diff, 1) << "Non-adjacent cells at index " << i;
    }
}

// ── Obstacle handling ────────────────────────────────────────────

TEST(DStarLiteAlgorithm, GoalBlocked_ReturnsEmpty)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    auto engine = makeEngine(grid);
    engine.initialize({0, 0, 0}, {5, 5, 0});
    engine.computeShortestPath();
    EXPECT_TRUE(engine.extractPath().empty());
}

TEST(DStarLiteAlgorithm, PathAvoidsObstacles)
{
    astar::GridMap grid(10, 10);
    // Build a vertical wall at x=3
    for (int y = 0; y <= 8; ++y) grid.setObstacle(3, y, true);

    auto engine = makeEngine(grid, astar::Connectivity::N4);
    engine.initialize({0, 5, 0}, {6, 5, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    for (const auto& c : path) {
        EXPECT_FALSE(c.x == 3 && c.y <= 8)
            << "Path passes through obstacle at (" << c.x << "," << c.y << ")";
    }
}

// ── Incremental replanning ───────────────────────────────────────

TEST(DStarLiteAlgorithm, Replan_AdaptsToNewObstacle)
{
    auto engine = makeEngine(astar::GridMap(10, 10));
    engine.initialize({0, 0, 0}, {9, 9, 0});
    engine.computeShortestPath();
    auto path1 = engine.extractPath();
    ASSERT_FALSE(path1.empty());

    // Block the path with a wall
    for (int y = 0; y < 7; ++y)
        engine.setObstacle({5, y, 0});

    engine.replan({0, 0, 0});
    auto path2 = engine.extractPath();
    ASSERT_FALSE(path2.empty());

    // New path must not pass through blocked cells
    for (const auto& c : path2) {
        EXPECT_FALSE(c.x == 5 && c.y < 7)
            << "Replanned path passes through obstacle at ("
            << c.x << "," << c.y << ")";
    }
}

TEST(DStarLiteAlgorithm, RemoveObstacle_OpensNewPath)
{
    // Full vertical wall at x=5
    astar::GridMap grid(10, 10);
    for (int y = 0; y <= 9; ++y) grid.setObstacle(5, y, true);
    auto engine = makeEngine(grid);

    engine.initialize({0, 5, 0}, {9, 5, 0});
    engine.computeShortestPath();
    auto blocked_path = engine.extractPath();
    EXPECT_TRUE(blocked_path.empty()) << "Should be blocked by wall";

    // Remove one cell to open a gap
    engine.removeObstacle({5, 5, 0});
    engine.replan({0, 5, 0});
    auto open_path = engine.extractPath();

    ASSERT_FALSE(open_path.empty());
    bool passes_gap = false;
    for (const auto& c : open_path) {
        if (c.x == 5 && c.y == 5) { passes_gap = true; break; }
    }
    EXPECT_TRUE(passes_gap);
}

TEST(DStarLiteAlgorithm, SyncMap_DetectsChanges)
{
    // Start with an all-free map, then add an obstacle externally.
    astar::GridMap grid(10, 10);

    auto is_walkable = [&grid](const astar::Vec3& c) {
        return grid.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [&grid](const astar::Vec3& c) {
        return grid.getCostCell(c.x, c.y);
    };

    dstarlite::DStarLiteAlgorithm engine(grid.getHeight(), grid.getWidth(),
                                 is_walkable, get_cost);

    engine.initialize({0, 0, 0}, {9, 9, 0});
    engine.computeShortestPath();
    auto path1 = engine.extractPath();
    ASSERT_FALSE(path1.empty());

    // Modify the grid externally — engine doesn't know yet.
    grid.setObstacle(5, 5, true);

    // syncMap discovers the change and triggers vertex updates.
    engine.syncMap();
    engine.replan({0, 0, 0});
    auto path2 = engine.extractPath();
    ASSERT_FALSE(path2.empty());

    // The new path must not go through the newly-blocked cell.
    for (const auto& c : path2) {
        EXPECT_FALSE(c.x == 5 && c.y == 5)
            << "Path passes through newly-blocked cell";
    }
}

// ── Limits ──────────────────────────────────────────────────────

TEST(DStarLiteAlgorithm, MaxExpansions_TerminatesEarly)
{
    auto engine = makeEngine(astar::GridMap(1000, 1000));
    engine.setMaxExpansions(50);
    engine.setGoalTolerance(0);
    engine.initialize({0, 0, 0}, {999, 999, 0});
    engine.computeShortestPath();
    // Should not hang — test that it terminates.
    SUCCEED();
}

// ── Goal tolerance ──────────────────────────────────────────────

TEST(DStarLiteAlgorithm, GoalTolerance_NearGoalSuccess)
{
    astar::GridMap grid(10, 10);
    // Block goal (5,5) and its immediate Manhattan neighbours
    grid.setObstacle(5, 5, true);
    grid.setObstacle(4, 5, true);
    grid.setObstacle(5, 4, true);
    grid.setObstacle(6, 5, true);
    grid.setObstacle(5, 6, true);

    auto engine = makeEngine(grid);
    engine.setGoalTolerance(2);
    engine.initialize({0, 0, 0}, {5, 5, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    // The effective goal should be within 2 cells of (5,5)
    int dist = std::abs(path.back().x - 5) + std::abs(path.back().y - 5);
    EXPECT_LE(dist, 2);
}

// ── 3-D ─────────────────────────────────────────────────────────

TEST(DStarLiteAlgorithm, ThreeD_FindsPath_OpenVolume)
{
    auto walk = [](const astar::Vec3&) { return true; };
    auto cost = [](const astar::Vec3&) { return 0.0; };

    dstarlite::DStarLiteAlgorithm engine(5, 5, walk, cost,
                                 astar::Connectivity::N26,
                                 astar::Heuristic::Diagonal,
                                 5);  // 5 layers
    engine.initialize({0, 0, 0}, {3, 3, 3});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path.back(),  (astar::Vec3{3, 3, 3}));
}

// ═══════════════════════════════════════════════════════════════════
// Group 2: DStarLitePathSearch integration tests (IMapView + planner)
// ═══════════════════════════════════════════════════════════════════

// TestMapView: wraps GridMap into DStarLitePathSearch::IMapView.
// Must provide isWalkable, getCost, getStepSize (from finenav::IMapView)
// PLUS rows, cols, layers (from DStarLitePathSearch::IMapView).
class TestMapView : public finenav::DStarLitePathSearch::IWorldView {
public:
    TestMapView(const astar::GridMap& g, double step_size)
        : grid_(g), step_(step_size) {}

    bool isWalkable(const finenav::common::Position3D& p) const override {
        return grid_.isWalkableCell(
            static_cast<int>(std::floor(p.x / step_)),
            static_cast<int>(std::floor(p.y / step_)));
    }
    double getCost(const finenav::common::Position3D& p) const override {
        return grid_.getCostCell(
            static_cast<int>(std::floor(p.x / step_)),
            static_cast<int>(std::floor(p.y / step_)));
    }
    double getStepSize() const override { return step_; }

    int rows()   const override { return grid_.getHeight(); }
    int cols()   const override { return grid_.getWidth(); }
    int layers() const override { return 1; }

private:
    const astar::GridMap& grid_;
    double step_;
};

inline void configurePlanner(finenav::DStarLitePathSearch& planner)
{
    dstarlite_planner::Params p;
    p.connectivity      = "8";
    p.heuristic         = "Diagonal";
    p.goal_tolerance    = 0.6;
    p.max_expansions    = 1000000;
    p.max_path_steps    = 10000;
    p.max_planning_time = 5.0;
    planner.configure(p);
}

// PlanningContext stores const references into owned storage, so the owned
// RobotState / ref_traj must outlive every plan() call.  Bundle them so a
// temporary can convert to `const PlanningContext&` at each call site.
struct DStarLiteTestCtx {
    finenav::core::RobotState robot_state;
    finenav::core::InputTrajectoryT<finenav::DStarLitePathSearch> ref_traj;
    finenav::PlanningContext<finenav::DStarLitePathSearch> ctx{robot_state, ref_traj};

    DStarLiteTestCtx(double sx, double sy, double gx, double gy)
    {
        robot_state.pose.position.x = sx;
        robot_state.pose.position.y = sy;
        ref_traj.poses.resize(1);
        ref_traj.poses[0].position.x = gx;
        ref_traj.poses[0].position.y = gy;
        ref_traj.poses[0].orientation.w = 1.0;
    }

    operator const finenav::PlanningContext<finenav::DStarLitePathSearch>&() const
    {
        return ctx;
    }
};

inline DStarLiteTestCtx makeCtx(double sx, double sy, double gx, double gy)
{
    return DStarLiteTestCtx{sx, sy, gx, gy};
}

TEST(DStarLitePathSearch, PlanSucceeds_OpenGrid)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);
    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_GE(result->poses.size(), 1u);
}

TEST(DStarLitePathSearch, PlanFails_GoalBlocked)
{
    astar::GridMap grid(10, 10);
    // Block the goal and all neighbours so goal tolerance can't rescue.
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
            grid.setObstacle(5 + dx, 5 + dy, true);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view).has_value());
}

TEST(DStarLitePathSearch, PlanFails_InvalidStepSize)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 0.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view).has_value());
}

TEST(DStarLitePathSearch, OutputPoses_AreWorldCoordinates)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);
    auto result = planner.plan(makeCtx(0.5, 0.5, 3.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    for (const auto& pose : result->poses) {
        EXPECT_GE(pose.position.x, 0.0);
        EXPECT_LT(pose.position.x, 10.0);
        EXPECT_GE(pose.position.y, 0.0);
        EXPECT_LT(pose.position.y, 10.0);
    }
}

TEST(DStarLitePathSearch, ReplanAfterObstacleChange)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);

    // First plan: open grid.
    auto result1 = planner.plan(makeCtx(0.5, 0.5, 9.5, 9.5), view);
    ASSERT_TRUE(result1.has_value());
    ASSERT_GE(result1->poses.size(), 1u);

    // Add obstacles on the original path.
    grid.setObstacle(5, 5, true);
    grid.setObstacle(5, 4, true);
    grid.setObstacle(5, 6, true);

    // Replan — same planner instance, same goal, updated map.
    auto result2 = planner.plan(makeCtx(0.5, 0.5, 9.5, 9.5), view);
    ASSERT_TRUE(result2.has_value());

    // The new path must avoid the obstacles.
    for (const auto& pose : result2->poses) {
        int cx = static_cast<int>(std::floor(pose.position.x));
        int cy = static_cast<int>(std::floor(pose.position.y));
        EXPECT_FALSE(cx == 5 && cy == 5)
            << "Replanned path passes through obstacle at (" << cx << "," << cy << ")";
        EXPECT_FALSE(cx == 5 && cy == 4);
        EXPECT_FALSE(cx == 5 && cy == 6);
    }
}

TEST(DStarLitePathSearch, Reset_ClearsEngineState)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);

    // Initial plan.
    auto result1 = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result1.has_value());

    // Reset — should clear the engine.
    planner.reset();

    // After reset, a new plan with a different goal should work.
    auto result2 = planner.plan(makeCtx(0.5, 0.5, 8.5, 2.5), view);
    ASSERT_TRUE(result2.has_value());
}
