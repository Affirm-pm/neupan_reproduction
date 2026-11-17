// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/**
 * @file detour_test.cpp
 * @brief DetourPlanner unit tests — re-anchoring, blockage detection, detour generation
 *
 * @details
 * ==========================================================================
 * 测试总览
 * ==========================================================================
 * DetourPlanner 是 FineNav pipeline 中的局部重规划层，每周期被 TrackingPolicy
 * 调用，负责：将机器人重新锚定到参考轨迹上、检测前方障碍并生成绕障路径。
 *
 * 测试分组:
 *   -# 输出契约 — 路径始终从 robot 位姿开始 (re-anchoring)
 *   -# 致死阻塞 — !isWalkable → detour 绕过，rejoin
 *   -# 软阻塞 — cost >= blockage_threshold → detour
 *   -# 防御 — 空参考轨迹、全阻塞 pass-through
 *
 * ==========================================================================
 * 测试环境
 * ==========================================================================
 * 测试框架:     Google Test (GTest)
 * 依赖:         finenav_route_planner, common/grid_map.hpp
 * ROS 依赖:     rclcpp (日志), geometry_msgs (Pose)
 * 运行:
 * @code
 *   cd ~/FineNav_ws
 *   colcon test --packages-select finenav_route_planner
 *   ./build/finenav_route_planner/test/detour_test
 * @endcode
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "finenav_route_planner/detour_planner.hpp"
#include "grid_map.hpp"

namespace {

using finenav::common::Position3D;

// ── Test map view (same adaptor pattern as route_test.cpp) ───────────────
class TestMapView : public astar::IWorldView {
public:
    TestMapView(const astar::GridMap& g, double step_size)
        : grid_(g), step_(step_size) {}

    bool isWalkable(const Position3D& p) const override {
        return grid_.isWalkableCell(
            static_cast<int>(std::floor(p.x / step_)),
            static_cast<int>(std::floor(p.y / step_)));
    }
    double getCost(const Position3D& p) const override {
        return grid_.getCostCell(
            static_cast<int>(std::floor(p.x / step_)),
            static_cast<int>(std::floor(p.y / step_)));
    }
    double getStepSize() const override { return step_; }

private:
    const astar::GridMap& grid_;
    double step_;
};

// ── helpers ──────────────────────────────────────────────────────────────

finenav::DetourPlanner makePlanner()
{
    finenav::DetourPlanner planner;
    detour_planner::Params p;
    p.scan_distance_m      = 10.0;
    p.blockage_threshold   = 0.8;
    p.reconnect_offset_m   = 1.0;
    p.obstacle_cost_weight = 1.0;
    p.max_planning_time    = 1.0;
    planner.configure(p);
    return planner;
}

// Straight reference path along y = `y`, x from x0 to x1 at `spacing`.
std::vector<geometry_msgs::msg::Pose> makeRefPath(
    double x0, double x1, double y, double spacing = 0.5)
{
    std::vector<geometry_msgs::msg::Pose> poses;
    for (double x = x0; x <= x1 + 1e-9; x += spacing) {
        geometry_msgs::msg::Pose p;
        p.position.x    = x;
        p.position.y    = y;
        p.orientation.w = 1.0;
        poses.push_back(p);
    }
    return poses;
}

// PlanningContext stores const references into owned storage, so the owned
// RobotState / ref_traj must outlive every plan() call.  Bundle them so a
// temporary can convert to `const PlanningContext&` at each call site.
struct DetourTestCtx {
    finenav::core::RobotState robot_state;
    finenav::core::InputTrajectoryT<finenav::DetourPlanner> ref_traj;
    finenav::PlanningContext<finenav::DetourPlanner> ctx{robot_state, ref_traj};

    DetourTestCtx(double rx, double ry, std::vector<geometry_msgs::msg::Pose> ref)
    {
        robot_state.pose.position.x = rx;
        robot_state.pose.position.y = ry;
        robot_state.pose.position.z = 0.0;
        robot_state.pose.orientation.w = 1.0;
        ref_traj.poses = std::move(ref);
    }

    operator const finenav::PlanningContext<finenav::DetourPlanner>&() const
    {
        return ctx;
    }
};

inline DetourTestCtx makeCtx(
    double rx, double ry, std::vector<geometry_msgs::msg::Pose> ref)
{
    return DetourTestCtx{rx, ry, std::move(ref)};
}

void expectSamePath(const std::vector<geometry_msgs::msg::Pose>& a,
                    const std::vector<geometry_msgs::msg::Pose>& b)
{
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_NEAR(a[i].position.x, b[i].position.x, 1e-9);
        EXPECT_NEAR(a[i].position.y, b[i].position.y, 1e-9);
    }
}

}  // namespace

// =========================================================================
// Output contract
// =========================================================================

TEST(DetourPlanner, EmptyReference_ReturnsEmptyOutput)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, {}), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->poses.empty());
}

TEST(DetourPlanner, ClearPath_RobotAtStart_OutputIdentical)
{
    // Robot standing exactly on the first reference waypoint: re-anchoring
    // degenerates to the identity — output equals the reference.
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(0.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());
    expectSamePath(result->poses, ref);
}

TEST(DetourPlanner, ClearPath_RobotMidPath_DropsWaypointsBehind)
{
    // Robot on the path but halfway along it: output starts at the robot
    // and the reference prefix behind the robot is dropped.
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(8.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    EXPECT_NEAR(result->poses.front().position.x, 8.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);

    for (const auto& p : result->poses)
        EXPECT_GE(p.position.x, 8.5 - 1e-6)
            << "Kept waypoint behind robot: x=" << p.position.x;
}

// =========================================================================
// Lethal blockage
// =========================================================================

TEST(DetourPlanner, HijackedRobot_ReanchorsFromRobot)
{
    // Robot displaced off the reference (manual override), with a wall
    // between it and the path.  The output must start at the robot and
    // reconnect to the reference around the wall.
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // Wall row y=2, x=0..4: blocks the straight line from robot to path.
    for (int x = 0; x <= 4; ++x) grid.setObstacle(x, 2, true);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 4.5, ref), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // Starts at the (displaced) robot pose.
    EXPECT_NEAR(result->poses.front().position.x, 2.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 4.5, 1e-6);

    // Never crosses the wall.
    for (const auto& p : result->poses)
        EXPECT_FALSE(p.position.x < 5.0 && p.position.y >= 2.0 && p.position.y < 3.0)
            << "Crossed wall at (" << p.position.x << ", " << p.position.y << ")";

    // Rejoins the reference and reaches its end.
    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);
}

TEST(DetourPlanner, LethalBlockage_DetoursAroundAndRejoins)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // Wall at x=8, rows y=0..2 — blocks the path, forces a detour over y=3.
    for (int y = 0; y <= 2; ++y) grid.setObstacle(8, y, true);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // Output starts at the robot.
    EXPECT_NEAR(result->poses.front().position.x, 2.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);

    // Ends at the reference end.
    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);

    // Never enters the wall column below the gap.
    for (const auto& p : result->poses)
        EXPECT_FALSE(p.position.x > 8.0 && p.position.x < 9.0 && p.position.y < 3.0)
            << "Entered wall at (" << p.position.x << ", " << p.position.y << ")";

    // The detour climbs over the wall (some point at y >= 3).
    bool climbed = false;
    for (const auto& p : result->poses)
        if (p.position.y >= 3.0) climbed = true;
    EXPECT_TRUE(climbed);
}

// =========================================================================
// Soft blockage (cost threshold)
// =========================================================================

TEST(DetourPlanner, SoftBlockage_TriggersOnCostThreshold)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // High cost (still walkable) on the path cell at x=8.
    grid.setCost(8, 0, 0.95);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());

    // A detour happened: the output bulges away from the straight line
    // (the cost-weighted search sidesteps the expensive cell).
    bool bulged = false;
    for (const auto& p : result->poses)
        if (p.position.y > 1.0) bulged = true;
    EXPECT_TRUE(bulged);

    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);
}

// =========================================================================
// Long obstacle / fully blocked
// =========================================================================

TEST(DetourPlanner, LongObstacle_SkipsUnreachableCandidates_RejoinsBeyond)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // Wall covering x=6..10, rows y=0..2: candidates within
    // reconnect_offset_m of the blockage are unreachable (inside the wall)
    // and must be skipped automatically.
    for (int x = 6; x <= 10; ++x)
        for (int y = 0; y <= 2; ++y)
            grid.setObstacle(x, y, true);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());

    // While crossing the wall span, the path must stay above it.
    for (const auto& p : result->poses)
        EXPECT_FALSE(p.position.x > 6.0 && p.position.x < 11.0 && p.position.y < 3.0)
            << "Entered wall at (" << p.position.x << ", " << p.position.y << ")";

    // Rejoins the reference beyond the wall and reaches its end.
    bool rejoined = false;
    for (const auto& p : result->poses)
        if (std::abs(p.position.y - 0.5) < 1e-6 && p.position.x >= 11.0) rejoined = true;
    EXPECT_TRUE(rejoined);
    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);
}

TEST(DetourPlanner, FullyBlocked_PassesThrough)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // Wall across the full grid height at x=8 — no reconnect candidate is
    // reachable; the planner must warn and pass the reference through.
    for (int y = 0; y < 10; ++y) grid.setObstacle(8, y, true);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());
    expectSamePath(result->poses, ref);
}
