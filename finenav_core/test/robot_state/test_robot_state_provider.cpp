// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "test_helpers.hpp"

// ── push one → latest returns it ──────────────────────────────────────────────
TEST(RobotStateProvider, PushOne_LatestReturnsIt)
{
    TestProvider p;
    p.pushState(ts(1), makeState(3.0, -1.5, 0.8));
    auto rs = p.getRobotState();
    ASSERT_TRUE(rs.has_value());
    EXPECT_NEAR(rs->position.x(), 3.0, 1e-9);
    EXPECT_NEAR(rs->position.y(), -1.5, 1e-9);
}

// ── push N → latest is the most recent ─────────────────────────────────────────
TEST(RobotStateProvider, PushN_LatestIsMostRecent)
{
    TestProvider p;
    p.pushState(ts(1), makeState(0.0, 0.0, 0.0));
    p.pushState(ts(2), makeState(2.0, 0.0, 0.0));
    auto rs = p.getRobotState();
    ASSERT_TRUE(rs.has_value());
    EXPECT_NEAR(rs->position.x(), 2.0, 1e-9);
}

// ── interpolate between two states at mid-point ────────────────────────────────
TEST(RobotStateProvider, InterpolateAtMidpoint)
{
    TestProvider p;
    p.pushState(ts(1), makeState(0.0, 0.0, 0.0));
    p.pushState(ts(2), makeState(2.0, 0.0, 0.0));

    auto rs = p.getRobotState(tf(1.5));  // t = 1.5 s
    ASSERT_TRUE(rs.has_value());
    // position: linear interpolation → (1, 0, 0).
    EXPECT_NEAR(rs->position.x(), 1.0, 1e-9);
    EXPECT_NEAR(rs->position.y(), 0.0, 1e-9);
    // orientation: slerp midpoint of 0 and 0 → still ~0
    EXPECT_NEAR(rs->orientation.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-9);
}

// ── query outside window → nullopt ─────────────────────────────────────────────
TEST(RobotStateProvider, OutOfWindow_ReturnsNullopt)
{
    TestProvider p(1.0);  // 1 s window
    p.pushState(ts(1), makeState(0.0, 0.0, 0.0));
    p.pushState(ts(2), makeState(1.0, 0.0, 0.0));
    EXPECT_FALSE(p.getRobotState(tf(0.5)).has_value());
}

// ── future query → nullopt ─────────────────────────────────────────────────────
TEST(RobotStateProvider, FutureQuery_ReturnsNullopt)
{
    TestProvider p;
    p.pushState(ts(1), makeState(0.0, 0.0, 0.0));
    EXPECT_FALSE(p.getRobotState(ts(2)).has_value());
}

// ── empty provider → nullopt ───────────────────────────────────────────────────
TEST(RobotStateProvider, Empty_ReturnsNullopt)
{
    TestProvider p;
    EXPECT_FALSE(p.getRobotState().has_value());
    EXPECT_FALSE(p.getRobotState(ts(0)).has_value());
}

// ── resetHistory → empty again ─────────────────────────────────────────────────
TEST(RobotStateProvider, ResetHistory_Empties)
{
    TestProvider p;
    p.pushState(ts(1), makeState(1.0, 2.0, 0.0));
    ASSERT_TRUE(p.getRobotState().has_value());

    p.resetHistory();
    EXPECT_FALSE(p.getRobotState().has_value());
}

// ── setHistoryDuration(0) prunes to latest only ────────────────────────────────
TEST(RobotStateProvider, SetHistoryDurationZero_KeepsOnlyLatest)
{
    TestProvider p(1.0);
    p.pushState(ts(1), makeState(0.0, 0.0, 0.0));
    p.pushState(ts(2), makeState(3.0, 0.0, 0.0));                     // newest
    ASSERT_TRUE(p.getRobotState(tf(1.5)).has_value());   // at 1.5, both in window

    p.setHistoryDuration(0.0);
    EXPECT_TRUE(p.getRobotState().has_value());          // latest still there
    EXPECT_FALSE(p.getRobotState(ts(1)).has_value());    // t=1 pruned (only latest kept)
}
