// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "test_helpers.hpp"
#include "finenav_core/robot_state/eigen_conversions.hpp"

using finenav::core::fromMsg;

// ── push one → latest returns it ──────────────────────────────────────────────
TEST(RobotStateProvider, PushOne_LatestReturnsIt)
{
    TestProvider p;
    auto state = makeState(3.0, -1.5, 0.8);
    state.header.stamp = ts(1);
    p.pushState(state);
    auto rs = p.getRobotState();
    ASSERT_TRUE(rs.has_value());
    EXPECT_NEAR(rs->pose.position.x, 3.0, 1e-9);
    EXPECT_NEAR(rs->pose.position.y, -1.5, 1e-9);
}

// ── push N → latest is the most recent ─────────────────────────────────────────
TEST(RobotStateProvider, PushN_LatestIsMostRecent)
{
    TestProvider p;
    auto s1 = makeState(0.0, 0.0, 0.0);
    s1.header.stamp = ts(1);
    p.pushState(s1);
    auto s2 = makeState(2.0, 0.0, 0.0);
    s2.header.stamp = ts(2);
    p.pushState(s2);
    auto rs = p.getRobotState();
    ASSERT_TRUE(rs.has_value());
    EXPECT_NEAR(rs->pose.position.x, 2.0, 1e-9);
}

// ── interpolate between two states at mid-point ────────────────────────────────
TEST(RobotStateProvider, InterpolateAtMidpoint)
{
    TestProvider p;
    auto s1 = makeState(0.0, 0.0, 0.0);
    s1.header.stamp = ts(1);
    p.pushState(s1);
    auto s2 = makeState(2.0, 0.0, 0.0);
    s2.header.stamp = ts(2);
    p.pushState(s2);

    auto rs = p.getRobotState(tf(1.5));
    ASSERT_TRUE(rs.has_value());
    EXPECT_NEAR(rs->pose.position.x, 1.0, 1e-9);
    EXPECT_NEAR(rs->pose.position.y, 0.0, 1e-9);

    Eigen::Quaterniond ori;
    fromMsg(*rs, ori);
    EXPECT_NEAR(ori.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-9);
}

// ── query outside window → nullopt ─────────────────────────────────────────────
TEST(RobotStateProvider, OutOfWindow_ReturnsNullopt)
{
    TestProvider p(1.0);
    auto s1 = makeState(0.0, 0.0, 0.0);
    s1.header.stamp = ts(1);
    p.pushState(s1);
    auto s2 = makeState(1.0, 0.0, 0.0);
    s2.header.stamp = ts(2);
    p.pushState(s2);
    EXPECT_FALSE(p.getRobotState(tf(0.5)).has_value());
}

// ── future query → nullopt ─────────────────────────────────────────────────────
TEST(RobotStateProvider, FutureQuery_ReturnsNullopt)
{
    TestProvider p;
    auto s1 = makeState(0.0, 0.0, 0.0);
    s1.header.stamp = ts(1);
    p.pushState(s1);
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
    auto s1 = makeState(1.0, 2.0, 0.0);
    s1.header.stamp = ts(1);
    p.pushState(s1);
    ASSERT_TRUE(p.getRobotState().has_value());

    p.resetHistory();
    EXPECT_FALSE(p.getRobotState().has_value());
}

// ── setHistoryDuration(0) prunes to latest only ────────────────────────────────
TEST(RobotStateProvider, SetHistoryDurationZero_KeepsOnlyLatest)
{
    TestProvider p(1.0);
    auto s1 = makeState(0.0, 0.0, 0.0);
    s1.header.stamp = ts(1);
    p.pushState(s1);
    auto s2 = makeState(3.0, 0.0, 0.0);
    s2.header.stamp = ts(2);
    p.pushState(s2);
    ASSERT_TRUE(p.getRobotState(tf(1.5)).has_value());

    p.setHistoryDuration(0.0);
    EXPECT_TRUE(p.getRobotState().has_value());
    EXPECT_FALSE(p.getRobotState(ts(1)).has_value());
}
