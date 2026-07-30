// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "finenav_localizer/sources/pose_anchor.hpp"

namespace finenav::localizer {
namespace {

using namespace finenav::core;

constexpr double kEps = 1e-9;

rclcpp::Time t(int64_t sec) {
    return rclcpp::Time(sec * 1000000000LL, RCL_SYSTEM_TIME);
}

RobotState makeState(double x = 0, double y = 0, double z = 0) {
    RobotState s;
    s.position         = Eigen::Vector3d(x, y, z);
    s.orientation      = Eigen::Quaterniond::Identity();
    s.linear_velocity  = Eigen::Vector3d::Zero();
    s.angular_velocity = Eigen::Vector3d::Zero();
    return s;
}

TEST(PoseAnchor, TickReturnsNulloptWhenNoData) {
    PoseAnchor pa;
    auto s = makeState();
    EXPECT_FALSE(pa.tick(s).has_value());
    EXPECT_FALSE(pa.hasAbsolutePoseData());
}

TEST(PoseAnchor, InstantSmoothingConvergesInOneTick) {
    PoseAnchor pa;
    pa.configure(0.0, 100.0);
    auto s = makeState(1, 2, 0);

    pa.feed(Eigen::Vector3d(10, 20, 0), Eigen::Quaterniond::Identity(), t(1));

    auto corr = pa.tick(s);
    ASSERT_TRUE(corr.has_value());
    EXPECT_NEAR(corr->position_delta.x(), 9.0, kEps);
    EXPECT_NEAR(corr->position_delta.y(), 18.0, kEps);

    EXPECT_FALSE(pa.tick(s).has_value());
}

TEST(PoseAnchor, SmoothingGradualConvergence) {
    PoseAnchor pa;
    pa.configure(0.5, 100.0);
    auto s = makeState();

    pa.feed(Eigen::Vector3d(10, 0, 0), Eigen::Quaterniond::Identity(), t(0));

    double moved = 0.0;
    int ticks = 0;
    while (ticks < 5000) {
        auto c = pa.tick(s);
        if (!c.has_value()) break;
        s.position += c->position_delta;
        moved += c->position_delta.norm();
        ++ticks;
    }
    EXPECT_LT(ticks, 5000);
    EXPECT_GT(moved, 0.0);
    EXPECT_NEAR(s.position.x(), 10.0, 1e-3);
}

TEST(PoseAnchor, NewFeedReplacesTarget) {
    PoseAnchor pa;
    pa.configure(0.0, 100.0);
    auto s = makeState();

    pa.feed(Eigen::Vector3d(5, 0, 0), Eigen::Quaterniond::Identity(), t(0));
    pa.feed(Eigen::Vector3d(10, 0, 0), Eigen::Quaterniond::Identity(), t(1));

    auto corr = pa.tick(s);
    ASSERT_TRUE(corr.has_value());
    EXPECT_NEAR(corr->position_delta.x(), 10.0, kEps);
}

}  // namespace
}  // namespace finenav::localizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
