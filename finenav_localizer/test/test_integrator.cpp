// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "finenav_localizer/sources/integrator.hpp"

namespace finenav::localizer {
namespace {

using namespace finenav::core;

constexpr double kEps = 1e-9;

rclcpp::Time t(int64_t sec) {
    return rclcpp::Time(sec * 1000000000LL, RCL_SYSTEM_TIME);
}

RobotState makeState() {
    RobotState s;
    s.position         = Eigen::Vector3d::Zero();
    s.orientation      = Eigen::Quaterniond::Identity();
    s.linear_velocity  = Eigen::Vector3d::Zero();
    s.angular_velocity = Eigen::Vector3d::Zero();
    return s;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Twist integration
// ══════════════════════════════════════════════════════════════════════════════

TEST(Integrator, TwistFirstMessageStoresOnly) {
    Integrator ig;
    auto s = makeState();

    ig.acceptTwist(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0), t(0), s);
    EXPECT_NEAR(s.position.x(), 0.0, kEps);
}

TEST(Integrator, TwistIntegratesPosition) {
    Integrator ig;
    auto s = makeState();

    ig.acceptTwist(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0), t(0), s);
    ig.acceptTwist(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0), t(1), s);

    EXPECT_NEAR(s.position.x(), 1.0, kEps);
}

TEST(Integrator, TwistIntegratesOrientation) {
    Integrator ig;
    auto s = makeState();

    ig.acceptTwist(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, M_PI_2), t(0), s);
    ig.acceptTwist(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, M_PI_2), t(1), s);

    const double yaw = Eigen::AngleAxisd(s.orientation).angle();
    EXPECT_NEAR(yaw, M_PI_2, 1e-6);
}

TEST(Integrator, TwistRejectsOutOfOrder) {
    Integrator ig;
    auto s = makeState();

    ig.acceptTwist(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(0, 0, 0), t(2), s);
    ig.acceptTwist(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0), t(1), s);  // older — ignored
    ig.acceptTwist(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(0, 0, 0), t(3), s);

    // dt = 3-2 = 1s at 2m/s = 2m
    EXPECT_NEAR(s.position.x(), 2.0, kEps);
}

}  // namespace
}  // namespace finenav::localizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
