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

nav_msgs::msg::Odometry makeOdom(double x, double y, double yaw,
                                  int64_t sec) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec  = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = 0;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    return msg;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Twist
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

// ══════════════════════════════════════════════════════════════════════════════
//  RelPose
// ══════════════════════════════════════════════════════════════════════════════

TEST(Integrator, RelPoseFirstMessageStoresOnly) {
    Integrator ig;
    auto s = makeState();

    ig.acceptRelPose(makeOdom(1, 0, 0, 0), s);
    EXPECT_NEAR(s.position.x(), 0.0, kEps);
}

TEST(Integrator, RelPoseAppliesDelta) {
    Integrator ig;
    auto s = makeState();

    ig.acceptRelPose(makeOdom(0, 0, 0, 0), s);
    ig.acceptRelPose(makeOdom(1, 0, 0, 1), s);

    EXPECT_NEAR(s.position.x(), 1.0, kEps);
}

TEST(Integrator, RelPoseRejectsOutOfOrder) {
    Integrator ig;
    auto s = makeState();

    ig.acceptRelPose(makeOdom(0, 0, 0, 2), s);
    ig.acceptRelPose(makeOdom(9, 0, 0, 1), s);  // older — ignored
    ig.acceptRelPose(makeOdom(2, 0, 0, 3), s);

    EXPECT_NEAR(s.position.x(), 2.0, kEps);
}

}  // namespace
}  // namespace finenav::localizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
