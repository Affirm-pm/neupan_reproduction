// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "finenav_localizer/sources/velocity_provider.hpp"

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

TEST(VelocityProvider, SetFromTwist) {
    VelocityProvider vp;
    auto s = makeState();

    vp.setFromTwist(Eigen::Vector3d(1, 2, 0), Eigen::Vector3d(0, 0, 0.5), s);

    EXPECT_NEAR(s.linear_velocity.x(), 1.0, kEps);
    EXPECT_NEAR(s.linear_velocity.y(), 2.0, kEps);
    EXPECT_NEAR(s.angular_velocity.z(), 0.5, kEps);
}

TEST(VelocityProvider, AutoDiffFirstCallNoOp) {
    VelocityProvider vp;
    auto s = makeState();
    s.linear_velocity = Eigen::Vector3d(9, 9, 9);

    vp.autoDiffFromPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity(),
                        t(0), s);

    EXPECT_NEAR(s.linear_velocity.x(), 9.0, kEps);
}

TEST(VelocityProvider, AutoDiffComputesVelocity) {
    VelocityProvider vp;
    auto s = makeState();

    vp.autoDiffFromPose(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity(),
                        t(0), s);
    vp.autoDiffFromPose(Eigen::Vector3d(2, 0, 0), Eigen::Quaterniond::Identity(),
                        t(1), s);

    EXPECT_NEAR(s.linear_velocity.x(), 2.0, kEps);
}

}  // namespace
}  // namespace finenav::localizer

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
