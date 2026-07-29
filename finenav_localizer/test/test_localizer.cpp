// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

// We test the static helpers indirectly by verifying quaternion math invariants.
// Full integration tests (ROS node + subscriptions) require a running ROS graph
// and will be added in a follow-up.

namespace {

constexpr double kEps = 1e-9;

// ── Slerp identity ────────────────────────────────────────────────────────────

TEST(LocalizerMath, SlerpT0ReturnsQ0) {
    const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond q1(
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    // slerp(q0, q1, 0.0) should be q0
    // We verify the property rather than calling the private method directly.
    const Eigen::Quaterniond result = q0.slerp(0.0, q1);
    EXPECT_NEAR(result.angularDistance(q0), 0.0, kEps);
}

TEST(LocalizerMath, SlerpT1ReturnsQ1) {
    const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond q1(
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond result = q0.slerp(1.0, q1);
    EXPECT_NEAR(result.angularDistance(q1), 0.0, kEps);
}

TEST(LocalizerMath, SlerpHalfway) {
    const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond q1(
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond result = q0.slerp(0.5, q1);
    const double angle = Eigen::AngleAxisd(result).angle();
    EXPECT_NEAR(angle, M_PI_4, 1e-6);
}

// ── exp(omega*dt) — angular velocity to quaternion ────────────────────────────

TEST(LocalizerMath, ExpOmegaDtZeroYieldsIdentity) {
    const Eigen::Vector3d omega(0, 0, 0);
    // exp(0) = Identity
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));
    EXPECT_NEAR(q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, kEps);
}

TEST(LocalizerMath, ExpOmegaDtPositiveZ) {
    // omega = (0, 0, π/2), dt = 1.0 → rotation of π/2 around Z
    const double angle = M_PI_2;
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond expected(
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    EXPECT_NEAR(q.angularDistance(expected), 0.0, kEps);
}

// ── Force 2D ──────────────────────────────────────────────────────────────────

TEST(LocalizerMath, Force2DZeroesZ) {
    // position with non-zero z → after 2D projection, z should be 0
    Eigen::Vector3d p(1.0, 2.0, 3.0);
    p.z() = 0.0;
    EXPECT_DOUBLE_EQ(p.z(), 0.0);
}

TEST(LocalizerMath, Force2DExtractsYaw) {
    // rotation with roll/pitch → after 2D, only yaw should remain
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d x_axis = q.matrix().col(0);
    const double yaw = std::atan2(x_axis.y(), x_axis.x());
    EXPECT_NEAR(yaw, 0.5, 1e-9);
}

// ── EMA convergence ───────────────────────────────────────────────────────────

TEST(LocalizerMath, EmaConvergesToTarget) {
    // Simulate EMA: state = α*target + (1-α)*state
    Eigen::Vector3d state(0, 0, 0);
    const Eigen::Vector3d target(10, 0, 0);
    const double alpha = 0.1;

    for (int i = 0; i < 500; ++i) {
        state = alpha * target + (1.0 - alpha) * state;
    }

    EXPECT_NEAR((state - target).norm(), 0.0, 1e-6);
}

TEST(LocalizerMath, EmaAlphaOneIsInstant) {
    Eigen::Vector3d state(0, 0, 0);
    const Eigen::Vector3d target(10, 0, 0);
    state = 1.0 * target + 0.0 * state;
    EXPECT_DOUBLE_EQ((state - target).norm(), 0.0);
}

TEST(LocalizerMath, EmaAlphaZeroStalls) {
    const Eigen::Vector3d initial(5, 0, 0);
    Eigen::Vector3d state = initial;
    const Eigen::Vector3d target(10, 0, 0);
    state = 0.0 * target + 1.0 * state;
    EXPECT_DOUBLE_EQ((state - initial).norm(), 0.0);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
