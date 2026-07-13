// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "finenav_localizer/kalman_filter/time_delay_kalman_filter.hpp"

using namespace finenav::localizer;

// 2 slots, dim_x = 3  →  dim_x_ex = 6.
static Eigen::MatrixXd state3()   { Eigen::MatrixXd x(3, 1); x << 1, 2, 3; return x; }
static Eigen::MatrixXd cov3()     { return Eigen::Matrix3d::Identity() * 2.0; }

// ── init ──────────────────────────────────────────────────────────────────────
TEST(TimeDelayKF, Init_AllSlotsSeeded)
{
    TimeDelayKalmanFilter tdkf;
    ASSERT_TRUE(tdkf.init(state3(), cov3(), 2));  // 2 slots, dim_x=3 → dim_x_ex=6
    EXPECT_EQ(tdkf.dimX(),   3);
    EXPECT_EQ(tdkf.dimXEx(), 6);
    EXPECT_EQ(tdkf.maxDelayStep(), 2);

    // Both slots have the same initial state.
    EXPECT_NEAR(tdkf.getLatestX().norm(), state3().norm(), 1e-10);
    EXPECT_NEAR((tdkf.getXAtSlot(1) - state3()).norm(), 0.0, 1e-10);
}

TEST(TimeDelayKF, Init_RejectsEmpty)
{
    TimeDelayKalmanFilter tdkf;
    EXPECT_FALSE(tdkf.init(Eigen::MatrixXd(), cov3(), 2));          // empty x
    EXPECT_FALSE(tdkf.init(state3(), Eigen::MatrixXd(), 2));        // empty P0
    EXPECT_FALSE(tdkf.init(state3(), cov3(), 0));                   // max_delay < 1
}

// ── predictWithDelay: block-slide structure ───────────────────────────────────
TEST(TimeDelayKF, PredictWithDelay_BlockSlideStructure)
{
    TimeDelayKalmanFilter tdkf;
    ASSERT_TRUE(tdkf.init(state3(), cov3(), 2));  // dim_x=3, dim_ex=6

    Eigen::MatrixXd x_next(3, 1); x_next.setZero();
    Eigen::MatrixXd A = Eigen::Matrix3d::Identity() * 0.5;
    Eigen::MatrixXd Q = Eigen::Matrix3d::Identity() * 0.1;

    ASSERT_TRUE(tdkf.predictWithDelay(x_next, A, Q));

    // After slide: slot 0 = zeros, slot 1 = old slot 0 (state3).
    EXPECT_NEAR(tdkf.getLatestX().norm(), 0.0, 1e-10);
    EXPECT_NEAR((tdkf.getXAtSlot(1) - state3()).norm(), 0.0, 1e-10);

    // P(0,0) = A·P00·Aᵀ + Q = 0.5I·2I·0.5I + 0.1I = 0.6I
    const auto P00 = tdkf.getLatestP();
    EXPECT_NEAR(P00(0,0), 0.6, 1e-10);
    EXPECT_NEAR(P00(1,1), 0.6, 1e-10);

    // P(1,1) = old P(0,0) = 2I  (the old leading block slid into slot 1)
    const auto P11 = tdkf.getPAtSlot(1);
    EXPECT_NEAR(P11(0,0), 2.0, 1e-10);
}

// ── updateWithDelay: slot guard ────────────────────────────────────────────────
TEST(TimeDelayKF, UpdateWithDelay_SlotGuard)
{
    TimeDelayKalmanFilter tdkf;
    ASSERT_TRUE(tdkf.init(state3(), cov3(), 2));
    EXPECT_FALSE(tdkf.updateWithDelay(Eigen::Vector3d::Zero(),
                                      Eigen::Matrix3d::Identity(),
                                      Eigen::Matrix3d::Identity(), 5)); // ≥ max
}

// ── updateWithTwoSlots: slot guard ─────────────────────────────────────────────
TEST(TimeDelayKF, UpdateWithTwoSlots_SlotGuard)
{
    TimeDelayKalmanFilter tdkf;
    ASSERT_TRUE(tdkf.init(state3(), cov3(), 2));
    EXPECT_FALSE(tdkf.updateWithTwoSlots(Eigen::Vector3d::Zero(),
                                         Eigen::Matrix3d::Identity(), 0,
                                         Eigen::Matrix3d::Identity(), 3,
                                         Eigen::Matrix3d::Identity()));
}

// ── getPAtSlot clamps ──────────────────────────────────────────────────────────
TEST(TimeDelayKF, GetPAtSlot_Clamps)
{
    TimeDelayKalmanFilter tdkf;
    ASSERT_TRUE(tdkf.init(state3(), cov3(), 2));
    // slot 9 exceeds max → clamps to slot 1 (max_delay_step-1 = 1)
    const auto P1 = tdkf.getPAtSlot(1);
    const auto P9 = tdkf.getPAtSlot(9);
    EXPECT_NEAR((P9 - P1).norm(), 0.0, 1e-10);
}

// ── resetState + setLeadingCovariance ──────────────────────────────────────────
TEST(TimeDelayKF, ResetStateAndSetLeadingCovariance)
{
    TimeDelayKalmanFilter tdkf;
    ASSERT_TRUE(tdkf.init(state3(), cov3(), 2));
    tdkf.resetState();
    EXPECT_NEAR(tdkf.getLatestX().norm(), 0.0, 1e-10);

    const Eigen::MatrixXd newP = Eigen::Matrix3d::Identity() * 7.0;
    tdkf.setLeadingCovariance(newP);
    EXPECT_NEAR(tdkf.getLatestP()(0, 0), 7.0, 1e-10);
    // Other slot unchanged.
    EXPECT_NEAR(tdkf.getPAtSlot(1)(0, 0), 2.0, 1e-10);
}
