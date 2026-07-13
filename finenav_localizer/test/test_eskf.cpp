// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <sophus/so3.hpp>

#include "finenav_localizer/eskf.hpp"

using namespace finenav::localizer;
using ESKFd = ESKF<NavigationModel<double>>;

// ── predict: nominal advances + nominal_history_ grows ────────────────────────
TEST(ESKF, Predict_NominalAdvancesAndHistoryGrows)
{
    ESKFd eskf({}, 5);   // 5 delay slots
    const auto init = eskf.getState();
    EXPECT_NEAR(init.v.norm(), 0.0, 1e-10);

    eskf.predict(0.1);
    // Nominal v=0 → position unchanged; R unchanged.
    EXPECT_NEAR(eskf.getState().p.norm(), 0.0, 1e-10);

    // History: one entry pushed (the pre-propagation nominal).
    EXPECT_NEAR((eskf.getStateAtDelay(1).p - init.p).norm(), 0.0, 1e-10);
    // getStateAtDelay(0) returns current nominal (same as getState).
    EXPECT_NEAR((eskf.getStateAtDelay(0).p - eskf.getState().p).norm(), 0.0, 1e-10);
}

// ── getStateAtDelay: falls back to oldest available ────────────────────────────
TEST(ESKF, GetStateAtDelay_Fallback)
{
    ESKFd eskf({}, 5);
    // History empty → returns current nominal (delay_step=3 > size 0).
    EXPECT_NEAR((eskf.getStateAtDelay(3).p - eskf.getState().p).norm(), 0.0, 1e-10);
    // delay_step ≤ 0 → current nominal.
    EXPECT_NEAR((eskf.getStateAtDelay(0).p - eskf.getState().p).norm(), 0.0, 1e-10);
}

// ── findDelayStep + maxTrackedDelaySec ─────────────────────────────────────────
TEST(ESKF, DelayStepMapping_Deterministic)
{
    ESKFd eskf({}, 5);
    // Before any predict the accumulated time is still 1e15 → any delay maps to
    // the oldest slot.
    const std::size_t ds = eskf.findDelayStep(0.05);
    EXPECT_LT(ds, static_cast<std::size_t>(eskf.maxDelayStep()));
    EXPECT_GT(eskf.maxTrackedDelaySec(), 0.0);
}

// ── update<6> single-slot: nominal corrected, history mutated ──────────────────
TEST(ESKF, UpdateSingleSlot_NominalAndHistoryCorrected)
{
    ESKFd eskf({}, 5);
    // Warm up so the history is populated.
    for (int i = 0; i < 5; ++i) eskf.predict(0.01);

    // Feed a pose observation at delay_step=0 (current tick) that is 1 m away.
    Eigen::Matrix<double, 6, 1> innov; innov.setZero();
    innov(0) = 1.0;   // x-innovation: "observed position x = 1.0, nominal = 0"
    Eigen::Matrix<double, 6, 12> H; H.setZero();
    H.block<3, 3>(0, 0).setIdentity();
    H.block<3, 3>(3, 3).setIdentity();
    const Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity() * 0.05;

    ASSERT_TRUE(eskf.update<6>(innov, H, R, 0));

    // The nominal should have moved in x (not stay at zero).
    const double new_x = eskf.getState().p.x();
    EXPECT_GT(new_x, 0.0);    // was pulled toward the observation
}

// ── update<6> dual-slot ────────────────────────────────────────────────────────
TEST(ESKF, UpdateDualSlot_NominalCorrected)
{
    ESKFd eskf({}, 5);
    for (int i = 0; i < 5; ++i) eskf.predict(0.01);

    Eigen::Matrix<double, 6, 1> innov; innov.setZero();
    innov(0) = 0.5;
    Eigen::Matrix<double, 6, 12> H; H.setZero();
    H.block<3, 3>(0, 0).setIdentity();
    H.block<3, 3>(3, 3).setIdentity();
    const Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;

    ASSERT_TRUE(eskf.update<6>(innov, H, H, R, 0, 1));  // ds_to=0, ds_from=1
    // Nominal should have shifted.
    EXPECT_GT(eskf.getState().p.x(), 0.0);
}

// ── ESKF with non-positive dt (skipped) ────────────────────────────────────────
TEST(ESKF, Predict_NonPositiveDtSkipped)
{
    ESKFd eskf({}, 3);
    const auto before = eskf.getState().p;
    eskf.predict(-0.01);   // WARN, skip
    EXPECT_NEAR((eskf.getState().p - before).norm(), 0.0, 1e-10);
}
