// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "finenav_localizer/kalman_filter/kf_math.hpp"

using namespace finenav::localizer::kf;

// ── update: zero innovation (measurement matches prediction) ─────────────────
TEST(KfMathUpdate, ZeroInnovation_StateUnchanged)
{
    Eigen::MatrixXd x(3, 1); x << 1, 2, 3;
    Eigen::MatrixXd P  = Eigen::MatrixXd::Identity(3, 3) * 2.0;
    Eigen::MatrixXd y(3, 1); y << 1, 2, 3;
    Eigen::MatrixXd y_pred = y;
    const Eigen::MatrixXd C      = Eigen::MatrixXd::Identity(3, 3);
    const Eigen::MatrixXd R      = Eigen::MatrixXd::Identity(3, 3) * 0.1;

    const Eigen::MatrixXd x_before = x;
    const Eigen::MatrixXd P_before = P;
    EXPECT_TRUE(update(x, P, y, y_pred, C, R));
    EXPECT_LT((x - x_before).norm(), 1e-12);           // x unchanged
    EXPECT_LT((P - P.transpose()).norm(), 1e-12);      // symmetric
}

// ── update: identity matrices → analytic expected values ─────────────────────
TEST(KfMathUpdate, IdentityMatrices_ProducesKnownGain)
{
    Eigen::MatrixXd x(2, 1); x << 0, 0;
    Eigen::MatrixXd P = Eigen::Matrix2d::Identity() * 2.0;
    Eigen::MatrixXd y(2, 1); y << 1, 0;
    Eigen::MatrixXd y_pred(2, 1); y_pred << 0, 0;
    const Eigen::MatrixXd C = Eigen::Matrix2d::Identity();
    const Eigen::MatrixXd R = Eigen::Matrix2d::Identity() * 0.5;

    const bool ok = update(x, P, y, y_pred, C, R);
    EXPECT_TRUE(ok);

    // K = P·Cᵀ(R + C·P·Cᵀ)⁻¹ = 2I · (0.5I+2I·I·I)⁻¹ · I = 2I · (2.5I)⁻¹ = 0.8 I
    // x = [0,0] + 0.8I * [1,0] = [0.8, 0]
    EXPECT_NEAR(x(0), 0.8, 1e-10);
    EXPECT_NEAR(x(1), 0.0, 1e-10);
    // P = (I−KC)·P = (I − 0.8I)·2I = 0.4 I
    EXPECT_NEAR(P(0, 0), 0.4, 1e-10);
    EXPECT_NEAR(P(1, 1), 0.4, 1e-10);
    EXPECT_LT((P - P.transpose()).norm(), 1e-12);    // forced symmetry
}

// ── update: NaN/Inf guard (K becomes NaN → should return false, x,P unchanged)─
TEST(KfMathUpdate, NaNGuard_ReturnsFalseAndStateUnchanged)
{
    Eigen::MatrixXd x(2, 1); x << 0, 0;
    Eigen::MatrixXd P  = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd y(2, 1); y << 0, 0;
    const Eigen::MatrixXd y_pred = y;
    const Eigen::MatrixXd C = Eigen::Matrix2d::Zero();   // causes (R+0)⁻¹ fine,
    const Eigen::MatrixXd R = Eigen::Matrix2d::Zero();   // but K formula degenerates
    // With R=0, C=0 → PCT=0 → K = 0·0⁻¹ = NaN (0/0).

    const Eigen::MatrixXd x_before = x;
    const Eigen::MatrixXd P_before = P;
    EXPECT_FALSE(update(x, P, y, y_pred, C, R));
    EXPECT_TRUE(x.isApprox(x_before));
    EXPECT_TRUE(P.isApprox(P_before));
}

// ── update: dimension mismatch → returns false ────────────────────────────────
TEST(KfMathUpdate, DimensionMismatch_ReturnsFalse)
{
    Eigen::MatrixXd x(3, 1); x.setZero();
    Eigen::MatrixXd P = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd y(2, 1); y.setZero();
    Eigen::MatrixXd y_p(2, 1); y_p.setZero();
    Eigen::MatrixXd C(2, 3); C.setZero();
    Eigen::MatrixXd R(3, 3); R.setIdentity();   // rows != y.rows
    EXPECT_FALSE(update(x, P, y, y_p, C, R));
}

// ── predict: standard form ─────────────────────────────────────────────────────
TEST(KfMathPredict, StandardForm_MatchesManual)
{
    Eigen::MatrixXd x(2, 1); x << 1, 2;
    Eigen::MatrixXd P  = Eigen::Matrix2d::Identity() * 4.0;
    Eigen::MatrixXd x_next(2, 1); x_next << 2, 3;
    const Eigen::MatrixXd A = Eigen::Matrix2d::Identity() * 2.0;
    const Eigen::MatrixXd Q = Eigen::Matrix2d::Identity() * 0.1;

    EXPECT_TRUE(predict(x, P, x_next, A, Q));
    EXPECT_TRUE(x.isApprox(x_next));
    // P = A·P·Aᵀ + Q = 2I · 4I · 2I + 0.1I = 16I + 0.1I = 16.1 I
    EXPECT_NEAR(P(0, 0), 16.1, 1e-10);
    EXPECT_NEAR(P(1, 1), 16.1, 1e-10);
}

TEST(KfMathPredict, DimensionMismatch_ReturnsFalse)
{
    Eigen::MatrixXd x(2, 1); x.setZero();
    Eigen::MatrixXd P = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd x_next(3, 1); x_next.setZero();  // wrong size
    Eigen::MatrixXd A = Eigen::Matrix2d::Identity();
    Eigen::MatrixXd Q = Eigen::Matrix2d::Identity();
    EXPECT_FALSE(predict(x, P, x_next, A, Q));
}
