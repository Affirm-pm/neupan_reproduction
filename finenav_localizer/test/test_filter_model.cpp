// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include "finenav_localizer/filter_model.hpp"

namespace fn = finenav::localizer;
using Model = fn::NavigationModelD;

// ============================================================================
//  NavigationModel (pure math — no ROS required)
// ============================================================================

TEST(NavigationModel, PredictNominal_ZeroVelocity_StaysAtOrigin)
{
    fn::NavStateD s;  // default: p=0, R=I, v=0, ω=0
    const auto ns = Model::predictNominal(s, 1.0);
    EXPECT_NEAR(ns.p.norm(), 0.0, 1e-12);
    EXPECT_NEAR((ns.R.matrix() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
}

TEST(NavigationModel, PredictNominal_ConstantVelocity_PositionIntegrates)
{
    fn::NavStateD s;
    s.v = {1.0, 2.0, 0.0};
    const auto ns = Model::predictNominal(s, 3.0);
    EXPECT_NEAR(ns.p.x(), 3.0, 1e-12);
    EXPECT_NEAR(ns.p.y(), 6.0, 1e-12);
    EXPECT_NEAR(ns.p.z(), 0.0, 1e-12);
}

TEST(NavigationModel, ObservePose_ZeroInnovationAtNominal)
{
    fn::NavStateD s;
    s.p = {1.0, -2.0, 0.5};
    s.R = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, 1.2});

    const auto obs = Model::observePose(s, s.p, s.R);
    EXPECT_NEAR(obs.innovation.norm(), 0.0, 1e-10);
}

TEST(NavigationModel, ObserveVelocity_ZeroInnovationAtNominal)
{
    fn::NavStateD s;
    s.v     = {1.0, 0.5, 0.0};
    s.omega = {0.0, 0.0, 0.3};

    // body-frame velocity matching the nominal
    const Eigen::Vector3d v_body = s.R.inverse() * s.v;
    const auto obs = Model::observeVelocity(s, v_body, s.omega);
    EXPECT_NEAR(obs.innovation.norm(), 0.0, 1e-9);
}
