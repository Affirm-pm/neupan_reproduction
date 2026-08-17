// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/critics/constraint_critic.hpp"
#include "finenav_mppi_controller/critics/cost_critic.hpp"
#include "finenav_mppi_controller/critics/goal_angle_critic.hpp"
#include "finenav_mppi_controller/critics/goal_critic.hpp"
#include "finenav_mppi_controller/critics/path_align_critic.hpp"
#include "finenav_mppi_controller/critics/path_angle_critic.hpp"
#include "finenav_mppi_controller/critics/path_follow_critic.hpp"
#include "finenav_mppi_controller/critics/prefer_forward_critic.hpp"
#include "finenav_mppi_controller/critics/twirling_critic.hpp"
#include "finenav_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics {
namespace {

class TestMapView final : public nav2_mppi_controller::IMapView {
  public:
    bool isTrackingUnknown() const override { return false; }

    bool considerFootprint() const override { return false; }

    bool isCollision(float x, float, float) const override { return collide_when_x_negative ? x < 0.0f : collision; }

    float getRadius() const override { return 0.2f; }

    int getCost(const Position3D&) const override {
        ++cost_queries;
        return cost;
    }

    float costAtPose(float, float, float) const override { return static_cast<float>(cost); }

    std::string getBaseFrameID() const override { return "base_link"; }

    int cost{0};
    bool collision{false};
    bool collide_when_x_negative{false};
    mutable std::size_t cost_queries{0};
};

struct CriticTestData {
    models::State state;
    models::Trajectories trajectories;
    models::Path path;
    xt::xtensor<float, 1> costs{xt::zeros<float>({2})};
    float model_dt{0.1f};
    CriticData data{state,        trajectories, path, costs, model_dt, false, std::make_shared<DiffDriveMotionModel>(),
                    std::nullopt, std::nullopt};

    CriticTestData() {
        state.reset(2, 4);
        trajectories.reset(2, 4);
        path.reset(6);
        state.pose.pose.orientation.w = 1.0;
        for (std::size_t i = 0; i < path.x.size(); ++i) {
            path.x(i) = static_cast<float>(i) * 0.2f;
        }
    }

    void setAllTrajectoryY(float value) { trajectories.y.fill(value); }

    void setTrajectoryX(float start, float step) {
        for (std::size_t batch = 0; batch < trajectories.x.shape(0); ++batch) {
            for (std::size_t time = 0; time < trajectories.x.shape(1); ++time) {
                trajectories.x(batch, time) = start + step * static_cast<float>(time);
            }
        }
    }

    void setAllPathPointsValid() { data.path_pts_valid = std::vector<bool>(path.x.size() - 1, true); }
};

void expectPositiveCosts(const CriticTestData& fixture) {
    EXPECT_GT(fixture.costs(0), 0.0f);
    EXPECT_GT(fixture.costs(1), 0.0f);
}

TEST(CriticScoreTest, DisabledCriticLeavesCostsUnchanged) {
    finenav_mppi_controller::Params params;
    params.TwirlingCritic.enabled = false;
    CriticTestData fixture;
    fixture.state.wz.fill(0.4f);
    TestMapView map_view;
    TwirlingCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    EXPECT_EQ(fixture.costs(0), 0.0f);
    EXPECT_EQ(fixture.costs(1), 0.0f);
}

TEST(CriticScoreTest, ConstraintCriticPenalizesVelocityAboveLimit) {
    finenav_mppi_controller::Params params;
    params.vx_max = 0.5;
    params.vy_max = 0.0;
    CriticTestData fixture;
    fixture.state.vx.fill(1.0f);
    TestMapView map_view;
    ConstraintCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
}

TEST(CriticScoreTest, ConstraintCriticPenalizesAckermannTurningRadius) {
    finenav_mppi_controller::Params params;
    CriticTestData fixture;
    fixture.data.motion_model = std::make_shared<AckermannMotionModel>();
    fixture.state.vx.fill(0.1f);
    fixture.state.wz.fill(1.0f);
    TestMapView map_view;
    ConstraintCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
}

TEST(CriticScoreTest, PreferForwardCriticPenalizesReverseMotionNearGoal) {
    finenav_mppi_controller::Params params;
    CriticTestData fixture;
    fixture.state.pose.pose.position.x = 0.9;
    fixture.state.vx.fill(-0.2f);
    TestMapView map_view;
    PreferForwardCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
}

TEST(CriticScoreTest, TwirlingCriticPenalizesAngularMotion) {
    finenav_mppi_controller::Params params;
    params.TwirlingCritic.enabled = true;
    CriticTestData fixture;
    fixture.state.wz.fill(0.4f);
    TestMapView map_view;
    TwirlingCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
}

TEST(CriticScoreTest, VelocityDeadbandCriticScoresDiffDriveAndOmniAtHigherPower) {
    finenav_mppi_controller::Params params;
    params.VelocityDeadbandCritic.enabled = true;
    params.VelocityDeadbandCritic.cost_power = 2;
    params.VelocityDeadbandCritic.deadband_velocities = {0.2, 0.2, 0.2};
    TestMapView map_view;
    VelocityDeadbandCritic critic;
    critic.initialize(params);

    CriticTestData differential_drive;
    differential_drive.state.vx.fill(0.05f);
    differential_drive.state.vy.fill(0.05f);
    differential_drive.state.wz.fill(0.05f);
    critic.score(differential_drive.data, map_view);

    CriticTestData omni_drive;
    omni_drive.data.motion_model = std::make_shared<OmniMotionModel>();
    omni_drive.state.vx.fill(0.05f);
    omni_drive.state.vy.fill(0.05f);
    omni_drive.state.wz.fill(0.05f);
    critic.score(omni_drive.data, map_view);

    expectPositiveCosts(differential_drive);
    expectPositiveCosts(omni_drive);
    EXPECT_GT(omni_drive.costs(0), differential_drive.costs(0));
    EXPECT_GT(omni_drive.costs(1), differential_drive.costs(1));
}

TEST(CriticScoreTest, VelocityDeadbandCriticUsesLinearCostAtPowerOne) {
    finenav_mppi_controller::Params params;
    params.VelocityDeadbandCritic.enabled = true;
    params.VelocityDeadbandCritic.cost_power = 1;
    params.VelocityDeadbandCritic.deadband_velocities = {0.2, 0.2, 0.2};
    CriticTestData fixture;
    fixture.state.vx.fill(0.05f);
    fixture.state.wz.fill(0.05f);
    TestMapView map_view;
    VelocityDeadbandCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
}

TEST(CriticScoreTest, GoalCriticsScoreNearGoalAndSkipFarGoal) {
    finenav_mppi_controller::Params params;
    TestMapView map_view;
    GoalCritic goal_critic;
    GoalAngleCritic goal_angle_critic;
    goal_critic.initialize(params);
    goal_angle_critic.initialize(params);

    CriticTestData near_goal;
    near_goal.state.pose.pose.position.x = 0.9;
    near_goal.trajectories.yaws.fill(static_cast<float>(M_PI_2));
    goal_critic.score(near_goal.data, map_view);
    goal_angle_critic.score(near_goal.data, map_view);

    expectPositiveCosts(near_goal);

    CriticTestData far_goal;
    far_goal.state.pose.pose.position.x = -2.0;
    far_goal.trajectories.yaws.fill(static_cast<float>(M_PI_2));
    goal_critic.score(far_goal.data, map_view);
    goal_angle_critic.score(far_goal.data, map_view);

    EXPECT_EQ(far_goal.costs(0), 0.0f);
    EXPECT_EQ(far_goal.costs(1), 0.0f);
}

TEST(CriticScoreTest, PathFollowCriticScoresEndpointDistance) {
    finenav_mppi_controller::Params params;
    params.PathFollowCritic.offset_from_furthest = 1;
    CriticTestData fixture;
    fixture.state.pose.pose.position.x = -1.0;
    fixture.setAllPathPointsValid();
    fixture.data.furthest_reached_path_point = 1;
    fixture.setAllTrajectoryY(0.3f);
    TestMapView map_view;
    PathFollowCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
}

TEST(CriticScoreTest, PathAngleCriticScoresHeadingError) {
    finenav_mppi_controller::Params params;
    params.vx_min = 0.0;
    params.PathAngleCritic.offset_from_furthest = 1;
    params.PathAngleCritic.max_angle_to_furthest = 0.1;
    CriticTestData fixture;
    fixture.path.y(2) = 1.0f;
    fixture.data.furthest_reached_path_point = 1;
    TestMapView map_view;
    PathAngleCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
}

TEST(CriticScoreTest, PathAlignCriticScoresLateralDeviation) {
    finenav_mppi_controller::Params params;
    params.PathAlignCritic.offset_from_furthest = 1;
    params.PathAlignCritic.trajectory_point_step = 1;
    CriticTestData fixture;
    fixture.state.pose.pose.position.x = -1.0;
    fixture.data.furthest_reached_path_point = 5;
    fixture.setAllPathPointsValid();
    fixture.setAllTrajectoryY(0.2f);
    fixture.setTrajectoryX(0.0f, 0.05f);
    TestMapView map_view;
    PathAlignCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
}

TEST(CriticScoreTest, CostCriticLeavesFreeSpaceCostsAtZero) {
    finenav_mppi_controller::Params params;
    CriticTestData fixture;
    TestMapView map_view;
    CostCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    EXPECT_EQ(fixture.costs(0), 0.0f);
    EXPECT_EQ(fixture.costs(1), 0.0f);
    EXPECT_FALSE(fixture.data.fail_flag);
    EXPECT_GT(map_view.cost_queries, 0u);
}

TEST(CriticScoreTest, CostCriticMarksFailureWhenAllTrajectoriesCollide) {
    finenav_mppi_controller::Params params;
    CriticTestData fixture;
    TestMapView map_view;
    map_view.cost = 254;
    map_view.collision = true;
    CostCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    expectPositiveCosts(fixture);
    EXPECT_TRUE(fixture.data.fail_flag);
}

TEST(CriticScoreTest, CostCriticKeepsPlanningWhenOneTrajectoryIsFree) {
    finenav_mppi_controller::Params params;
    CriticTestData fixture;
    fixture.trajectories.x(0, 0) = -0.1f;
    fixture.trajectories.x(0, 1) = -0.1f;
    fixture.trajectories.x(0, 2) = -0.1f;
    fixture.trajectories.x(0, 3) = -0.1f;
    fixture.trajectories.x(1, 0) = 1.0f;
    fixture.trajectories.x(1, 1) = 1.0f;
    fixture.trajectories.x(1, 2) = 1.0f;
    fixture.trajectories.x(1, 3) = 1.0f;
    TestMapView map_view;
    map_view.cost = 254;
    map_view.collide_when_x_negative = true;
    CostCritic critic;
    critic.initialize(params);

    critic.score(fixture.data, map_view);

    EXPECT_FALSE(fixture.data.fail_flag);
    EXPECT_GT(fixture.costs(0), fixture.costs(1));
    EXPECT_GT(fixture.costs(1), 0.0f);
}

}  // namespace
}  // namespace mppi::critics
