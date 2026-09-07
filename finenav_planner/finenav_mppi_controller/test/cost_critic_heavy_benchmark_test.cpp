// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "cost_critic_heavy_fixture.hpp"
#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/critics/cost_critic.hpp"

namespace mppi::benchmark_support
{
namespace
{

class CountingHeavyMapView final : public nav2_mppi_controller::IMapView
{
public:
  bool isTrackingUnknown() const override {return map_view_.isTrackingUnknown();}
  bool considerFootprint() const override {return map_view_.considerFootprint();}
  float getRadius() const override {return map_view_.getRadius();}
  std::string getBaseFrameID() const override {return map_view_.getBaseFrameID();}
  int getCost(const Position3D & position) const override {return map_view_.getCost(position);}

  float costAtPose(float x, float y, float theta) const override
  {
    return map_view_.costAtPose(x, y, theta);
  }

  bool isCollision(float x, float y, float theta) const override
  {
    ++collision_queries_;
    return map_view_.isCollision(x, y, theta);
  }

  std::size_t collisionQueries() const {return collision_queries_;}

private:
  HeavyObstacleMapView map_view_;
  mutable std::size_t collision_queries_{0U};
};

finenav_mppi_controller::Params makeParams()
{
  finenav_mppi_controller::Params params;
  params.CostCritic.enabled = true;
  params.CostCritic.cost_power = 1;
  params.CostCritic.cost_weight = 3.81;
  params.CostCritic.critical_cost = 300.0;
  params.CostCritic.near_goal_distance = 1.0;
  return params;
}

TEST(CostCriticHeavyBenchmarkTest, OBBEdgeQuerySeesObstacleBeforeCenterPoint)
{
  HeavyObstacleMapView map_view;

  EXPECT_EQ(map_view.getCost(Position3D{0.0, 0.0, 0.0}), 200);
  EXPECT_FALSE(map_view.isCollision(0.0F, 0.0F, 0.0F));
  EXPECT_TRUE(map_view.isCollision(0.35F, 0.0F, 0.0F));
}

TEST(CostCriticHeavyBenchmarkTest, HeavyAndEarlyCollisionPathsAreContiguous)
{
  models::Trajectories trajectories;
  trajectories.reset(40U, 1U);

  populateHeavyObstacleTrajectories(trajectories);

  EXPECT_FLOAT_EQ(trajectories.y(0, 0), -2.5F);
  EXPECT_FLOAT_EQ(trajectories.y(15, 0), -2.5F);
  EXPECT_FLOAT_EQ(trajectories.y(16, 0), 0.0F);
  EXPECT_FLOAT_EQ(trajectories.y(23, 0), 0.0F);
  EXPECT_FLOAT_EQ(trajectories.y(24, 0), 1.0F);
  EXPECT_FLOAT_EQ(trajectories.y(31, 0), 1.0F);
  EXPECT_FLOAT_EQ(trajectories.y(32, 0), 2.0F);
  EXPECT_FLOAT_EQ(trajectories.y(37, 0), 2.0F);
  EXPECT_FLOAT_EQ(trajectories.y(38, 0), 3.0F);
}

TEST(CostCriticHeavyBenchmarkTest, MixedObstaclePathsKeepSomeCandidatesFeasible)
{
  constexpr unsigned int kBatchSize = 20U;
  constexpr unsigned int kTimeSteps = 30U;
  models::State state;
  models::Trajectories trajectories;
  models::Path path;
  models::Vector costs = models::Vector::Zero(kBatchSize);
  float model_dt = 0.05F;
  CriticData data{state, trajectories, path, costs, model_dt, false,
    std::make_shared<DiffDriveMotionModel>(), std::nullopt, std::nullopt};

  state.reset(kBatchSize, kTimeSteps);
  trajectories.reset(kBatchSize, kTimeSteps);
  path.reset(2);
  state.pose.pose.orientation.w = 1.0;
  path.x << 0.0F, 10.0F;
  populateHeavyObstacleTrajectories(trajectories);

  HeavyObstacleMapView map_view;
  critics::CostCritic critic({1U, 1U});
  critic.initialize(makeParams());
  critic.score(data, map_view);

  EXPECT_FALSE(data.fail_flag);
  EXPECT_GT(costs.maxCoeff(), 1.0e30F);
  EXPECT_LT(costs.minCoeff(), 100.0F);
}

std::size_t countCollisionQueriesForLateral(float lateral)
{
  constexpr unsigned int kTimeSteps = 30U;
  models::State state;
  models::Trajectories trajectories;
  models::Path path;
  models::Vector costs = models::Vector::Zero(1U);
  float model_dt = 0.05F;
  CriticData data{state, trajectories, path, costs, model_dt, false,
    std::make_shared<DiffDriveMotionModel>(), std::nullopt, std::nullopt};

  state.reset(1U, kTimeSteps);
  trajectories.reset(1U, kTimeSteps);
  path.reset(2);
  state.pose.pose.orientation.w = 1.0;
  path.x << 0.0F, 10.0F;
  for (Eigen::Index step = 0; step < trajectories.x.cols(); ++step) {
    trajectories.x(0, step) = 0.06F * static_cast<float>(step);
    trajectories.y(0, step) = lateral;
    trajectories.yaws(0, step) = 0.0F;
  }

  CountingHeavyMapView map_view;
  critics::CostCritic critic;
  critic.initialize(makeParams());
  critic.score(data, map_view);
  return map_view.collisionQueries();
}

TEST(CostCriticHeavyBenchmarkTest, CollisionGroupsHaveIncreasingWorkBeforeExit)
{
  constexpr std::size_t kTimeSteps = 30U;
  const std::size_t full_scan = countCollisionQueriesForLateral(-2.5F);
  const std::size_t early_collision = countCollisionQueriesForLateral(0.0F);
  const std::size_t middle_collision = countCollisionQueriesForLateral(1.0F);
  const std::size_t late_collision = countCollisionQueriesForLateral(2.0F);
  const std::size_t free_path = countCollisionQueriesForLateral(3.0F);

  EXPECT_EQ(full_scan, kTimeSteps);
  EXPECT_LT(early_collision, middle_collision);
  EXPECT_LT(middle_collision, late_collision);
  EXPECT_LT(late_collision, kTimeSteps);
  EXPECT_EQ(free_path, kTimeSteps);
}

}  // namespace
}  // namespace mppi::benchmark_support
