// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"
#include "cost_critic_heavy_fixture.hpp"

namespace
{

class FreeSpaceMap final : public nav2_mppi_controller::IMapView
{
public:
  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return false;}
  bool isCollision(float, float, float) const override {return false;}
  float getRadius() const override {return 0.2F;}
  int getCost(const Position3D &) const override {return 0;}
  float costAtPose(float, float, float) const override {return 0.0F;}
  std::string getBaseFrameID() const override {return "base_link";}
};

finenav_mppi_controller::Params makeParams()
{
  finenav_mppi_controller::Params params;
  params.batch_size = 3000;
  params.time_steps = 30;
  params.iteration_count = 1;
  params.model_dt = 0.05;
  params.temperature = 0.3;
  params.gamma = 0.015;
  params.vx_max = 0.5;
  params.vx_min = -0.2;
  params.vy_max = 0.0;
  params.wz_max = 1.0;
  params.ax_min = -1.0;
  params.ax_max = 1.0;
  params.ay_min = -1.0;
  params.ay_max = 1.0;
  params.az_max = 2.0;
  params.vx_std = 0.2;
  params.vy_std = 0.1;
  params.wz_std = 0.3;
  params.motion_model = "DiffDrive";
  params.regenerate_noises = false;
  params.visualize = true;
  params.CostCritic.num_threads = 8;
  params.CostCritic.chunk_size = 1;
  params.PathAlignCritic.offset_from_furthest = 1;
  params.PathAlignCritic.trajectory_point_step = 1;
  params.PathFollowCritic.offset_from_furthest = 1;
  params.PathAngleCritic.offset_from_furthest = 1;
  params.TwirlingCritic.enabled = false;
  params.VelocityDeadbandCritic.enabled = false;
  return params;
}

nav_msgs::msg::Path makePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int index = 0; index < 40; ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(index) * 0.1;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

TEST(OptimizerVisualizedResetTest, ReusesLargeDiagnosticBuffersAcrossEpisodes)
{
  const auto params = makeParams();
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  critics.push_back(std::make_unique<mppi::critics::ConstraintCritic>());
  critics.push_back(std::make_unique<mppi::critics::CostCritic>(
    mppi::critics::CostCriticExecutionOptions{8, 1}));
  critics.push_back(std::make_unique<mppi::critics::GoalAngleCritic>());
  critics.push_back(std::make_unique<mppi::critics::GoalCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathAlignCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathAngleCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathFollowCritic>());
  critics.push_back(std::make_unique<mppi::critics::PreferForwardCritic>());
  critics.push_back(std::make_unique<mppi::critics::TwirlingCritic>());
  critics.push_back(std::make_unique<mppi::critics::VelocityDeadbandCritic>());

  mppi::Optimizer optimizer;
  optimizer.initialize("test", critics, params);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist speed;
  const auto path = makePath();
  FreeSpaceMap map;

  for (int episode = 0; episode < 10; ++episode) {
    for (int cycle = 0; cycle < 3; ++cycle) {
      const auto result = optimizer.evalControl(pose, speed, path, map);
      EXPECT_EQ(result.controls.rows(), params.time_steps);
      EXPECT_EQ(result.trajectory.rows(), params.time_steps);
    }
    optimizer.reset();
  }
}

TEST(OptimizerVisualizedResetTest, RepeatedHeavyObstacleEpisodesReleaseDiagnostics)
{
  const auto params = makeParams();
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  critics.push_back(std::make_unique<mppi::critics::ConstraintCritic>());
  critics.push_back(std::make_unique<mppi::critics::CostCritic>(
    mppi::critics::CostCriticExecutionOptions{8, 1}));
  critics.push_back(std::make_unique<mppi::critics::GoalAngleCritic>());
  critics.push_back(std::make_unique<mppi::critics::GoalCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathAlignCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathAngleCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathFollowCritic>());
  critics.push_back(std::make_unique<mppi::critics::PreferForwardCritic>());
  critics.push_back(std::make_unique<mppi::critics::TwirlingCritic>());
  critics.push_back(std::make_unique<mppi::critics::VelocityDeadbandCritic>());

  mppi::Optimizer optimizer;
  optimizer.initialize("heavy-obstacle-reset", critics, params);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist speed;
  const auto path = makePath();
  mppi::benchmark_support::HeavyObstacleMapView map;

  for (int episode = 0; episode < 8; ++episode) {
    const auto result = optimizer.evalControl(pose, speed, path, map);
    EXPECT_EQ(result.controls.rows(), params.time_steps);
    EXPECT_EQ(result.trajectory.rows(), params.time_steps);
    ASSERT_TRUE(optimizer.getCriticCostBreakdown().valid);
    optimizer.reset();
  }
}

}  // namespace
