// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "finenav_mppi_controller/optimizer.hpp"

namespace mppi
{
namespace
{

class TestMapView : public nav2_mppi_controller::IMapView
{
public:
  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return false;}
  bool isCollision(float, float, float) const override {return false;}
  float getRadius() const override {return 0.0f;}
  int getCost(const Position3D &) const override {return 0;}
  float costAtPose(float, float, float) const override {return 0.0f;}
  std::string getBaseFrameID() const override {return "base_link";}
};

class TrackingCritic : public critics::CriticFunction
{
public:
  TrackingCritic(int & initialize_count, int & score_count)
  : initialize_count_(initialize_count), score_count_(score_count) {}

  void initialize(const finenav_mppi_controller::Params &) override
  {
    ++initialize_count_;
  }

  void score(CriticData &, const nav2_mppi_controller::IMapView &) override
  {
    ++score_count_;
  }

private:
  int & initialize_count_;
  int & score_count_;
};

class TestOptimizer : public Optimizer
{
public:
  void setKnownControlSequence()
  {
    settings_.time_steps = 3;
    settings_.model_dt = 1.0f;
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
    state_.pose.pose.orientation.w = 1.0;
    control_sequence_.vx = {1.0f, 2.0f, 3.0f};
    control_sequence_.vy = {0.0f, 0.0f, 0.0f};
    control_sequence_.wz = {0.0f, 0.0f, 0.0f};
  }

  OptimizationResult finalize()
  {
    return finalizeControlSequence();
  }

  const models::ControlSequence & internalControlSequence() const
  {
    return control_sequence_;
  }

  void configureConstraints()
  {
    finenav_mppi_controller::Params params;
    params.batch_size = 1;
    params.time_steps = 3;
    params.regenerate_noises = false;
    params.vx_max = 0.5;
    params.vx_min = -0.2;
    params.vy_max = 0.3;
    params.wz_max = 0.8;
    getParams(params);
  }

  void setActiveConstraints(const models::ControlConstraints & constraints)
  {
    settings_.constraints = constraints;
  }

  models::ControlConstraints constraints() const
  {
    return settings_.constraints;
  }

  void resetOptimizer()
  {
    reset();
  }

  void scoreCritics(const nav2_mppi_controller::IMapView & map_view)
  {
    critic_manager_.evalTrajectoriesScores(critics_data_, map_view);
  }
};

TEST(OptimizerResultTest, ReturnsShiftedResultAndPreservesWarmStart)
{
  TestOptimizer optimizer;
  optimizer.setKnownControlSequence();

  const auto result = optimizer.finalize();

  EXPECT_FLOAT_EQ(result.controls(0, 0), 2.0f);
  EXPECT_FLOAT_EQ(result.controls(1, 0), 3.0f);
  EXPECT_FLOAT_EQ(result.controls(2, 0), 3.0f);
  EXPECT_FLOAT_EQ(result.trajectory(0, 0), 2.0f);
  EXPECT_FLOAT_EQ(result.trajectory(1, 0), 5.0f);
  EXPECT_FLOAT_EQ(result.trajectory(2, 0), 8.0f);

  const auto & warm_start = optimizer.internalControlSequence();
  EXPECT_FLOAT_EQ(warm_start.vx(0), 2.0f);
  EXPECT_FLOAT_EQ(warm_start.vx(1), 3.0f);
  EXPECT_FLOAT_EQ(warm_start.vx(2), 3.0f);
}

TEST(OptimizerResultTest, ResetPreservesActiveConstraints)
{
  TestOptimizer optimizer;
  optimizer.configureConstraints();
  optimizer.setActiveConstraints({1.2f, -0.7f, 0.6f, 1.5f});

  optimizer.resetOptimizer();

  const auto constraints = optimizer.constraints();
  EXPECT_FLOAT_EQ(constraints.vx_max, 1.2f);
  EXPECT_FLOAT_EQ(constraints.vx_min, -0.7f);
  EXPECT_FLOAT_EQ(constraints.vy, 0.6f);
  EXPECT_FLOAT_EQ(constraints.wz, 1.5f);
}

TEST(OptimizerResultTest, InitializeTransfersCriticsToManager)
{
  int initialize_count = 0;
  int score_count = 0;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics;
  critics.push_back(std::make_unique<TrackingCritic>(initialize_count, score_count));

  finenav_mppi_controller::Params params;
  params.batch_size = 1;
  params.time_steps = 3;
  params.regenerate_noises = false;

  TestOptimizer optimizer;
  optimizer.initialize("MPPIController", critics, params);

  EXPECT_TRUE(critics.empty());
  EXPECT_EQ(initialize_count, 1);

  TestMapView map_view;
  optimizer.scoreCritics(map_view);
  EXPECT_EQ(score_count, 1);
}

}  // namespace
}  // namespace mppi
