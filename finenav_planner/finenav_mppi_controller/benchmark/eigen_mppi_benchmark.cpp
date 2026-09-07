// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <benchmark/benchmark.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"
#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/tools/noise_generator.hpp"

namespace
{

// Match the historical mppi/01_open baseline workload (3k trajectories x 30 steps).
constexpr unsigned int kBatchSize = 3000;
constexpr unsigned int kTimeSteps = 30;
constexpr unsigned int kIterationCount = 1;
constexpr unsigned int kPathPoints = 50;

class EmptyMapView final : public nav2_mppi_controller::IMapView
{
public:
  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return false;}
  bool isCollision(float, float, float) const override {return false;}
  float getRadius() const override {return 0.0F;}
  int getCost(const Position3D &) const override {return 0;}
  float costAtPose(float, float, float) const override {return 0.0F;}
  std::string getBaseFrameID() const override {return "base_link";}
};

class BenchmarkOptimizer final : public mppi::Optimizer
{
public:
  void integrate(
    mppi::models::BatchArray & output,
    const mppi::models::BatchArray & sequence) const
  {
    integrateStateVelocities(output, sequence);
  }

  void updateControls()
  {
    updateControlSequence();
  }
};

finenav_mppi_controller::Params makeParams()
{
  finenav_mppi_controller::Params params;
  params.time_steps = kTimeSteps;
  params.model_dt = 0.05;
  params.batch_size = kBatchSize;
  params.iteration_count = kIterationCount;
  params.temperature = 0.3;
  params.gamma = 0.015;
  params.vx_max = 0.5;
  params.vx_min = -0.35;
  params.vy_max = 0.0;
  params.wz_max = 1.9;
  params.vx_std = 0.2;
  params.vy_std = 0.2;
  params.wz_std = 0.4;
  params.regenerate_noises = false;
  params.motion_model = "DiffDrive";
  return params;
}

geometry_msgs::msg::PoseStamped makePose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Twist makeSpeed()
{
  geometry_msgs::msg::Twist speed;
  speed.linear.x = 0.1;
  return speed;
}

nav_msgs::msg::Path makePath()
{
  nav_msgs::msg::Path path;
  path.poses.resize(kPathPoints);
  for (unsigned int i = 0; i < kPathPoints; ++i) {
    path.poses[i].pose.position.x = 0.05 * static_cast<double>(i);
    path.poses[i].pose.orientation.w = 1.0;
  }
  return path;
}

void initializeOptimizer(BenchmarkOptimizer & optimizer)
{
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  critics.push_back(std::make_unique<mppi::critics::ConstraintCritic>());
  critics.push_back(std::make_unique<mppi::critics::CostCritic>());
  critics.push_back(std::make_unique<mppi::critics::GoalAngleCritic>());
  critics.push_back(std::make_unique<mppi::critics::GoalCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathAlignCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathAngleCritic>());
  critics.push_back(std::make_unique<mppi::critics::PathFollowCritic>());
  critics.push_back(std::make_unique<mppi::critics::PreferForwardCritic>());
  critics.push_back(std::make_unique<mppi::critics::TwirlingCritic>());
  critics.push_back(std::make_unique<mppi::critics::VelocityDeadbandCritic>());
  optimizer.initialize("EigenBenchmark", critics, makeParams());
}

mppi::models::OptimizerSettings makeCoreSettings()
{
  const auto params = makeParams();
  mppi::models::OptimizerSettings settings;
  settings.batch_size = kBatchSize;
  settings.time_steps = kTimeSteps;
  settings.model_dt = static_cast<float>(params.model_dt);
  settings.sampling_std.vx = static_cast<float>(params.vx_std);
  settings.sampling_std.vy = static_cast<float>(params.vy_std);
  settings.sampling_std.wz = static_cast<float>(params.wz_std);
  return settings;
}

double Percentile95(const std::vector<double> & samples)
{
  if (samples.empty()) {
    return 0.0;
  }
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const auto index = static_cast<size_t>(std::ceil(0.95 * sorted.size())) - 1;
  return sorted[index];
}

void BM_OptimizerEvalControl(benchmark::State & state)
{
  BenchmarkOptimizer optimizer;
  initializeOptimizer(optimizer);
  const auto pose = makePose();
  const auto speed = makeSpeed();
  const auto path = makePath();
  EmptyMapView map_view;
  benchmark::DoNotOptimize(optimizer.evalControl(pose, speed, path, map_view));

  for (auto _ : state) {
    auto result = optimizer.evalControl(pose, speed, path, map_view);
    benchmark::DoNotOptimize(result.controls);
    benchmark::DoNotOptimize(result.trajectory);
  }
}

void BM_StateReset(benchmark::State & state)
{
  mppi::models::State model;
  for (auto _ : state) {
    model.reset(kBatchSize, kTimeSteps);
    benchmark::DoNotOptimize(model.vx);
  }
}

void BM_TrajectoryReset(benchmark::State & state)
{
  mppi::models::Trajectories trajectories;
  for (auto _ : state) {
    trajectories.reset(kBatchSize, kTimeSteps);
    benchmark::DoNotOptimize(trajectories.x);
  }
}

void BM_NoiseAdd(benchmark::State & state)
{
  auto settings = makeCoreSettings();
  mppi::NoiseGenerator generator;
  generator.initialize(settings, false, false);
  mppi::models::State model;
  model.reset(kBatchSize, kTimeSteps);
  mppi::models::ControlSequence controls;
  controls.reset(kTimeSteps);

  for (auto _ : state) {
    generator.setNoisedControls(model, controls);
    benchmark::DoNotOptimize(model.cvx);
  }
  generator.shutdown();
}

void BM_MotionModelPredict(benchmark::State & state)
{
  mppi::models::State model;
  model.reset(kBatchSize, kTimeSteps);
  model.cvx.setConstant(0.2F);
  model.cwz.setConstant(0.1F);
  mppi::DiffDriveMotionModel motion_model;

  for (auto _ : state) {
    motion_model.predict(model);
    benchmark::DoNotOptimize(model.vx);
  }
}

void BM_TrajectoryIntegration(benchmark::State & state)
{
  BenchmarkOptimizer optimizer;
  initializeOptimizer(optimizer);
  const auto pose = makePose();
  const auto speed = makeSpeed();
  const auto path = makePath();
  EmptyMapView map_view;
  const auto initial = optimizer.evalControl(pose, speed, path, map_view);
  const auto sequence = initial.controls;
  mppi::models::BatchArray output(kTimeSteps, 3);

  for (auto _ : state) {
    optimizer.integrate(output, sequence);
    benchmark::DoNotOptimize(output);
  }
}

void BM_ControlUpdate(benchmark::State & state)
{
  BenchmarkOptimizer optimizer;
  initializeOptimizer(optimizer);
  const auto pose = makePose();
  const auto speed = makeSpeed();
  const auto path = makePath();
  EmptyMapView map_view;
  benchmark::DoNotOptimize(optimizer.evalControl(pose, speed, path, map_view));

  for (auto _ : state) {
    optimizer.updateControls();
    const auto controls = optimizer.getControlSequence();
    benchmark::DoNotOptimize(controls);
  }
}

}  // namespace

BENCHMARK(BM_OptimizerEvalControl)->UseRealTime()->ComputeStatistics("p95", Percentile95);
BENCHMARK(BM_StateReset)->UseRealTime()->ComputeStatistics("p95", Percentile95);
BENCHMARK(BM_TrajectoryReset)->UseRealTime()->ComputeStatistics("p95", Percentile95);
BENCHMARK(BM_NoiseAdd)->UseRealTime()->ComputeStatistics("p95", Percentile95);
BENCHMARK(BM_MotionModelPredict)->UseRealTime()->ComputeStatistics("p95", Percentile95);
BENCHMARK(BM_TrajectoryIntegration)->UseRealTime()->ComputeStatistics("p95", Percentile95);
BENCHMARK(BM_ControlUpdate)->UseRealTime()->ComputeStatistics("p95", Percentile95);

BENCHMARK_MAIN();
