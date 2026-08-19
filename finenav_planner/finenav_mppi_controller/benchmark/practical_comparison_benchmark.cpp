// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.

#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include "finenav_mppi_controller/optimizer.hpp"

namespace
{

constexpr unsigned int kPracticalBatchSize = 5000;
constexpr unsigned int kCompatibilityBatchSize = 3000;
constexpr unsigned int kPracticalTimeSteps = 56;
constexpr unsigned int kPracticalIterations = 2;
constexpr unsigned int kCompatibilityIterations = 1;
constexpr unsigned int kPathPoints = 100;

enum class Scenario {Open, Inflated, Mixed};

class ScenarioMapView final : public nav2_mppi_controller::IMapView
{
public:
  explicit ScenarioMapView(Scenario scenario)
  : scenario_(scenario) {}

  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return false;}
  float getRadius() const override {return 0.25F;}
  std::string getBaseFrameID() const override {return "base_link";}

  int getCost(const Position3D & position) const override
  {
    if (scenario_ == Scenario::Open) {
      return 0;
    }

    const float x = static_cast<float>(position.x());
    const float y = static_cast<float>(position.y());
    const float variation = std::abs(std::sin(1.7F * x)) * 90.0F;
    const float lateral = std::min(std::abs(y) * 180.0F, 120.0F);
    return static_cast<int>(std::clamp(20.0F + variation + lateral, 1.0F, 220.0F));
  }

  float costAtPose(float x, float y, float) const override
  {
    return static_cast<float>(getCost(
             Position3D{static_cast<double>(x),
               static_cast<double>(y), 0.0}));
  }

  bool isCollision(float x, float y, float) const override
  {
    if (scenario_ != Scenario::Mixed) {
      return false;
    }

    const bool first_band = x > 0.65F && x < 0.85F && std::abs(y) < 0.16F;
    const bool second_band = x > 1.65F && x < 1.82F && std::abs(y) < 0.10F;
    return first_band || second_band;
  }

private:
  Scenario scenario_;
};

struct BenchmarkRun
{
  mppi::OptimizationResult result;
  std::size_t retries;
};

class BenchmarkOptimizer final : public mppi::Optimizer
{
public:
  BenchmarkRun runOnce(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & path,
    const nav2_mppi_controller::IMapView & map_view)
  {
    prepare(pose, speed, path);
    std::size_t attempts = 0;
    do {
      ++attempts;
      optimize(map_view);
    } while (fallback(critics_data_.fail_flag));
    return {finalizeControlSequence(), attempts - 1};
  }
};

finenav_mppi_controller::Params makeParams(
  unsigned int batch_size,
  unsigned int time_steps,
  unsigned int iteration_count)
{
  finenav_mppi_controller::Params params;
  params.batch_size = batch_size;
  params.time_steps = time_steps;
  params.iteration_count = iteration_count;
  params.model_dt = 0.05;
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

  params.ConstraintCritic.enabled = true;
  params.CostCritic.enabled = true;
  params.GoalAngleCritic.enabled = true;
  params.GoalCritic.enabled = true;
  params.PathAlignCritic.enabled = true;
  params.PathAngleCritic.enabled = true;
  params.PathFollowCritic.enabled = true;
  params.PreferForwardCritic.enabled = true;
  params.TwirlingCritic.enabled = true;
  params.VelocityDeadbandCritic.enabled = true;
  return params;
}

std::vector<std::unique_ptr<mppi::critics::CriticFunction>> makeCritics()
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
  return critics;
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
    auto & pose = path.poses[i].pose;
    pose.position.x = 0.05 * static_cast<double>(i);
    pose.orientation.w = 1.0;
  }
  return path;
}

template<typename Array>
double checksumArray(const Array & array)
{
  double checksum = 0.0;
  if constexpr (requires {array.rows(); array.cols();}) {
    for (decltype(array.rows()) row = 0; row < array.rows(); ++row) {
      for (decltype(array.cols()) col = 0; col < array.cols(); ++col) {
        checksum += static_cast<double>(array(row, col)) *
          (1.0 + static_cast<double>(row + col) * 1.0e-4);
      }
    }
  } else {
    const auto shape = array.shape();
    for (std::size_t row = 0; row < shape[0]; ++row) {
      for (std::size_t col = 0; col < shape[1]; ++col) {
        checksum += static_cast<double>(array(row, col)) *
          (1.0 + static_cast<double>(row + col) * 1.0e-4);
      }
    }
  }
  return checksum;
}

template<typename Array>
std::size_t arrayRows(const Array & array)
{
  if constexpr (requires {array.rows();}) {
    return static_cast<std::size_t>(array.rows());
  } else {
    return array.shape()[0];
  }
}

template<typename Array>
std::size_t arrayCols(const Array & array)
{
  if constexpr (requires {array.cols();}) {
    return static_cast<std::size_t>(array.cols());
  } else {
    return array.shape()[1];
  }
}

template<typename Array>
void dumpArray(std::ofstream & output, const char * name, const Array & array)
{
  for (std::size_t row = 0; row < arrayRows(array); ++row) {
    for (std::size_t col = 0; col < arrayCols(array); ++col) {
      output << name << ' ' << row << ' ' << col << ' '
             << std::setprecision(9) << static_cast<double>(array(row, col)) << '\n';
    }
  }
}

double checksumResult(const mppi::OptimizationResult & result)
{
  return checksumArray(result.controls) + 0.5 * checksumArray(result.trajectory);
}

void runScenario(
  benchmark::State & state,
  Scenario scenario,
  unsigned int batch_size,
  unsigned int time_steps,
  unsigned int iteration_count)
{
#ifdef PRACTICAL_BENCHMARK_SEED
  PRACTICAL_BENCHMARK_SEED(42);
#endif
  const auto params = makeParams(batch_size, time_steps, iteration_count);
  BenchmarkOptimizer optimizer;
  auto critics = makeCritics();
  optimizer.initialize("PracticalComparison", critics, params);
  const auto pose = makePose();
  const auto speed = makeSpeed();
  const auto path = makePath();
  const ScenarioMapView map_view(scenario);

  auto warmup = optimizer.runOnce(pose, speed, path, map_view);
  benchmark::DoNotOptimize(warmup.result);

  std::optional<BenchmarkRun> last_run;
  for (auto _ : state) {
    last_run.emplace(optimizer.runOnce(pose, speed, path, map_view));
    benchmark::DoNotOptimize(last_run->result);
  }

  state.PauseTiming();
  if (last_run) {
    const auto & controls = last_run->result.controls;
    const auto & trajectory = last_run->result.trajectory;
    state.counters["checksum"] = checksumResult(last_run->result);
    state.counters["controls_checksum"] = checksumArray(controls);
    state.counters["trajectory_checksum"] = checksumArray(trajectory);
    state.counters["first_vx"] = controls(0, 0);
    state.counters["first_wz"] = controls(0, 1);
    state.counters["terminal_x"] = trajectory(arrayRows(trajectory) - 1, 0);
    state.counters["terminal_y"] = trajectory(arrayRows(trajectory) - 1, 1);
    state.counters["terminal_yaw"] = trajectory(arrayRows(trajectory) - 1, 2);
    state.counters["controls_rows"] = static_cast<double>(arrayRows(controls));
    state.counters["controls_cols"] = static_cast<double>(arrayCols(controls));
    state.counters["trajectory_rows"] = static_cast<double>(arrayRows(trajectory));
    state.counters["trajectory_cols"] = static_cast<double>(arrayCols(trajectory));
    state.counters["retries"] = static_cast<double>(last_run->retries);

    if (const char * dump_path = std::getenv("FINENAV_MPPI_BENCHMARK_DUMP_RESULT")) {
      std::ofstream output(dump_path);
      dumpArray(output, "controls", controls);
      dumpArray(output, "trajectory", trajectory);
    }
  }
  state.ResumeTiming();
  optimizer.shutdown();
}

double percentile95(const std::vector<double> & samples)
{
  if (samples.empty()) {
    return 0.0;
  }
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const auto index = static_cast<std::size_t>(std::ceil(0.95 * sorted.size())) - 1;
  return sorted[index];
}

void BM_Open5000(benchmark::State & state)
{
  runScenario(
    state, Scenario::Open, kPracticalBatchSize, kPracticalTimeSteps,
    kPracticalIterations);
}

void BM_Inflated5000(benchmark::State & state)
{
  runScenario(
    state, Scenario::Inflated, kPracticalBatchSize, kPracticalTimeSteps,
    kPracticalIterations);
}

void BM_Mixed5000(benchmark::State & state)
{
  runScenario(
    state, Scenario::Mixed, kPracticalBatchSize, kPracticalTimeSteps,
    kPracticalIterations);
}

void BM_Open3000(benchmark::State & state)
{
  runScenario(
    state, Scenario::Open, kCompatibilityBatchSize, kPracticalTimeSteps,
    kCompatibilityIterations);
}

void BM_Inflated3000(benchmark::State & state)
{
  runScenario(
    state, Scenario::Inflated, kCompatibilityBatchSize, kPracticalTimeSteps,
    kCompatibilityIterations);
}

void BM_Mixed3000(benchmark::State & state)
{
  runScenario(
    state, Scenario::Mixed, kCompatibilityBatchSize, kPracticalTimeSteps,
    kCompatibilityIterations);
}

}  // namespace

// Optimizer carries its control sequence across calls, so each repetition must
// measure the same single planning call in both implementations.
#define REGISTER_PRACTICAL_BENCHMARK(function, name) \
  BENCHMARK(function)->Name(name)->Iterations(1)->UseRealTime() \
  ->ComputeStatistics("p95", percentile95)

REGISTER_PRACTICAL_BENCHMARK(BM_Open5000, "practical/open_5000x56x2");
REGISTER_PRACTICAL_BENCHMARK(BM_Inflated5000, "practical/inflated_5000x56x2");
REGISTER_PRACTICAL_BENCHMARK(BM_Mixed5000, "practical/mixed_5000x56x2");
REGISTER_PRACTICAL_BENCHMARK(BM_Open3000, "practical/open_3000x56x1");
REGISTER_PRACTICAL_BENCHMARK(BM_Inflated3000, "practical/inflated_3000x56x1");
REGISTER_PRACTICAL_BENCHMARK(BM_Mixed3000, "practical/mixed_3000x56x1");

BENCHMARK_MAIN();
