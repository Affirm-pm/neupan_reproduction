// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.

#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/critics/cost_critic.hpp"
#include "cost_critic_heavy_fixture.hpp"

namespace
{

using mppi::CriticData;
using mppi::DiffDriveMotionModel;
using mppi::critics::CostCritic;
using mppi::models::Vector;
namespace models = mppi::models;

enum class Scenario : int
{
  CenterCost = 0,
  FootprintScan = 1,
  MixedEarlyCollision = 2
};

class BenchmarkMapView final : public nav2_mppi_controller::IMapView
{
public:
  explicit BenchmarkMapView(Scenario scenario)
  : scenario_(scenario) {}

  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return scenario_ != Scenario::CenterCost;}
  float getRadius() const override {return 0.25F;}
  std::string getBaseFrameID() const override {return "base_link";}

  int getCost(const Position3D & position) const override
  {
    const float x = static_cast<float>(position.x());
    const float y = static_cast<float>(position.y());
    const float oscillation = std::abs(std::sin(1.7F * x + 0.3F * y)) * 80.0F;
    return static_cast<int>(std::clamp(20.0F + oscillation + std::abs(y) * 120.0F, 1.0F, 220.0F));
  }

  float costAtPose(float x, float y, float) const override
  {
    return static_cast<float>(getCost(
             Position3D{static_cast<double>(x), static_cast<double>(y),
               0.0}));
  }

  bool isCollision(float x, float y, float theta) const override
  {
    if (scenario_ == Scenario::CenterCost) {
      return false;
    }

    // A deterministic stand-in for a footprint map query. The arithmetic keeps
    // the query non-trivial while remaining independent of shared mutable state.
    float accumulator = x + y + theta;
    const int work = scenario_ == Scenario::FootprintScan ? 96 : 32;
    for (int iteration = 0; iteration < work; ++iteration) {
      accumulator = std::sin(accumulator + 0.013F * static_cast<float>(iteration));
    }
    (void)accumulator;

    const bool collision_band = x > 0.15F && x < 0.19F && std::abs(y) < 0.22F;
    if (scenario_ == Scenario::FootprintScan) {
      return collision_band && y > 0.0F;
    }
    return collision_band;
  }

private:
  Scenario scenario_;
};

struct BenchmarkInput
{
  BenchmarkInput(unsigned int batch_size, unsigned int time_steps, Scenario scenario)
  : costs(models::Vector::Zero(batch_size)), map_view(scenario),
    data{state, trajectories, path, costs, model_dt, false,
      std::make_shared<DiffDriveMotionModel>(), std::nullopt, std::nullopt}
  {
    state.reset(batch_size, time_steps);
    trajectories.reset(batch_size, time_steps);
    path.reset(32);
    state.pose.pose.orientation.w = 1.0;
    for (Eigen::Index point = 0; point < path.x.size(); ++point) {
      path.x(point) = 0.25F * static_cast<float>(point);
      path.y(point) = 0.0F;
    }

    for (Eigen::Index trajectory = 0; trajectory < trajectories.x.rows(); ++trajectory) {
      const float lateral = (static_cast<int>(trajectory % 17U) - 8) * 0.035F;
      for (Eigen::Index step = 0; step < trajectories.x.cols(); ++step) {
        trajectories.x(trajectory, step) = 0.02F * static_cast<float>(step);
        trajectories.y(trajectory, step) = lateral;
        trajectories.yaws(trajectory, step) = 0.01F * static_cast<float>(trajectory % 11U);
      }
    }
  }

  void resetCosts()
  {
    costs.setZero();
    data.fail_flag = false;
  }

  models::State state;
  models::Trajectories trajectories;
  models::Path path;
  models::Vector costs;
  float model_dt{0.05F};
  BenchmarkMapView map_view;
  CriticData data;
};

struct HeavyBenchmarkInput
{
  HeavyBenchmarkInput(unsigned int batch_size, unsigned int time_steps)
  : costs(models::Vector::Zero(batch_size)),
    data{state, trajectories, path, costs, model_dt, false,
      std::make_shared<DiffDriveMotionModel>(), std::nullopt, std::nullopt}
  {
    state.reset(batch_size, time_steps);
    trajectories.reset(batch_size, time_steps);
    path.reset(2);
    state.pose.pose.orientation.w = 1.0;
    path.x << 0.0F, 10.0F;
    path.y << 0.0F, 0.0F;
    mppi::benchmark_support::populateHeavyObstacleTrajectories(trajectories);
  }

  void resetCosts()
  {
    costs.setZero();
    data.fail_flag = false;
  }

  models::State state;
  models::Trajectories trajectories;
  models::Path path;
  models::Vector costs;
  float model_dt{0.05F};
  mppi::benchmark_support::HeavyObstacleMapView map_view;
  CriticData data;
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

double checksum(const models::Vector & values)
{
  double result = 0.0;
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    result += static_cast<double>(values(index)) * (1.0 + 1.0e-5 * static_cast<double>(index));
  }
  return result;
}

template<typename Input>
void expectOracle(
  const models::Vector & oracle_costs, bool oracle_fail,
  const Input & candidate)
{
  if (oracle_fail != candidate.data.fail_flag || oracle_costs.size() != candidate.costs.size()) {
    throw std::runtime_error("CostCritic scheduling changed fail flag or output size");
  }
  for (Eigen::Index index = 0; index < oracle_costs.size(); ++index) {
    if (std::abs(oracle_costs(index) - candidate.costs(index)) > 1.0e-5F) {
      throw std::runtime_error("CostCritic scheduling changed trajectory cost");
    }
  }
}

void BM_CostCritic(benchmark::State & state)
{
  const unsigned int batch_size = static_cast<unsigned int>(state.range(0));
  const unsigned int time_steps = static_cast<unsigned int>(state.range(1));
  const std::size_t thread_count = static_cast<std::size_t>(state.range(2));
  const Scenario scenario = static_cast<Scenario>(state.range(3));
  const std::size_t chunk_size = static_cast<std::size_t>(state.range(4));

  BenchmarkInput oracle(batch_size, time_steps, scenario);
  CostCritic serial_critic({1U, 1U});
  serial_critic.initialize(makeParams());
  serial_critic.score(oracle.data, oracle.map_view);
  const models::Vector oracle_costs = oracle.costs;
  const bool oracle_fail = oracle.data.fail_flag;

  BenchmarkInput candidate(batch_size, time_steps, scenario);
  CostCritic candidate_critic({thread_count, chunk_size});
  candidate_critic.initialize(makeParams());
  candidate_critic.score(candidate.data, candidate.map_view);
  expectOracle(oracle_costs, oracle_fail, candidate);

  for (auto _ : state) {
    state.PauseTiming();
    candidate.resetCosts();
    state.ResumeTiming();
    candidate_critic.score(candidate.data, candidate.map_view);
    benchmark::DoNotOptimize(candidate.costs.data());
    benchmark::DoNotOptimize(candidate.data.fail_flag);
  }

  state.counters["batch"] = static_cast<double>(batch_size);
  state.counters["time_steps"] = static_cast<double>(time_steps);
  state.counters["threads"] = static_cast<double>(thread_count);
  state.counters["checksum"] = checksum(candidate.costs);
}

void BM_CostCriticHeavy(benchmark::State & state)
{
  const unsigned int batch_size = static_cast<unsigned int>(state.range(0));
  const unsigned int time_steps = static_cast<unsigned int>(state.range(1));
  const std::size_t thread_count = static_cast<std::size_t>(state.range(2));
  const std::size_t chunk_size = static_cast<std::size_t>(state.range(3));

  HeavyBenchmarkInput oracle(batch_size, time_steps);
  CostCritic serial_critic({1U, 1U});
  serial_critic.initialize(makeParams());
  serial_critic.score(oracle.data, oracle.map_view);
  const models::Vector oracle_costs = oracle.costs;
  const bool oracle_fail = oracle.data.fail_flag;

  HeavyBenchmarkInput candidate(batch_size, time_steps);
  CostCritic candidate_critic({thread_count, chunk_size});
  candidate_critic.initialize(makeParams());
  candidate_critic.score(candidate.data, candidate.map_view);
  expectOracle(oracle_costs, oracle_fail, candidate);

  for (auto _ : state) {
    state.PauseTiming();
    candidate.resetCosts();
    state.ResumeTiming();
    candidate_critic.score(candidate.data, candidate.map_view);
    benchmark::DoNotOptimize(candidate.costs.data());
    benchmark::DoNotOptimize(candidate.data.fail_flag);
  }

  state.counters["batch"] = static_cast<double>(batch_size);
  state.counters["time_steps"] = static_cast<double>(time_steps);
  state.counters["threads"] = static_cast<double>(thread_count);
  state.counters["checksum"] = checksum(candidate.costs);
}

}  // namespace

void ConfigureCostCriticBenchmark(benchmark::internal::Benchmark * benchmark)
{
  constexpr std::array<unsigned int, 2> batches{3000U, 5000U};
  constexpr std::array<unsigned int, 2> time_steps{30U, 56U};
  constexpr std::array<int, 3> scenarios{0, 1, 2};
  constexpr std::array<int, 3> thread_counts{2, 4, 8};

  // One thread is the serial oracle. More threads use OpenMP dynamic scheduling.
  for (std::size_t workload = 0; workload < batches.size(); ++workload) {
    for (const int scenario : scenarios) {
      benchmark->Args(
      {
        static_cast<int>(batches[workload]), static_cast<int>(time_steps[workload]),
        1, scenario, 1});
      for (const int threads : thread_counts) {
        benchmark->Args(
        {
          static_cast<int>(batches[workload]), static_cast<int>(time_steps[workload]),
          threads, scenario, 1});
      }
    }
  }
}

void ConfigureHeavyCostCriticBenchmark(benchmark::internal::Benchmark * benchmark)
{
  constexpr std::array<unsigned int, 2> batches{3000U, 5000U};
  constexpr std::array<unsigned int, 2> time_steps{30U, 56U};
  constexpr std::array<int, 3> thread_counts{2, 4, 8};

  for (std::size_t workload = 0; workload < batches.size(); ++workload) {
    benchmark->Args(
    {
      static_cast<int>(batches[workload]), static_cast<int>(time_steps[workload]),
      1, 1});
    for (const int threads : thread_counts) {
      benchmark->Args(
      {
        static_cast<int>(batches[workload]), static_cast<int>(time_steps[workload]),
        threads, 1});
    }
  }
}

BENCHMARK(BM_CostCritic)->Apply(ConfigureCostCriticBenchmark)->UseRealTime();
BENCHMARK(BM_CostCriticHeavy)->Apply(ConfigureHeavyCostCriticBenchmark)->UseRealTime();

BENCHMARK_MAIN();
