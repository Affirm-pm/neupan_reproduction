// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/critics/cost_critic.hpp"

namespace mppi::critics
{
namespace
{

class DeterministicMapView final : public nav2_mppi_controller::IMapView
{
public:
  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return true;}
  bool isCollision(float x, float, float) const override {return x < -0.5f;}
  float getRadius() const override {return 0.2f;}

  int getCost(const Position3D & position) const override
  {
    const int query = query_count.fetch_add(1, std::memory_order_relaxed);
    if (throw_after >= 0 && query >= throw_after) {
      throw std::runtime_error("deterministic map failure");
    }
    const double x = position.x();
    if (x < -0.5) {
      return 254;
    }
    if (x < 0.0) {
      return 80;
    }
    if (x < 1.0) {
      return 0;
    }
    return 200;
  }

  float costAtPose(float x, float y, float) const override
  {
    return static_cast<float>(getCost(Position3D{x, y, 0.0}));
  }

  std::string getBaseFrameID() const override {return "base_link";}

  mutable std::atomic<int> query_count{0};
  int throw_after{-1};
};

class ConcurrentAccessMapView final : public nav2_mppi_controller::IMapView
{
public:
  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return true;}
  bool isCollision(float, float, float) const override {return false;}
  float getRadius() const override {return 0.2F;}

  int getCost(const Position3D &) const override
  {
    const int current = active_calls_.fetch_add(1, std::memory_order_relaxed) + 1;
    int observed = max_active_calls_.load(std::memory_order_relaxed);
    while (current > observed &&
      !max_active_calls_.compare_exchange_weak(
        observed, current, std::memory_order_relaxed))
    {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    active_calls_.fetch_sub(1, std::memory_order_relaxed);
    return 80;
  }

  float costAtPose(float x, float y, float) const override
  {
    return static_cast<float>(getCost(Position3D{x, y, 0.0}));
  }

  std::string getBaseFrameID() const override {return "base_link";}
  int maxActiveCalls() const {return max_active_calls_.load(std::memory_order_relaxed);}

private:
  mutable std::atomic<int> active_calls_{0};
  mutable std::atomic<int> max_active_calls_{0};
};

class CountingWorkloadMapView final : public nav2_mppi_controller::IMapView
{
public:
  static constexpr std::size_t max_trajectories = 3000U;

  explicit CountingWorkloadMapView(bool uneven_work)
  : uneven_work_(uneven_work)
  {
    reset();
  }

  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return true;}
  float getRadius() const override {return 0.2F;}

  int getCost(const Position3D & position) const override
  {
    const std::size_t trajectory = trajectoryIndex(position.x());
    if (trajectory >= max_trajectories) {
      return 0;
    }

    const int active = active_calls_.fetch_add(1, std::memory_order_relaxed) + 1;
    int observed = max_active_calls_.load(std::memory_order_relaxed);
    while (active > observed &&
      !max_active_calls_.compare_exchange_weak(
        observed, active, std::memory_order_relaxed))
    {
    }

    if (uneven_work_ && trajectory % 8U == 0U) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    query_counts_[trajectory].fetch_add(1U, std::memory_order_relaxed);
    active_calls_.fetch_sub(1, std::memory_order_relaxed);
    return 80;
  }

  bool isCollision(float x, float, float) const override
  {
    const std::size_t trajectory = trajectoryIndex(x);
    if (trajectory < max_trajectories) {
      collision_counts_[trajectory].fetch_add(1U, std::memory_order_relaxed);
    }
    return false;
  }

  float costAtPose(float x, float y, float) const override
  {
    return static_cast<float>(getCost(Position3D{x, y, 0.0}));
  }

  std::string getBaseFrameID() const override {return "base_link";}

  void reset() const
  {
    active_calls_.store(0, std::memory_order_relaxed);
    max_active_calls_.store(0, std::memory_order_relaxed);
    for (std::size_t index = 0; index < max_trajectories; ++index) {
      query_counts_[index].store(0U, std::memory_order_relaxed);
      collision_counts_[index].store(0U, std::memory_order_relaxed);
    }
  }

  std::size_t queryCount(std::size_t trajectory) const
  {
    return query_counts_[trajectory].load(std::memory_order_relaxed);
  }

  std::size_t collisionCount(std::size_t trajectory) const
  {
    return collision_counts_[trajectory].load(std::memory_order_relaxed);
  }

  int maxActiveCalls() const
  {
    return max_active_calls_.load(std::memory_order_relaxed);
  }

private:
  static std::size_t trajectoryIndex(double x)
  {
    if (x < 0.0 || x >= static_cast<double>(max_trajectories)) {
      return max_trajectories;
    }
    return static_cast<std::size_t>(x);
  }

  bool uneven_work_;
  mutable std::atomic<int> active_calls_{0};
  mutable std::atomic<int> max_active_calls_{0};
  mutable std::array<std::atomic<std::size_t>, max_trajectories> query_counts_{};
  mutable std::array<std::atomic<std::size_t>, max_trajectories> collision_counts_{};
};

struct CostCriticFixture
{
  static constexpr unsigned int batch_size = 4;
  static constexpr unsigned int time_steps = 6;

  models::State state;
  models::Trajectories trajectories;
  models::Path path;
  models::Vector costs{models::Vector::Zero(batch_size)};
  float model_dt{0.1f};
  CriticData data{state, trajectories, path, costs, model_dt, false,
    std::make_shared<DiffDriveMotionModel>(),
    std::nullopt, std::nullopt};

  CostCriticFixture()
  {
    state.reset(batch_size, time_steps);
    trajectories.reset(batch_size, time_steps);
    path.reset(3);
    state.pose.pose.orientation.w = 1.0;
    path.x << 0.0f, 5.0f, 10.0f;

    trajectories.x.row(0).setConstant(-1.0f);
    trajectories.x.row(1).setConstant(-0.25f);
    trajectories.x.row(2).setConstant(0.5f);
    trajectories.x.row(3).setConstant(2.0f);
  }
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

struct ScoreResult
{
  models::Vector costs;
  bool fail_flag;
};

ScoreResult scoreWithOptions(
  std::size_t thread_count,
  std::size_t chunk_size = 1U,
  DeterministicMapView * map_view = nullptr)
{
  CostCriticFixture fixture;
  DeterministicMapView local_map_view;
  auto & selected_map_view = map_view == nullptr ? local_map_view : *map_view;
  CostCritic critic({thread_count, chunk_size});
  critic.initialize(makeParams());
  critic.score(fixture.data, selected_map_view);
  return {fixture.costs, fixture.data.fail_flag};
}

void expectSameResult(const ScoreResult & expected, const ScoreResult & actual)
{
  ASSERT_EQ(expected.costs.size(), actual.costs.size());
  for (Eigen::Index index = 0; index < expected.costs.size(); ++index) {
    EXPECT_NEAR(expected.costs(index), actual.costs(index), 1.0e-5F);
  }
  EXPECT_EQ(expected.fail_flag, actual.fail_flag);
}

struct LargeCostCriticFixture
{
  LargeCostCriticFixture(unsigned int batch_size, unsigned int time_steps)
  : costs(models::Vector::Zero(batch_size)),
    data{state, trajectories, path, costs, model_dt, false,
      std::make_shared<DiffDriveMotionModel>(), std::nullopt, std::nullopt}
  {
    state.reset(batch_size, time_steps);
    trajectories.reset(batch_size, time_steps);
    path.reset(2);
    state.pose.pose.orientation.w = 1.0;
    path.x << 100.0F, 101.0F;
    for (Eigen::Index trajectory = 0; trajectory < trajectories.x.rows(); ++trajectory) {
      trajectories.x.row(trajectory).setConstant(static_cast<float>(trajectory));
      trajectories.y.row(trajectory).setZero();
      trajectories.yaws.row(trajectory).setZero();
    }
  }

  models::State state;
  models::Trajectories trajectories;
  models::Path path;
  models::Vector costs;
  float model_dt{0.05F};
  CriticData data;
};

void expectCompleteParallelScore(
  unsigned int batch_size,
  unsigned int time_steps,
  bool uneven_work,
  unsigned int rounds)
{
  ASSERT_LE(batch_size, CountingWorkloadMapView::max_trajectories);
  CountingWorkloadMapView map_view(uneven_work);
  LargeCostCriticFixture fixture(batch_size, time_steps);
  CostCritic critic({4U, 1U});
  critic.initialize(makeParams());

  for (unsigned int round = 0; round < rounds; ++round) {
    map_view.reset();
    fixture.costs.setZero();
    fixture.data.fail_flag = false;
    ASSERT_NO_THROW(critic.score(fixture.data, map_view));
    ASSERT_FALSE(fixture.data.fail_flag);
    ASSERT_TRUE(fixture.costs.allFinite());
    for (unsigned int trajectory = 0; trajectory < batch_size; ++trajectory) {
      EXPECT_EQ(map_view.queryCount(trajectory), time_steps);
      EXPECT_EQ(map_view.collisionCount(trajectory), time_steps);
      EXPECT_GT(fixture.costs(static_cast<Eigen::Index>(trajectory)), 0.0F);
    }
  }

  if (uneven_work) {
    EXPECT_GT(map_view.maxActiveCalls(), 1);
  }
}

TEST(CostCriticParallelTest, SingleThreadOptionsMatchDefaultCritic) {
  const auto params = makeParams();
  const DeterministicMapView map_view;
  CostCriticFixture default_fixture;
  CostCriticFixture explicit_fixture;
  CostCritic default_critic;
  CostCritic explicit_critic({1U, 1U});
  default_critic.initialize(params);
  explicit_critic.initialize(params);

  default_critic.score(default_fixture.data, map_view);
  explicit_critic.score(explicit_fixture.data, map_view);

  ASSERT_EQ(default_fixture.costs.size(), explicit_fixture.costs.size());
  for (Eigen::Index index = 0; index < default_fixture.costs.size(); ++index) {
    EXPECT_FLOAT_EQ(default_fixture.costs(index), explicit_fixture.costs(index));
  }
  EXPECT_EQ(default_fixture.data.fail_flag, explicit_fixture.data.fail_flag);
  EXPECT_FALSE(explicit_fixture.data.fail_flag);
}

TEST(CostCriticParallelTest, OpenMPDynamicMatchesSerialOracle) {
  const ScoreResult serial = scoreWithOptions(1U);
  expectSameResult(serial, scoreWithOptions(4U, 2U));
}

TEST(CostCriticParallelTest, MoreThreadsThanTrajectoriesStillScoresEveryTrajectory) {
  const ScoreResult serial = scoreWithOptions(1U);
  expectSameResult(serial, scoreWithOptions(16U, 1U));
}

TEST(CostCriticParallelTest, OpenMPDynamicCanBeReusedAfterCallbackException) {
  DeterministicMapView map_view;
  map_view.throw_after = 1;
  CostCriticFixture failing_fixture;
  CostCritic critic({4U, 1U});
  critic.initialize(makeParams());
  EXPECT_THROW(critic.score(failing_fixture.data, map_view), std::runtime_error);

  map_view.throw_after = -1;
  map_view.query_count.store(0, std::memory_order_relaxed);
  CostCriticFixture recovered_fixture;
  EXPECT_NO_THROW(critic.score(recovered_fixture.data, map_view));
  EXPECT_FALSE(recovered_fixture.data.fail_flag);
}

TEST(CostCriticParallelTest, OpenMPDynamicPropagatesMapViewExceptions) {
  DeterministicMapView map_view;
  map_view.throw_after = 1;
  CostCriticFixture fixture;
  CostCritic critic({4U, 1U});
  critic.initialize(makeParams());
  EXPECT_THROW(critic.score(fixture.data, map_view), std::runtime_error);
}

TEST(CostCriticParallelTest, OpenMPDynamicCallsSharedMapViewConcurrently) {
  ConcurrentAccessMapView map_view;
  CostCriticFixture fixture;
  CostCritic critic({4U, 1U});
  critic.initialize(makeParams());
  critic.score(fixture.data, map_view);
  EXPECT_GT(map_view.maxActiveCalls(), 1);
}

TEST(CostCriticParallelTest, ProductionBatchIsCompleteAcrossRepeatedParallelScores)
{
  expectCompleteParallelScore(3000U, 30U, false, 2U);
}

TEST(CostCriticParallelTest, UnevenWorkloadIsCompleteAcrossRepeatedScores)
{
  expectCompleteParallelScore(64U, 20U, true, 3U);
}

}  // namespace
}  // namespace mppi::critics
