// Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <bit>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "finenav_mppi_controller/critic_manager.hpp"

namespace
{

static_assert(
  std::is_same_v<
    typename decltype(mppi::CriticCostBreakdown::contributions)::value_type,
    std::vector<float>>,
  "Diagnostic critic contributions must use STL-owned float storage");

class EmptyMapView final : public nav2_mppi_controller::IMapView
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

class AddCritic final : public mppi::critics::CriticFunction
{
public:
  explicit AddCritic(const float amount)
  : amount_(amount) {}

  void initialize(const finenav_mppi_controller::Params &) override {}

  void score(
    mppi::CriticData & data, const nav2_mppi_controller::IMapView &) override
  {
    data.costs += mppi::models::Vector::Constant(data.costs.size(), amount_);
  }

private:
  float amount_;
};

class CollisionCritic final : public mppi::critics::CriticFunction
{
public:
  void initialize(const finenav_mppi_controller::Params &) override {}

  void score(
    mppi::CriticData & data, const nav2_mppi_controller::IMapView &) override
  {
    data.costs(0) = std::numeric_limits<float>::infinity();
  }
};

TEST(CriticBreakdownTest, CapturesFiniteAndCollisionDeltas)
{
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  mppi::models::Path path;
  mppi::models::Vector costs = mppi::models::Vector::Zero(2);
  float model_dt = 0.05F;
  mppi::CriticCostBreakdown breakdown;
  mppi::CriticData data{
    state, trajectories, path, costs, model_dt, false, nullptr, std::nullopt, std::nullopt,
    &breakdown};

  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  critics.push_back(std::make_unique<AddCritic>(2.5F));
  critics.push_back(std::make_unique<CollisionCritic>());

  finenav_mppi_controller::Params params;
  mppi::CriticManager manager;
  manager.on_configure(critics, params);
  EmptyMapView map_view;
  manager.evalTrajectoriesScores(data, map_view);

  ASSERT_TRUE(breakdown.valid);
  ASSERT_EQ(breakdown.names.size(), 2U);
  ASSERT_EQ(breakdown.contributions.size(), 2U);
  EXPECT_EQ(breakdown.names[0], "UnknownCritic");
  EXPECT_EQ(breakdown.names[1], "UnknownCritic");
  EXPECT_FLOAT_EQ(breakdown.contributions[0][0], 2.5F);
  EXPECT_FLOAT_EQ(breakdown.contributions[0][1], 2.5F);
  EXPECT_EQ(
    std::bit_cast<std::uint32_t>(breakdown.contributions[1][0]), 0x7F800000U);
  EXPECT_FLOAT_EQ(breakdown.contributions[1][1], 0.0F);
  EXPECT_EQ(std::bit_cast<std::uint32_t>(data.costs(0)), 0x7F800000U);
  EXPECT_FLOAT_EQ(data.costs(1), 2.5F);
}

TEST(CriticBreakdownTest, NullBreakdownPreservesScoringPath)
{
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  mppi::models::Path path;
  mppi::models::Vector costs = mppi::models::Vector::Zero(2);
  float model_dt = 0.05F;
  mppi::CriticData data{
    state, trajectories, path, costs, model_dt, false, nullptr, std::nullopt, std::nullopt};

  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  critics.push_back(std::make_unique<AddCritic>(1.0F));
  finenav_mppi_controller::Params params;
  mppi::CriticManager manager;
  manager.on_configure(critics, params);
  EmptyMapView map_view;
  manager.evalTrajectoriesScores(data, map_view);

  EXPECT_FLOAT_EQ(data.costs(0), 1.0F);
  EXPECT_FLOAT_EQ(data.costs(1), 1.0F);
}

}  // namespace
