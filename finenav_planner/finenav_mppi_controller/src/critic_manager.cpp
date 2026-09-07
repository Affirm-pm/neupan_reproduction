// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "finenav_mppi_controller/critic_manager.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>
#include <utility>

#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"

namespace
{

bool isFiniteFloat(const float value)
{
  constexpr std::uint32_t exponent_mask = 0x7F800000U;
  return (std::bit_cast<std::uint32_t>(value) & exponent_mask) != exponent_mask;
}

const char * criticName(const mppi::critics::CriticFunction & critic)
{
  if (dynamic_cast<const mppi::critics::ConstraintCritic *>(&critic)) {
    return "ConstraintCritic";
  }
  if (dynamic_cast<const mppi::critics::CostCritic *>(&critic)) {
    return "CostCritic";
  }
  if (dynamic_cast<const mppi::critics::GoalAngleCritic *>(&critic)) {
    return "GoalAngleCritic";
  }
  if (dynamic_cast<const mppi::critics::GoalCritic *>(&critic)) {
    return "GoalCritic";
  }
  if (dynamic_cast<const mppi::critics::PathAlignCritic *>(&critic)) {
    return "PathAlignCritic";
  }
  if (dynamic_cast<const mppi::critics::PathAngleCritic *>(&critic)) {
    return "PathAngleCritic";
  }
  if (dynamic_cast<const mppi::critics::PathFollowCritic *>(&critic)) {
    return "PathFollowCritic";
  }
  if (dynamic_cast<const mppi::critics::PreferForwardCritic *>(&critic)) {
    return "PreferForwardCritic";
  }
  if (dynamic_cast<const mppi::critics::TwirlingCritic *>(&critic)) {
    return "TwirlingCritic";
  }
  if (dynamic_cast<const mppi::critics::VelocityDeadbandCritic *>(&critic)) {
    return "VelocityDeadbandCritic";
  }
  return "UnknownCritic";
}

}  // namespace

namespace mppi
{
using namespace nav2_mppi_controller;

void CriticManager::on_configure(
    std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
    const finenav_mppi_controller::Params & params)
{
    critics_ = std::move(critics);
    for (size_t q = 0; q < critics_.size(); q++) {
        critics_[q]->initialize(params);
    }
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data, const IMapView& map_view) const
{
    if (data.critic_cost_breakdown) {
        data.critic_cost_breakdown->clear();
    }

    for (size_t q = 0; q < critics_.size(); q++) {
        if (data.fail_flag) {
            break;
        }

        models::Vector costs_before;
        if (data.critic_cost_breakdown) {
            costs_before = data.costs;
        }
        critics_[q]->score(data, map_view);

        if (data.critic_cost_breakdown) {
            if (costs_before.size() != data.costs.size()) {
                continue;
            }
            CriticCostBreakdown::Contribution contribution(
              static_cast<std::size_t>(data.costs.size()), 0.0F);
            for (Eigen::Index i = 0; i < data.costs.size(); ++i) {
                const float before = costs_before(i);
                const float after = data.costs(i);
                if (isFiniteFloat(before) && isFiniteFloat(after)) {
                    const float delta = after - before;
                    contribution[static_cast<std::size_t>(i)] = isFiniteFloat(delta) ?
                      std::max(delta, 0.0F) :
                      std::numeric_limits<float>::infinity();
                } else if (isFiniteFloat(before) && !isFiniteFloat(after)) {
                    contribution[static_cast<std::size_t>(i)] =
                      std::numeric_limits<float>::infinity();
                }
            }
            data.critic_cost_breakdown->names.emplace_back(criticName(*critics_[q]));
            data.critic_cost_breakdown->contributions.push_back(std::move(contribution));
        }
    }

    if (data.critic_cost_breakdown) {
        data.critic_cost_breakdown->valid = !data.critic_cost_breakdown->names.empty();
    }
}

}  // namespace mppi
