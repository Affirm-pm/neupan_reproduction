// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include "finenav_mppi_controller/critics/cost_critic.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <exception>
#include <mutex>
#include <limits>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace mppi::critics
{
using nav2_mppi_controller::IMapView;

CostCritic::CostCritic(CostCriticExecutionOptions options)
: execution_options_(std::move(options))
{
  if (execution_options_.num_threads == 0U) {
    execution_options_.num_threads = 1U;
  }
  if (execution_options_.chunk_size == 0U) {
    execution_options_.chunk_size = 1U;
  }
}

CostCritic::~CostCritic() = default;

void CostCritic::executeTrajectories(
  std::size_t count,
  const std::function<void(std::size_t)> & callback)
{
  if (execution_options_.num_threads == 1U) {
    for (std::size_t index = 0; index < count; ++index) {
      callback(index);
    }
    return;
  }

#ifdef _OPENMP
  std::exception_ptr first_exception;
  std::mutex exception_mutex;
  std::atomic<bool> cancelled{false};
  std::atomic<int> observed_threads{1};
  auto invoke = [&](std::size_t index) {
      int observed = observed_threads.load(std::memory_order_relaxed);
      const int active_threads = omp_get_num_threads();
      while (active_threads > observed &&
        !observed_threads.compare_exchange_weak(
          observed, active_threads, std::memory_order_relaxed))
      {
      }
      if (cancelled.load(std::memory_order_relaxed)) {
        return;
      }
      try {
        callback(index);
      } catch (...) {
        std::lock_guard<std::mutex> lock(exception_mutex);
        if (!first_exception) {
          first_exception = std::current_exception();
        }
        cancelled.store(true, std::memory_order_relaxed);
      }
    };
  const int threads = static_cast<int>(execution_options_.num_threads);
  const int chunk = static_cast<int>(execution_options_.chunk_size);
#pragma omp parallel for schedule(dynamic, chunk) num_threads(threads)
  for (std::int64_t index = 0; index < static_cast<std::int64_t>(count); ++index) {
    invoke(static_cast<std::size_t>(index));
  }
  if (first_exception) {
    std::rethrow_exception(first_exception);
  }
  observed_openmp_threads_ = observed_threads.load(std::memory_order_relaxed);
  return;
#else
  throw std::runtime_error("CostCritic requires OpenMP for more than one thread");
#endif
}

void CostCritic::initialize(const finenav_mppi_controller::Params & p)
{
  logger_ = rclcpp::get_logger("CostCritic");
  const auto & c = p.CostCritic;
  enabled_ = c.enabled;
  power_ = static_cast<unsigned int>(c.cost_power);
  weight_ = static_cast<float>(c.cost_weight);
  critical_cost_ = static_cast<float>(c.critical_cost);
  collision_cost_ = static_cast<float>(c.collision_cost);
  near_goal_distance_ = static_cast<float>(c.near_goal_distance);

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;

  RCLCPP_INFO(
    logger_, "scheduler=OpenMPDynamic configured_threads=%zu chunk_size=%zu",
    execution_options_.num_threads,
    execution_options_.chunk_size);
}


void CostCritic::score(CriticData & data, const IMapView & map_view)
{
  if (!this->enabled_) {
    return;
  }


  // If near the goal, don't apply the preferential term since the goal is near obstacles
  const bool near_goal =
    utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path);

  models::Vector repulsive_cost(data.costs.size());
  repulsive_cost.setZero();

  const Eigen::Index trajectory_count = data.trajectories.x.rows();
  const Eigen::Index traj_len = data.trajectories.x.cols();
  std::vector<std::uint8_t> collision_flags(static_cast<std::size_t>(trajectory_count), 0U);

  executeTrajectories(
    static_cast<std::size_t>(trajectory_count),
    [&](std::size_t trajectory_index) {
      const Eigen::Index i = static_cast<Eigen::Index>(trajectory_index);
      bool trajectory_collide = false;
      const auto & traj = data.trajectories;

      for (Eigen::Index j = 0; j < traj_len; ++j) {
        // The costAtPose doesn't use orientation
        // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
        // So the center point has more information than the footprint
        const float pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), map_view);
        // In free space.
        if (pose_cost < 1.0f) {continue;}

        if (inCollision(traj.x(i, j), traj.y(i, j), traj.yaws(i, j), map_view)) {
          trajectory_collide = true;
          break;
        }

        // Let near-collision trajectory points be punished severely
        // Note that we collision check based on the footprint actual,
        // but score based on the center-point cost regardless
        if (pose_cost >= 253) {
          repulsive_cost[i] += critical_cost_;
        } else if (!near_goal) {
          // Generally prefer trajectories further from obstacles.
          repulsive_cost[i] += pose_cost;
        }
      }

      if (trajectory_collide) {
        repulsive_cost[i] = std::numeric_limits<float>::max();
      }
      collision_flags[trajectory_index] = trajectory_collide ? 1U : 0U;
    });

  bool all_trajectories_collide = true;
  for (const std::uint8_t collision : collision_flags) {
    if (collision == 0U) {
      all_trajectories_collide = false;
      break;
    }
  }

  if (data.critic_cost_breakdown) {
    data.critic_cost_breakdown->collision_flags = collision_flags;
  }

  data.costs += (weight_ * repulsive_cost / static_cast<float>(traj_len)).pow(power_);
  data.fail_flag = all_trajectories_collide;
  static rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
  RCLCPP_INFO_THROTTLE(
    logger_, throttle_clock, 2000,
    "CostCritic scheduler=OpenMPDynamic observed_threads=%d configured_threads=%zu",
    observed_openmp_threads_, execution_options_.num_threads);
  RCLCPP_DEBUG(logger_, "Scored");
}


}  // namespace mppi::critics
