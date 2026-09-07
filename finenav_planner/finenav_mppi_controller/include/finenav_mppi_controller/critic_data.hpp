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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "nav2_core/goal_checker.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"
#include "finenav_mppi_controller/models/path.hpp"
#include "finenav_mppi_controller/motion_models.hpp"


namespace mppi
{

struct CriticCostBreakdown
{
  using Contribution = std::vector<float>;

  std::vector<std::string> names;
  // Diagnostic data is copied into ROS messages before it leaves the optimizer.
  // Keep it in plain STL storage so reset/destruction never depends on Eigen's
  // alignment or allocator state.
  std::vector<Contribution> contributions;
  // Per-trajectory collision state reported by CostCritic. An explicit flag
  // avoids inferring collisions from floating-point sentinel costs.
  std::vector<std::uint8_t> collision_flags;
  bool valid{false};

  void clear()
  {
    names.clear();
    contributions.clear();
    collision_flags.clear();
    valid = false;
  }
};

/**
 * @struct mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories, path, costs, and
 * important parameters to share
 */
struct CriticData
{
  const models::State & state;
  const models::Trajectories & trajectories;
  const models::Path & path;

  models::Vector & costs;
  float & model_dt;

  bool fail_flag;
  std::shared_ptr<MotionModel> motion_model;
  std::optional<std::vector<bool>> path_pts_valid;
  std::optional<size_t> furthest_reached_path_point;

  // Optional diagnostic output. Keeping this nullable avoids a batch-sized
  // costs copy on the normal non-visualized control path.
  CriticCostBreakdown * critic_cost_breakdown{nullptr};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
