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

#include "nav2_mppi_controller/critics/path_angle_critic.hpp"

#include <math.h>

namespace mppi::critics
{

template <typename MapView>
void PathAngleCritic<MapView>::initialize()
{
  // 直接从父节点参数取固定默认值（原来注释给出的默认）
  float vx_min = -0.35f;
  if (fabs(vx_min) < 1e-6) {  // zero
    reversing_allowed_ = false;
  } else if (vx_min < 0.0f) {   // reversing possible
    reversing_allowed_ = true;
  }

  // 直接赋默认值，不动态加载
  offset_from_furthest_ = 4u;
  power_ = 1;
  weight_ = 2.0f;
  threshold_to_consider_ = 0.5f;
  max_angle_to_furthest_ = 1.2f;
  forward_preference_ = true;

  if (!reversing_allowed_) {
    forward_preference_ = true;
  }

//  RCLCPP_INFO(
//    logger_,
//    "PathAngleCritic instantiated with %d power and %f weight. Reversing %s",
//    power_, weight_, reversing_allowed_ ? "allowed." : "not allowed.");
}

template <typename MapView>
void PathAngleCritic<MapView>::score(CriticData & data, const MapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }

  if (utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

  const float goal_x = xt::view(data.path.x, offseted_idx);
  const float goal_y = xt::view(data.path.y, offseted_idx);

  if (utils::posePointAngle(
      data.state.pose.pose, goal_x, goal_y, forward_preference_) < max_angle_to_furthest_)
  {
    return;
  }

  auto yaws_between_points = xt::atan2(
    goal_y - data.trajectories.y,
    goal_x - data.trajectories.x);

  auto yaws =
    xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

  if (reversing_allowed_ && !forward_preference_) {
    const auto yaws_between_points_corrected = xt::where(
      yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
    const auto corrected_yaws = xt::abs(
      utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
    data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
  } else {
    data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
  }
}

}  // namespace mppi::critics

