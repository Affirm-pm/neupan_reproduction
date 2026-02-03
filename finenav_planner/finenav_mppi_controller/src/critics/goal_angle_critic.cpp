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

#include "finenav_mppi_controller/critics/goal_angle_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void GoalAngleCritic::initialize()
{
  // 直接赋固定默认值（不再从参数服务器读取）
  power_ = 1;
  weight_ = 3.0f;
  threshold_to_consider_ = 0.5f;
  enabled_ = true;
  // RCLCPP_INFO(
  //   logger_,
  //   "GoalAngleCritic instantiated with %d power, %f weight, and %f "
  //   "angular threshold.",
  //   power_, weight_, threshold_to_consider_);
}

void GoalAngleCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_ )    //TODO:旧有的检查与距离的逻辑会触发，为什么
  {
    return;
  }

  const auto goal_idx = data.path.x.shape(0) - 1;
  const float goal_yaw = data.path.yaws(goal_idx);

  data.costs += xt::pow(
    xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
    weight_, power_);
  std::cout<<"[GoalAngleCritic]: Scored"<<std::endl;
}

}  // namespace mppi::critics

