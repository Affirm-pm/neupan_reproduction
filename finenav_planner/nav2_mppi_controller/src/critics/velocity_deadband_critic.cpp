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

#include "nav2_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{

template <typename MapView>
void VelocityDeadbandCritic<MapView>::initialize()
{
  // 直接赋默认值，不动态加载
  power_ = 1;
  weight_ = 35.0f;

  // Recast double to float
  std::vector<double> deadband_velocities{0.0, 0.0, 0.0};
  // 如果需要修改 deadband，可在此处直接更改以上数组
  std::transform(
    deadband_velocities.begin(), deadband_velocities.end(), deadband_velocities_.begin(),
    [](double d) {return static_cast<float>(d);});
//
//  RCLCPP_INFO_STREAM(
//    logger_, "VelocityDeadbandCritic instantiated with "
//      << power_ << " power, " << weight_ << " weight, deadband_velocity ["
//      << deadband_velocities_.at(0) << "," << deadband_velocities_.at(1) << ","
//      << deadband_velocities_.at(2) << "]");
}

template <typename MapView>
void VelocityDeadbandCritic<MapView>::score(CriticData & data, const MapView& map_view)
{
  using xt::evaluation_strategy::immediate;

  if (!this->enabled_) {
    return;
  }

  auto & vx = data.state.vx;
  auto & wz = data.state.wz;

  if (data.motion_model->isHolonomic()) {
    auto & vy = data.state.vy;
    if (power_ > 1u) {
      data.costs += xt::pow(
        xt::sum(
          std::move(
            xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
            xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0) +
            xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0)) *
          data.model_dt,
          {1}, immediate) *
        weight_,
        power_);
    } else {
      data.costs += xt::sum(
        (std::move(
          xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
          xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0) +
          xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0))) *
        data.model_dt,
        {1}, immediate) *
        weight_;
    }
    return;
  }

  if (power_ > 1u) {
    data.costs += xt::pow(
      xt::sum(
        std::move(
          xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
          xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0)) *
        data.model_dt,
        {1}, immediate) *
      weight_,
      power_);
  } else {
    data.costs += xt::sum(
      (std::move(
        xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
        xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0))) *
      data.model_dt,
      {1}, immediate) *
      weight_;
  }
  return;
}

}  // namespace mppi::critics

