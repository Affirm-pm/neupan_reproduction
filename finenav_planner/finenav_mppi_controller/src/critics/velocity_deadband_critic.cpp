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

#include "finenav_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void VelocityDeadbandCritic::initialize(const finenav_mppi_controller::Params & p)
{
  logger_ = rclcpp::get_logger("VelocityDeadbandCritic");
  const auto & c = p.VelocityDeadbandCritic;
  enabled_ = c.enabled;
  power_   = static_cast<unsigned int>(c.cost_power);
  weight_  = static_cast<float>(c.cost_weight);

  deadband_velocities_.resize(3);
  const auto & dv = c.deadband_velocities;
  for (size_t i = 0; i < 3 && i < dv.size(); ++i) {
    deadband_velocities_[i] = static_cast<float>(dv[i]);
  }
//
//  RCLCPP_INFO_STREAM(
//    logger_, "VelocityDeadbandCritic instantiated with "
//      << power_ << " power, " << weight_ << " weight, deadband_velocity ["
//      << deadband_velocities_.at(0) << "," << deadband_velocities_.at(1) << ","
//      << deadband_velocities_.at(2) << "]");
}


void VelocityDeadbandCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_) {
    return;
  }

  auto & vx = data.state.vx;
  auto & wz = data.state.wz;
  const auto vx_penalty = (std::fabs(deadband_velocities_.at(0)) - vx.abs()).max(0.0F);
  const auto wz_penalty = (std::fabs(deadband_velocities_.at(2)) - wz.abs()).max(0.0F);

  if (data.motion_model->isHolonomic()) {
    auto & vy = data.state.vy;
    const auto vy_penalty = (std::fabs(deadband_velocities_.at(1)) - vy.abs()).max(0.0F);
    const auto total_penalty = vx_penalty + vy_penalty + wz_penalty;
    if (power_ > 1u) {
      data.costs += ((total_penalty * data.model_dt).rowwise().sum() * weight_).pow(power_);
    } else {
      data.costs += (total_penalty * data.model_dt).rowwise().sum() * weight_;
    }
    return;
  }

  if (power_ > 1u) {
    data.costs += (((vx_penalty + wz_penalty) * data.model_dt).rowwise().sum() * weight_).pow(power_);
  } else {
    data.costs += ((vx_penalty + wz_penalty) * data.model_dt).rowwise().sum() * weight_;
  }
  return;
}

}  // namespace mppi::critics
