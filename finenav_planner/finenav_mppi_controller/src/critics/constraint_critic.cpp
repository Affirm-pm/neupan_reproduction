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

#include "finenav_mppi_controller/critics/constraint_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void ConstraintCritic::initialize(const finenav_mppi_controller::Params & p)
{
  logger_ = rclcpp::get_logger("ConstraintCritic");
  const auto & c = p.ConstraintCritic;
  enabled_ = c.enabled;
  power_   = static_cast<unsigned int>(c.cost_power);
  weight_  = static_cast<float>(c.cost_weight);

  // Velocity limits come from the top-level optimizer params
  const float vx_max = static_cast<float>(p.vx_max);
  const float vy_max = static_cast<float>(p.vy_max);
  const float vx_min = static_cast<float>(p.vx_min);

  const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);

  RCLCPP_DEBUG(logger_, "Critic initialized");
}

void ConstraintCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_) {
    return;
  }
  const auto sgn = data.state.vx.unaryExpr([](float value) {
      return value > 0.0F ? 1.0F : -1.0F;
    });
  const auto vel_total = sgn * (data.state.vx.square() + data.state.vy.square()).sqrt();
  const auto out_of_max_bounds_motion = (vel_total - max_vel_).max(0.0F);
  const auto out_of_min_bounds_motion = (min_vel_ - vel_total).max(0.0F);

  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    // Match Nav2's #5855 fix: keep the Ackermann radius ratio finite when
    // angular velocity is zero or very small.
    constexpr float epsilon = 1e-6F;
    const auto wz_safe = wz.abs().max(epsilon);
    const auto out_of_turning_rad_motion = (acker->getMinTurningRadius() -
      (vx.abs() / wz_safe)).max(0.0F);

    data.costs += (((out_of_max_bounds_motion + out_of_min_bounds_motion +
      out_of_turning_rad_motion) * data.model_dt).rowwise().sum() * weight_).pow(power_);
      RCLCPP_DEBUG(logger_, "[ConstraintCritic]: Scored");

    return;
  }

  data.costs += (((out_of_max_bounds_motion + out_of_min_bounds_motion) *
    data.model_dt).rowwise().sum() * weight_).pow(power_);
    RCLCPP_DEBUG(logger_, "[ConstraintCritic]: Scored");
}

}  // namespace mppi::critics
