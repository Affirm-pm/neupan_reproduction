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

void ConstraintCritic::initialize()
{
  // 以前通过参数服务器加载的参数，改为直接赋固定值（无需动态加载）
  power_ = 1;              // 原始默认值：1
  weight_ = 4.0f;          // 原始默认值：4.0
  enabled_ = true;
  // 父节点参数的默认值，直接赋值（原注释默认值）
  float vx_max = 0.5f;
  float vy_max = 0.0f;
  float vx_min = -0.35f;   // TODO：如需调整，修改此处常量

  const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);

    std::cout<<"[ContraintCritic] Critic initialized"<<std::endl;
}

void ConstraintCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  std::cout<<"[ConstrainCritic]: Begin to score"<<std::endl;
    std::cout<<"[ConstrainCritic]: Weight = "<< weight_ <<std::endl;
  if (!this->enabled_) {
    return;
  }
  auto sgn = xt::where(data.state.vx > 0.0, 1.0, -1.0);
  auto vel_total = sgn * xt::sqrt(data.state.vx * data.state.vx + data.state.vy * data.state.vy);
  auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);
  auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);

  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    auto out_of_turning_rad_motion = xt::maximum(
      acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0);

    data.costs += xt::pow(
      xt::sum(
        (std::move(out_of_max_bounds_motion) +
        std::move(out_of_min_bounds_motion) +
        std::move(out_of_turning_rad_motion)) *
        data.model_dt, {1}, immediate) * weight_, power_);
      std::cout<<"[ConstraintCritic]: Scored"<<std::endl;
    return;
  }

  data.costs += xt::pow(
    xt::sum(
      (std::move(out_of_max_bounds_motion) +
      std::move(out_of_min_bounds_motion)) *
      data.model_dt, {1}, immediate) * weight_, power_);
    std::cout<<"[ConstraintCritic]: Scored"<<std::endl;
}

}  // namespace mppi::critics

