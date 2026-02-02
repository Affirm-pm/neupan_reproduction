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

#include "finenav_mppi_controller/critics/twirling_critic.hpp"

namespace mppi::critics
{

template <typename MapView>
void TwirlingCritic<MapView>::initialize()
{
  // 直接赋默认值，不动态加载
  power_ = 1;
  weight_ = 10.0f;
}

template <typename MapView>
void TwirlingCritic<MapView>::score(CriticData & data, const MapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_)
  {
    return;
  }

  const auto wz = xt::abs(data.state.wz);
  data.costs += xt::pow(xt::mean(wz, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

