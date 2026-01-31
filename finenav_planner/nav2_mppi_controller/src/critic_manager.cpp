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

#include "nav2_mppi_controller/critic_manager.hpp"

namespace mppi
{

template <typename MapView>
void CriticManager<MapView>::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name, const std::vector<std::unique_ptr<critics::CriticFunction<MapView>>> & critics)
{
    parent_ = parent;
    name_ = name;
    auto node = parent_.lock();
    logger_ = node->get_logger();
    critics_ = std::move(critics);

//  getParams();   //TODO:参数加载
}

//void CriticManager::getParams()
//{
//  auto node = parent_.lock();
//  auto getParam = parameters_handler_->getParamGetter(name_);
//  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
//}


template <typename MapView>
std::string CriticManager<MapView>::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

template <typename MapView>
void CriticManager<MapView>::evalTrajectoriesScores(
  CriticData & data, const MapView& map_view) const
{
  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;
    }
    critics_[q]->score(data,map_view);
  }
}

}  // namespace mppi
