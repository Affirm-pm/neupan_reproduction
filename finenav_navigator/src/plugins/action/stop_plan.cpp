// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/stop_plan.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include "finenav_util/ros/task_channel.hpp"

namespace finenav::navigator {

BT::NodeStatus StopPlan::tick() {
    auto* master = config().blackboard->get<finenav::util::TaskMaster*>("_nav_task_master");
    if (!master) return BT::NodeStatus::FAILURE;

    std::string layer;
    if (!getInput("layer", layer)) return BT::NodeStatus::FAILURE;

    master->stop(layer);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::StopPlan>("StopPlan");
}
