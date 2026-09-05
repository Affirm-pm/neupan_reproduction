// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/publish_navigation_goal.hpp"

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include "finenav_navigator/pipeline_controller.hpp"

namespace finenav::navigator {

BT::NodeStatus PublishNavigationGoal::tick() {
    PipelineController* pipeline = nullptr;
    if (!config().blackboard->get("_pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    std::vector<geometry_msgs::msg::Pose> poses;
    if (!config().blackboard->get("goal_poses", poses)) {
        return BT::NodeStatus::FAILURE;
    }

    pipeline->publishNavigationGoal(std::move(poses));
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::PublishNavigationGoal>("PublishNavigationGoal");
}
