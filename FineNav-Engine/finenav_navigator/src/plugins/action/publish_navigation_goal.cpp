// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/publish_navigation_goal.hpp"

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include "finenav_navigator/pipeline_controller.hpp"

namespace finenav::navigator {

BT::PortsList PublishNavigationGoal::providedPorts() {
    return {
        BT::InputPort<PipelineController*>("pipeline", "{@_pipeline}",
            "PipelineController to publish the navigation goal to"),
        BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("goal_poses", "{@goal_poses}",
            "Goal poses injected into the control pipeline root"),
    };
}

BT::NodeStatus PublishNavigationGoal::tick() {
    PipelineController* pipeline = nullptr;
    if (!getInput("pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    std::vector<geometry_msgs::msg::Pose> poses;
    if (!getInput("goal_poses", poses)) {
        return BT::NodeStatus::FAILURE;
    }

    pipeline->publishNavigationGoal(std::move(poses));
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::PublishNavigationGoal>("PublishNavigationGoal");
}
