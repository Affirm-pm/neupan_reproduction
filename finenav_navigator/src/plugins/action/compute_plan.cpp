// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/compute_plan.hpp"

#include <geometry_msgs/msg/pose.hpp>

namespace finenav::navigator {

BT::NodeStatus ComputePlan::onStart() {
    auto* master = config().blackboard->get<finenav::util::TaskMaster*>("_nav_task_master");
    if (!master) return BT::NodeStatus::FAILURE;

    if (!getInput("layer", layer_)) return BT::NodeStatus::FAILURE;

    geometry_msgs::msg::Pose goal;
    std::string planner_id;
    if (!getInput("goal", goal) || !getInput("planner_id", planner_id))
        return BT::NodeStatus::FAILURE;

    master->start(layer_);
    last_goal_ = goal;
    last_planner_id_ = planner_id;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputePlan::onRunning() {
    using Status = finenav_interfaces::msg::TaskFeedback;

    auto* master = config().blackboard->get<finenav::util::TaskMaster*>("_nav_task_master");
    if (!master) return BT::NodeStatus::FAILURE;

    geometry_msgs::msg::Pose goal;
    std::string planner_id;
    if (!getInput("goal", goal) || !getInput("planner_id", planner_id))
        return BT::NodeStatus::FAILURE;

    auto fb = master->feedback(layer_);

    // Task completed or failed
    if (fb && fb->status == Status::COMPLETED) return BT::NodeStatus::SUCCESS;
    if (fb && fb->status == Status::FAILED)    return BT::NodeStatus::FAILURE;

    // Goal changed — restart the task
    if (taskChanged(goal, planner_id)) {
        master->start(layer_);
        last_goal_ = goal;
        last_planner_id_ = planner_id;
    }

    return BT::NodeStatus::RUNNING;
}

bool ComputePlan::taskChanged(const geometry_msgs::msg::Pose& goal,
                               const std::string& planner_id) const {
    if (planner_id != last_planner_id_) return true;
    return goal.position.x != last_goal_.position.x ||
           goal.position.y != last_goal_.position.y ||
           goal.position.z != last_goal_.position.z ||
           goal.orientation.x != last_goal_.orientation.x ||
           goal.orientation.y != last_goal_.orientation.y ||
           goal.orientation.z != last_goal_.orientation.z ||
           goal.orientation.w != last_goal_.orientation.w;
}

}  // namespace finenav::navigator

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::ComputePlan>("ComputePlan");
}
