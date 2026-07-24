// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/compute_plan.hpp"

#include <geometry_msgs/msg/pose.hpp>

namespace finenav::navigator {

BT::NodeStatus ComputePlan::tick() {
    using CmdMsg = finenav_interfaces::msg::TaskCmd;
    using Status = finenav_interfaces::msg::TaskFeedback;

    auto* master = config().blackboard->get<finenav::util::TaskMaster*>("_nav_task_master");
    if (!master) return BT::NodeStatus::FAILURE;

    std::string layer;
    if (!getInput("layer", layer)) return BT::NodeStatus::FAILURE;

    auto fb = master->feedback(layer);

    geometry_msgs::msg::Pose goal;
    std::string planner_id;
    if (!getInput("goal", goal) || !getInput("planner_id", planner_id))
        return BT::NodeStatus::FAILURE;

    // 1. Already running — check completion / task change
    if (fb && fb->status == Status::RUNNING) {
        if (taskChanged(goal, planner_id)) {
            CmdMsg cmd;
            cmd.goal = goal;
            cmd.planner_id = planner_id;
            master->start(layer, std::move(cmd));
            last_goal_ = goal;
            last_planner_id_ = planner_id;
        }
        return BT::NodeStatus::RUNNING;
    }

    // 2. Terminal states
    if (fb && fb->status == Status::COMPLETED) return BT::NodeStatus::SUCCESS;
    if (fb && fb->status == Status::FAILED)    return BT::NodeStatus::FAILURE;

    // 3. First tick — send goal
    CmdMsg cmd;
    cmd.goal = goal;
    cmd.planner_id = planner_id;
    master->start(layer, std::move(cmd));
    last_goal_ = goal;
    last_planner_id_ = planner_id;
    return BT::NodeStatus::RUNNING;
}

bool ComputePlan::taskChanged(const geometry_msgs::msg::Pose& goal,
                               const std::string& planner_id) const {
    if (planner_id != last_planner_id_) return true;
    return goal.position.x != last_goal_.position.x || // TODO： 会不会有浮点数不安全？
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
