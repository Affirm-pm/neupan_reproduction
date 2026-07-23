// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/compute_plan.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

BT::NodeStatus ComputePlan::tick() {
    using CmdMsg = finenav_interfaces::msg::TaskCmd;
    using Status = finenav_interfaces::msg::TaskFeedback;

    auto* master = config().blackboard->get<finenav::util::TaskMaster*>("_nav_task_master");
    if (!master) return BT::NodeStatus::FAILURE;

    std::string layer;
    if (!getInput("layer", layer)) return BT::NodeStatus::FAILURE;

    auto fb = master->feedback(layer);

    // 1. Already running — check completion / goal change
    if (fb && fb->status == Status::RUNNING) {
        geometry_msgs::msg::Pose current;
        if (!getInput("goal", current)) return BT::NodeStatus::FAILURE;

        if (goalChanged(current)) {
            std::string planner_id;
            getInput("planner_id", planner_id);
            CmdMsg cmd;
            cmd.goal = current;
            cmd.planner_id = planner_id;
            master->start(layer, std::move(cmd));
            last_sent_goal_ = current;
        }
        return BT::NodeStatus::RUNNING;
    }

    // 2. Terminal states
    if (fb && fb->status == Status::COMPLETED) return BT::NodeStatus::SUCCESS;
    if (fb && fb->status == Status::FAILED)    return BT::NodeStatus::FAILURE;

    // 3. First tick — send goal
    geometry_msgs::msg::Pose goal;
    std::string planner_id;
    if (!getInput("goal", goal) || !getInput("planner_id", planner_id))
        return BT::NodeStatus::FAILURE;

    CmdMsg cmd;
    cmd.goal = goal;
    cmd.planner_id = planner_id;
    master->start(layer, std::move(cmd));
    last_sent_goal_ = goal;
    return BT::NodeStatus::RUNNING;
}

bool ComputePlan::goalChanged(const geometry_msgs::msg::Pose& a, double eps) const { // TODO: 不合理的goalChanged检测
    return std::abs(a.position.x - last_sent_goal_.position.x) > eps ||
           std::abs(a.position.y - last_sent_goal_.position.y) > eps ||
           std::abs(a.position.z - last_sent_goal_.position.z) > eps;
}

}  // namespace finenav::navigator

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::ComputePlan>("ComputePlan");
}
