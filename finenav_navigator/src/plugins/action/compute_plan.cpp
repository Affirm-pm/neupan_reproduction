// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/compute_plan.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

ComputePlanAction::ComputePlanAction(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosActionNode<finenav_interfaces::action::ComputePlan>(name, conf, params) {}

bool ComputePlanAction::setGoal(Goal& goal) {
    if (!getInput("goal", goal.goal) ||
        !getInput("planner_id", goal.planner_id)) {
        FINENAV_LOG_ERROR(logger(), "ComputePlan: missing input ports");
        return false;
    }
    // Record the goal being sent so shouldResendGoal() can detect changes.
    last_sent_goal_ = goal.goal;
    return true;
}

bool ComputePlanAction::shouldResendGoal() {
    geometry_msgs::msg::Pose current_goal;
    if (!getInput("goal", current_goal)) {
        return false;
    }
    if (!posesEqual(current_goal, last_sent_goal_)) {
        FINENAV_LOG_INFO(logger(),
            "ComputePlan: goal changed ({:.2f},{:.2f})->({:.2f},{:.2f}), resending.",
            last_sent_goal_.position.x, last_sent_goal_.position.y,
            current_goal.position.x,   current_goal.position.y); // TODO： 只输出2维变化，感觉也不太合理
        return true;
    }
    return false;
}

BT::NodeStatus ComputePlanAction::onResultReceived(const WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        std::cout << "ComputePlanAction of "
                  << rclcpp_action::to_string(result.goal_id) << " succeeded." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

}

#include "finenav_navigator/thirdparty/plugins.hpp"
CreateRosNodePlugin(finenav::navigator::ComputePlanAction, "ComputePlan");
