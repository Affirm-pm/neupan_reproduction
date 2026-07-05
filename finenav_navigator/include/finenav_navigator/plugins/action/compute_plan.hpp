// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "finenav_interfaces/action/compute_plan.hpp"
#include "finenav_navigator/thirdparty/bt_action_node.hpp"

namespace finenav {

/**
 * @brief A behavior tree ActionNode that requests a plan from finenav_interfaces::action::ComputePlan action server
 */
class ComputePlanAction : public BT::RosActionNode<finenav_interfaces::action::ComputePlan> {
   public:
    ComputePlanAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<geometry_msgs::msg::Pose>("goal", "Target pose for path planning"),
            BT::InputPort<std::string>("planner_id", "ID of the planner to use")
        });
    }

    bool setGoal(Goal& goal) override;

    BT::NodeStatus onResultReceived(const WrappedResult& result) override;

    /**
     * @brief Called each tick while RUNNING. Returns true when the blackboard
     * goal has changed since the last send, triggering an in-place resend
     * (no cancel — relies on ControlLayer's pending/preempt slot).
     */
    bool shouldResendGoal() override;

private:
    geometry_msgs::msg::Pose last_sent_goal_{};

    static bool posesEqual(const geometry_msgs::msg::Pose& a,
                           const geometry_msgs::msg::Pose& b,
                           double eps = 1e-4) // TODO: equal不应该是这样算的
    {
        return std::abs(a.position.x - b.position.x) < eps &&
               std::abs(a.position.y - b.position.y) < eps &&
               std::abs(a.position.z - b.position.z) < eps;
    }
};

}

