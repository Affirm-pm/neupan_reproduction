// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/condition/is_stuck.hpp"

#include <cmath>
#include <behaviortree_cpp/bt_factory.h>

namespace finenav::navigator {

IsStuck::IsStuck(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ref_set_(false) {}

BT::PortsList IsStuck::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Pose>("robot_pose",      "{robot_pose}", "Current robot pose"),
        BT::InputPort<double>("dist_threshold", 0.05, "Minimum displacement [m] within window"),
        BT::InputPort<double>("timeout_s",      5.0,  "Time window [s] for stuck detection"),
    };
}

BT::NodeStatus IsStuck::tick() {
    geometry_msgs::msg::Pose robot_pose;
    if (!getInput("robot_pose", robot_pose)) {
        return BT::NodeStatus::FAILURE;
    }
    double dist_threshold = 0.05;
    double timeout_s      = 5.0;
    getInput("dist_threshold", dist_threshold);
    getInput("timeout_s",      timeout_s);

    const double cur_x = robot_pose.position.x;
    const double cur_y = robot_pose.position.y;
    const auto now = std::chrono::steady_clock::now();

    // BT::ConditionNode::halt() is final — detect "cold start" by checking
    // whether the node was last ticked more than timeout_s ago.
    if (ref_set_) {
        const double gap =
            std::chrono::duration<double>(now - last_tick_time_).count();
        if (gap > timeout_s) {
            ref_set_ = false;
        }
    }
    last_tick_time_ = now;

    if (!ref_set_) {
        ref_x_        = cur_x;
        ref_y_        = cur_y;
        window_start_ = now;
        ref_set_      = true;
        return BT::NodeStatus::FAILURE;
    }

    const double dx    = cur_x - ref_x_;
    const double dy    = cur_y - ref_y_;
    const double moved = std::sqrt(dx * dx + dy * dy);

    if (moved > dist_threshold) {
        ref_x_        = cur_x;
        ref_y_        = cur_y;
        window_start_ = now;
        return BT::NodeStatus::FAILURE;
    }

    const double elapsed =
        std::chrono::duration<double>(now - window_start_).count();
    return (elapsed > timeout_s) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::IsStuck>("IsStuck");
}
