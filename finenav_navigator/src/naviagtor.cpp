// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "navigator.hpp"

namespace finenav {

Navigator::Navigator(const rclcpp::NodeOptions & options) : Node("finenav_navigator", options) {}

void Navigator::initialize() {

    // Read parameters
    param_listener_ = std::make_unique<finenav_navigator::ParamListener>(shared_from_this());
    params_ = param_listener_->get_params();

    // Create Action Server
    action_server_ = std::make_unique<ActionServer>(
        shared_from_this(),
        params_.action_name,
        std::bind(&Navigator::execute, this)
    );

    // Initialize Behavior Tree
    bt_engine_ = std::make_unique<BehaviorTreeEngine>(shared_from_this());
    if (!params_.plugins.empty()) { bt_engine_->registerPlugins(params_.plugins); }
    if (!params_.behavior_trees.empty()) { bt_engine_->registerBehaviorTrees(params_.behavior_trees); }
}

void Navigator::execute() {
    RCLCPP_INFO(this->get_logger(), "Navigator executing action...");

    auto goal = action_server_->get_current_goal();

    try {
        tree_ = bt_engine_->factory().createTree(goal->behavior_tree);
    } catch (std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "Failed to create tree [%s]: %s", goal->behavior_tree.c_str(), ex.what());
    }

    auto blackboard = tree_.rootBlackboard();
    blackboard->set("goal", goal->pose.pose);

    rclcpp::WallRate loop_rate(params_.tick_frequency);
    RCLCPP_INFO(get_logger(), "Starting navigation to (%.2f, %.2f)",  goal->pose.pose.position.x, goal->pose.pose.position.y);

    while (rclcpp::ok()) {

        if (action_server_->is_cancel_requested()) {
            RCLCPP_INFO(get_logger(), "Cancellation requested.");
            tree_.haltTree();
            action_server_->terminate_all(); // TODO: terminate_all 返回的是 abort 而不是 canceled
            return;
        }

        if (action_server_->is_preempt_requested()) {
            RCLCPP_INFO(get_logger(), "Preemption requested. Stopping current tree.");
            tree_.haltTree();
            return; // action_server_ will handle the pending goal in the next loop
        }

        auto status = tree_.tickExactlyOnce();

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(get_logger(), "Navigation succeeded.");
            action_server_->succeeded_current();
            return;
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(get_logger(), "Navigation failed.");
            action_server_->terminate_all(); // 或者 abort_current
            return;

        }

        // --- Feedback ---
        // ...

        loop_rate.sleep();
    }

}


}
