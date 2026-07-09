// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_engine/finenav_engine.hpp"

#include <chrono>

namespace finenav {
namespace {
using NavAction = finenav_interfaces::action::NavigateToPose;
}  // namespace

// ══════════════════════════════════════════════════════════════════════════════
//  Constructor
// ══════════════════════════════════════════════════════════════════════════════

FineNavEngine::FineNavEngine(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node) {
    FINENAV_LOG_INFO(node_->get_logger(), "Initializing...");

    localizer_ = std::make_unique<finenav::localizer::FineNavLocalizer>(node_);
    FINENAV_LOG_INFO(node_->get_logger(), "Localizer ready.");

    navigator_ = std::make_unique<finenav::navigator::BtNavigator>(node_, localizer_.get());
    FINENAV_LOG_INFO(node_->get_logger(), "Navigator ready.");

    nav_client_ = rclcpp_action::create_client<NavAction>(
        node_, navigator_->getActionName());
    FINENAV_LOG_INFO(node_->get_logger(), "Navigation client ready.");

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped& msg) {
            navigateTo(msg.pose);
        });
    FINENAV_LOG_INFO(node_->get_logger(), "Goal pose subscriber ready (/goal_pose).");

    FINENAV_LOG_INFO(node_->get_logger(), "All singletons initialized.");
}

// ══════════════════════════════════════════════════════════════════════════════
//  MapServer removal
// ══════════════════════════════════════════════════════════════════════════════

bool FineNavEngine::removeMapServer(const std::string& name) {
    return maps_.erase(name) > 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  ControlLayer removal
// ══════════════════════════════════════════════════════════════════════════════

bool FineNavEngine::removeControlLayer(const std::string& name) {
    return layers_.erase(name) > 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Navigation control
// ══════════════════════════════════════════════════════════════════════════════

void FineNavEngine::navigateTo(const geometry_msgs::msg::Pose& goal,
                                const std::string& bt_id) {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "Navigator action server not available — cannot start navigation.");
        return;
    }

    auto goal_msg = NavAction::Goal();
    goal_msg.pose.pose         = goal;
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.behavior_tree     = bt_id;

    nav_client_->async_send_goal(goal_msg);
    FINENAV_LOG_INFO(node_->get_logger(),
        "Navigation goal sent: (%.2f, %.2f, %.2f)",
        goal.position.x, goal.position.y, goal.position.z);
}

void FineNavEngine::cancelNavigation() {
    nav_client_->async_cancel_all_goals();
    FINENAV_LOG_INFO(node_->get_logger(), "Navigation cancelled.");
}

}  // namespace finenav
