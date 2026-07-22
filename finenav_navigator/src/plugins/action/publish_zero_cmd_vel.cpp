// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/publish_zero_cmd_vel.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace finenav::navigator {

BT::NodeStatus PublishZeroCmdVel::tick() {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("_node");
    if (!node) return BT::NodeStatus::FAILURE;

    std::string topic;
    getInput("topic_name", topic);
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    geometry_msgs::msg::Twist zero;
    pub->publish(zero);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::PublishZeroCmdVel>("PublishZeroCmdVel");
}
