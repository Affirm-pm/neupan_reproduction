// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include "navigator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<finenav::Navigator>(rclcpp::NodeOptions());
    node->initialize();  // TODO: 为了用 node->shared_from_this()，不得不做两阶段初始化，考虑之后用上 rclcpp::LifeCycleNode
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

