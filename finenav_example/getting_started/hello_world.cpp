// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "rclcpp/rclcpp.hpp"
#include "finenav_engine/finenav_engine.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("hello_world");

    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine initialized successfully.");
    rclcpp::shutdown();
    return 0;
}
