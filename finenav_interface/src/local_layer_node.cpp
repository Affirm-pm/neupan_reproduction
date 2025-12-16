// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "simple_local_layer.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<finenav::SimpleLocalLayer>();
    node->initialize(); // TODO: 为了用 node->shared_from_this()，不得不做两阶段初始化，考虑之后用上 rclcpp::LifeCycleNode，在此之前要调查 Bond 与 IPC 的适配性问题（Nav2在做）
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
