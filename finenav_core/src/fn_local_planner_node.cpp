// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "fn_core/simple_test_planner.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<finenav::SimpleTestLocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
