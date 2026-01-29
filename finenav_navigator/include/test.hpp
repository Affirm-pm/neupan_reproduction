// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "navigator.hpp"

namespace finenav {

class TestNode : public rclcpp::Node {
  public:
    explicit TestNode(const rclcpp::NodeOptions & options) : Node("test_node", options) {
        std::cout << "[TestNode] Construct" << std::endl;
    }
};

}