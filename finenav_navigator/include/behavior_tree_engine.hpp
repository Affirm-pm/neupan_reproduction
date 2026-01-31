// Copyright (c) 2026.bt_action_node.hpp
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace finenav {

class BehaviorTreeEngine {
  public:
    explicit BehaviorTreeEngine(rclcpp::Node::SharedPtr node); //const std::vector<std::string> & plugin_libraries,
    virtual ~BehaviorTreeEngine() = default;

    void registerPlugins(const std::vector<std::string>& plugins);

    void registerBehaviorTrees(const std::vector<std::string>& tree_dirs);

    BT::BehaviorTreeFactory& factory();

  private:
    rclcpp::Node::WeakPtr node_;
    BT::BehaviorTreeFactory factory_;
    BT::RosNodeParams ros_params_;

};


}