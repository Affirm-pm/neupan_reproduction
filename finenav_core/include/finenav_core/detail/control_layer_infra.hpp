// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_util/ros/node_thread.hpp"
#include "finenav_util/ros/task_channel.hpp"

namespace finenav::core::detail {

struct ControlLayerInfra {
    rclcpp::Node::SharedPtr              pnode;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread;
    std::unique_ptr<finenav::util::TaskWorker> worker;
};

ControlLayerInfra buildControlLayerInfra(
    rclcpp::Node::SharedPtr parent_node,
    const std::string& layer_name,
    const ExecutionPolicyVariant& policy);

}  // namespace finenav::core::detail
