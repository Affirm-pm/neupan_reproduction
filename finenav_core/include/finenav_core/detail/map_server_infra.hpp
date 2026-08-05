// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/param_handle_keeper.hpp"
#include "finenav_util/ros/node_thread.hpp"

namespace finenav::core::detail {

struct MapServerInfra {
    rclcpp::Node::SharedPtr              pnode;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread;
    rclcpp::TimerBase::SharedPtr         update_timer;
    rclcpp::TimerBase::SharedPtr         shift_timer;
    ParamHandleKeeper                    param_keeper;
    double                               update_rate_hz  = 10.0;
    double                               shift_rate_hz   = 50.0;
    double                               shift_margin_ratio = 0.5;
};

MapServerInfra buildMapServerInfra(
    rclcpp::Node::SharedPtr parent_node,
    const std::string& server_name,
    double update_rate_hz,
    double shift_rate_hz,
    double shift_margin_ratio);

void declareMapServerParameters(
    rclcpp::Node& pnode,
    double& update_rate_hz,
    double& shift_rate_hz,
    double& shift_margin_ratio);

}  // namespace finenav::core::detail
