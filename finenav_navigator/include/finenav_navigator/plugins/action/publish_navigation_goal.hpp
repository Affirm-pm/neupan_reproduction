// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief PublishNavigationGoal — 开始导航：把 goal_poses[] 注入控制流水线根。
 *
 * 从黑盒读取 `goal_poses`（std::vector<geometry_msgs::msg::Pose>）与
 * `_pipeline`（PipelineController*），调用 publishNavigationGoal()。
 * 无端口，直接读黑盒（避免 vector<Pose> 的端口类型注册）。
 */
class PublishNavigationGoal : public BT::SyncActionNode {
public:
    PublishNavigationGoal(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
