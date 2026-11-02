// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief PublishNavigationGoal — 开始导航：把 goal_poses[] 注入控制流水线根。
 *
 * 经输入端口读取 `goal_poses`（std::vector<geometry_msgs::msg::Pose>，默认
 * `{@goal_poses}`）与 `pipeline`（PipelineController*，默认 `{@_pipeline}`），
 * 调用 publishNavigationGoal()。
 *
 * Ports:
 *   goal_poses (std::vector<geometry_msgs::msg::Pose>, 默认 "{@goal_poses}") 目标位姿序列。
 *   pipeline   (finenav::navigator::PipelineController*,  默认 "{@_pipeline}") 控制流水线。
 */
class PublishNavigationGoal : public BT::SyncActionNode {
public:
    PublishNavigationGoal(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
