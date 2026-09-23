// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief RestartBranch — 整条支路清停重起（升级 / 核选项）。
 *
 * 从黑盒读取 `_pipeline`（PipelineController*），端口 `branch` 为支路名，调用
 * PipelineController::restartBranch()：逆序 STOP 该支路全部 stage，再顺序 START。
 * 该支路同时被记为当前活跃支路。
 *
 * 与 RestartFromControlLayer 的关系：这是**整体**恢复，代价是上游正确的层也要
 * 重来一遍（丢掉其已有产出）。定位不到根因、或定点重启无效时使用。
 *
 * 未知支路名 → ERROR 日志（候选名列日志）+ no-op（restartBranch 对未知支路
 * 天然无副作用），仍返回 SUCCESS。
 *
 * Ports:
 *   branch (string, 必填) 支路名。
 *
 * Blackboard inputs:
 *   _pipeline (finenav::navigator::PipelineController*)
 */
class RestartBranch : public BT::SyncActionNode {
public:
    RestartBranch(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("branch", "Name of the pipeline branch to restart"),
        };
    }

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
