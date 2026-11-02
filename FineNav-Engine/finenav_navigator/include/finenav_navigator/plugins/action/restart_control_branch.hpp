// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief RestartControlBranch — 整条支路清停重起（升级 / 核选项）。
 *
 * 经输入端口 `pipeline`（PipelineController*，默认 `{@_pipeline}`）取得控制器，
 * 端口 `branch` 为支路名，调用
 * PipelineController::restartBranch()：逆序 STOP 该支路全部 stage，再顺序 START。
 * 该支路同时被记为当前活跃支路。
 *
 * 与 RestartFromControlLayer 的关系：这是**整体**恢复，代价是上游正确的层也要
 * 重来一遍（丢掉其已有产出）。定位不到根因、或定点重启无效时使用。
 *
 * 未知支路名 → ERROR 日志（候选名列日志，日志器取自端口 `node`）+ no-op
 * （restartBranch 对未知支路天然无副作用），仍返回 SUCCESS。
 *
 * Ports:
 *   branch   (string, 必填)                    支路名。
 *   pipeline (finenav::navigator::PipelineController*, 默认 "{@_pipeline}") 控制流水线。
 *   node     (rclcpp::Node::SharedPtr,         默认 "{@_node}")       仅用于错误日志。
 */
class RestartControlBranch : public BT::SyncActionNode {
public:
    RestartControlBranch(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
