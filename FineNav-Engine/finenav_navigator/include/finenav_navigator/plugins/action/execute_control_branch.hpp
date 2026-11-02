// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief ExecuteControlBranch — 有状态地激活并持有一条 control_pipeline 支路。
 *
 * 经输入端口 `pipeline`（PipelineController*，默认 `{@_pipeline}`）取得流水线控制器，
 * 端口 `branch` 为支路名。StatefulActionNode：
 * onStart 里调用 activateBranch(branch)（集合差：停旧−新、起新−旧、共享 stage 保留）
 * 并返回 RUNNING；此后每次 onRunning 轮询该支路全部 stage 的最新 feedback，
 * 按「任一 FAULTED → FAILURE；否则 RUNNING」聚合返回。被外部 halt
 * （如 ReactiveFallback 因 IsGoalReached 成功而切走）时，onHalted 仅在本支路
 * 仍是活跃支路时整管清停 stopAll()（取消/抢占）；若已被 activateBranch 切走
 * （多支路切换，停止已由集合差完成）则 no-op。
 * 未知支路名 → onStart 返回 FAILURE（候选名列日志，日志器取自端口 `node`）。
 *
 * Ports:
 *   branch   (string, 必填)                    支路名。
 *   pipeline (finenav::navigator::PipelineController*, 默认 "{@_pipeline}") 控制流水线。
 *   node     (rclcpp::Node::SharedPtr,         默认 "{@_node}")       仅用于错误日志。
 */
class ExecuteControlBranch : public BT::StatefulActionNode {
public:
    ExecuteControlBranch(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string branch_;
};

}  // namespace finenav::navigator
