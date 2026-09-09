// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief ExecuteControlPipelineBranch — 有状态地激活并持有一条 control_pipeline 支路。
 *
 * 从黑盒读取 `_pipeline`，端口 `branch` 为支路名。StatefulActionNode：
 * onStart 里调用 restartBranch(branch)（逆序 STOP + 顺序 START，自愈 FAULTED 层）
 * 并返回 RUNNING；此后每次 onRunning 轮询该支路全部 stage 的最新 feedback，
 * 按「任一 FAULTED → FAILURE；否则 RUNNING」聚合返回。被外部 halt
 * （如 ReactiveFallback 因 IsGoalReached 成功而切走）时 onHalted 调用
 * stopBranch(branch) 停掉整条支路。
 * 未知支路名 → onStart 返回 FAILURE（候选名列日志）。
 */
class ExecuteControlPipelineBranch : public BT::StatefulActionNode {
public:
    ExecuteControlPipelineBranch(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("branch", "Name of the pipeline branch to activate")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string branch_;
};

}  // namespace finenav::navigator
