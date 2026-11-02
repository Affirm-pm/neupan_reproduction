// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief RestartFromControlLayer — 从指定的 ControlLayer 起重启活跃支路的下游后缀。
 *
 * 经输入端口 `pipeline`（PipelineController*，默认 `{@_pipeline}`）取得控制器，
 * 端口 `control_layer` 为层名，调用 PipelineController::restartFrom()：逆序 STOP
 * 再顺序 START [control_layer .. leaf]，上游前缀保持运行。
 *
 * 与 RestartControlBranch 的关系：这是**定点**恢复，只动故障层及其下游；RestartControlBranch
 * 是整条支路清停重起的「核选项」，用于连根因都难以定位的情况。故障沿管线向下游
 * 传播，故「最上游的故障层」即根因 —— 通常与 GetFaultedControlLayer 配对使用。
 *
 * 层名不在活跃支路中 → ERROR 日志 + no-op（仍返回 SUCCESS，见 restartFrom 契约）。
 *
 * Ports:
 *   control_layer (string, 必填) 重启起点层名（含）。
 *   pipeline      (finenav::navigator::PipelineController*, 默认 "{@_pipeline}") 控制流水线。
 */
class RestartFromControlLayer : public BT::SyncActionNode {
public:
    RestartFromControlLayer(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
