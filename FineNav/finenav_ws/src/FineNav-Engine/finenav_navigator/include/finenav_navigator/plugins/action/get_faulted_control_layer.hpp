// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief GetFaultedControlLayer — 找出活跃支路上「最上游」的故障 ControlLayer。
 *
 * 从黑盒读取 `_pipeline`（PipelineController*），取当前活跃支路的 stage 序列
 * （上游→下游顺序），逐个查其最新 feedback，返回第一个 status == FAULTED 的
 * stage 名。
 *
 * 为什么「第一个」就是根因：故障沿管线向下游传播（上游 FAULTED → 下游
 * planRefState() == kUpstreamFaulted → 自己也 FAULTED），因此故障层一定构成
 * 从根因到叶子的一段**后缀**。后缀的头一个元素就是根因。
 *
 * Ports:
 *   control_layer (string, 输出) 最上游故障层名；无故障时保留旧值（BT 不应读取）。
 *
 * Blackboard inputs:
 *   _pipeline (finenav::navigator::PipelineController*)
 *
 * 返回 SUCCESS 当且仅当活跃支路上存在 FAULTED 层。
 * 无活跃支路 / 黑盒缺 _pipeline → FAILURE。
 *
 * 典型用法 —— 用输出端口喂给恢复动作，只重启根因及其下游：
 *   <GetFaultedControlLayer control_layer="{faulted_layer}"/>
 *   <RestartFromControlLayer   control_layer="{faulted_layer}"/>
 */
class GetFaultedControlLayer : public BT::SyncActionNode {
public:
    GetFaultedControlLayer(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<std::string>("control_layer",
                "Name of the most-upstream FAULTED ControlLayer of the active branch"),
        };
    }

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
