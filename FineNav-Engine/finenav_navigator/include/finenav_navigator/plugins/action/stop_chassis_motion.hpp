// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <behaviortree_cpp/action_node.h>

namespace finenav::navigator {

/**
 * @brief StopChassisMotion — 立即停车（下发零速度）。
 *
 * 经输入端口 `chassis_cmd`（IChassisCommand*，默认 `{@_chassis_cmd}`）取得底盘指令
 * 接口并调用 stop()。端口缺失或为空 → FAILURE；否则 SUCCESS。
 *
 * Ports:
 *   chassis_cmd (finenav::navigator::IChassisCommand*, 默认 "{@_chassis_cmd}") 底盘指令接口。
 */
class StopChassisMotion : public BT::SyncActionNode {
public:
    StopChassisMotion(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
