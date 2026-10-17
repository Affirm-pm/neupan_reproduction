// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace finenav::navigator {

/**
 * @brief KeepForDuration — 子节点连续返回 SUCCESS 达到指定时长后，本节点才返回 SUCCESS。
 *
 * 作为 DecoratorNode，每次 tick 会将子节点 tick 一次：
 *   - 子节点 SUCCESS  → 若计时未开始则启动计时；未到时返回 RUNNING，到时返回 SUCCESS
 *   - 子节点 FAILURE  → 重置计时，返回 FAILURE
 *   - 子节点 RUNNING  → 重置计时，返回 RUNNING（子节点通常不应 RUNNING）
 *
 * halt() 会重置内部计时器。
 *
 * 输入端口:
 *   - duration_sec (double, 默认 10.0): 子节点需连续 SUCCESS 的时长 [s]
 *
 * Usage in XML:
 * @code
 * <!-- 在 point_b 附近 3m 内持续 10 秒 → 认为到达（point_b 由用户显式提供） -->
 * <KeepForDuration duration_sec="10.0">
 *     <IsNearPoint point_a="{robot_pose}" point_b="{some_pose}" distance_threshold="3.0"/>
 * </KeepForDuration>
 * @endcode
 */
class KeepForDuration : public BT::DecoratorNode {
public:
    KeepForDuration(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("duration_sec", 10.0, "Required duration of continuous SUCCESS [s]"),
        };
    }

    BT::NodeStatus tick() override;
    void halt() override;

private:
    bool timer_running_{false};
    std::chrono::steady_clock::time_point timer_start_;
};

}  // namespace finenav
