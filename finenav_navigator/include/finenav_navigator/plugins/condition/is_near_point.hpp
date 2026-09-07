// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav::navigator {

/**
 * @brief IsNearPoint — 检查两个位姿点之间的 2D 距离是否小于指定阈值。
 *
 * 适用于"到达终点附近"区域判定，配合 KeepForDuration 实现
 * "在终点附近持续一段时间后认为完成导航"的逻辑。
 *
 * Blackboard 输入:
 *   - point_a (geometry_msgs::msg::Pose): 第一个位姿点（默认 robot_pose）
 *   - point_b (geometry_msgs::msg::Pose): 第二个位姿点（无默认，需由用户显式提供，
 *            例如 {goal_poses} 的某一点 / 地图关键点）
 *
 * 输入端口:
 *   - distance_threshold (double, 默认 3.0 m): 距离判定阈值
 *
 * Usage in XML:
 * @code
 * <!-- 两点均由调用方显式给出（不再隐含依赖单点 {goal}） -->
 * <IsNearPoint point_a="{robot_pose}" point_b="{some_pose}" distance_threshold="3.0"/>
 * @endcode
 */
class IsNearPoint : public BT::ConditionNode {
public:
    IsNearPoint(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}  // namespace finenav::navigator
