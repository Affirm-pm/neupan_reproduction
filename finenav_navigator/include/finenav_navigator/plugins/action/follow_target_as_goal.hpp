// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/action_node.h"

namespace finenav::navigator
{

/**
 * @brief FollowTargetAsGoal — 动态目标跟随 Action 节点。
 *
 * StatefulActionNode：onStart 创建 /target_pose 订阅（独立 callback_group +
 * 独立 executor spin_some），此后每次 onRunning 处理 ROS 回调并计算跟随目标点
 * （在 target 后方的 follow_distance 处），把单元素向量写入 blackboard 的
 * {goal_poses}，同时经 _pipeline 的 publishNavigationGoal({follow_goal})
 * 重新注入 _root/__feedback，驱动下游控制层以新的 ref_traj 重新规划。
 * 本节点始终返回 RUNNING；被 halt 时通过 onHalted 复位订阅状态。
 *
 * 典型用法：与 ExecuteControlPipelineBranch 放在 ReactiveSequence 中并发执行，
 * 由 ExecuteControlPipelineBranch 激活整条支路、本节点持续喂入最新跟随目标。
 *
 * 输入端口:
 *   - topic_name      (string, 默认 "/target_pose"):  订阅的目标话题名称
 *   - robot_pose      (Pose,   默认 "{robot_pose}"):   当前机器人位姿（从 blackboard 读取）
 *   - follow_distance (double, 默认 0.0):              跟随距离（在目标后方多远停下，单位 m）
 *   - update_distance (double, 默认 0.05):             最小位移阈值，新 goal 与上次距离低于此值时不更新
 *
 * 输出端口:
 *   - goal_poses (std::vector<geometry_msgs::msg::Pose>, 默认 "{goal_poses}"):
 *                 计算出的跟随目标（单元素向量），供 IsGoalReached 等条件节点
 *                 读取，作为"终点"（.back()）判定。
 *
 * Usage in XML:
 * @code
 * <FollowTargetAsGoal topic_name="/target_pose"
 *                     robot_pose="{robot_pose}"
 *                     goal_poses="{goal_poses}"
 *                     follow_distance="0.5"
 *                     update_distance="0.1"/>
 * @endcode
 */
class FollowTargetAsGoal : public BT::StatefulActionNode
{
public:
  FollowTargetAsGoal(
    const std::string & name,
    const BT::NodeConfig & conf);

  ~FollowTargetAsGoal() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /**
   * @brief 创建 ROS 订阅（独立 callback_group + 独立 executor），在 onStart 时调用
   */
  void initialize();

  /**
   * @brief 计算跟随目标点位姿
   * @param robot_pose  当前机器人位姿
   * @param target_pose 话题接收到的目标位姿
   * @return 偏移后的跟随目标位姿
   */
  geometry_msgs::msg::Pose computeFollowGoal(
    const geometry_msgs::msg::Pose & robot_pose,
    const geometry_msgs::msg::Pose & target_pose) const;

  /**
   * @brief 判断是否需要更新 blackboard goal_poses
   * @param new_goal 新计算的跟随目标
   * @return true 若与上次 goal 距离超过 update_distance
   */
  bool shouldUpdate(const geometry_msgs::msg::Pose & new_goal) const;

  /**
   * @brief 话题订阅回调
   */
  void onTargetPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;

  // 最近一次接收到的原始目标消息
  geometry_msgs::msg::PoseStamped last_msg_;
  bool last_msg_received_{false};

  // 上次写入 blackboard 的 goal（用于 update_distance 抖动抑制）
  geometry_msgs::msg::Pose last_goal_;
  bool has_last_goal_{false};
};

}  // namespace finenav::navigator
