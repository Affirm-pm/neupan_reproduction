// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/decorator_node.h"
#include "finenav_navigator/thirdparty/ros_node_params.hpp"

namespace finenav
{

/**
 * @brief DynamicGoalFollower — 动态目标跟随装饰器节点。
 *
 * 该节点订阅一个 PoseStamped 话题获取动态目标位姿，每次 tick 时计算跟随目标点
 * （在 target 后方的 follow_distance 处），写入 blackboard 的 {goal} 端口，
 * 然后 tick 子节点（通常为 ComputePlan），透传子节点状态。
 *
 * 典型用法：包裹 ComputePlan 实现持续动态跟随导航。
 *
 * 输入端口:
 *   - topic_name      (string, 默认 "/target_pose"):  订阅的目标话题名称
 *   - robot_pose      (Pose,   默认 "{robot_pose}"):   当前机器人位姿（从 blackboard 读取）
 *   - follow_distance (double, 默认 0.0):              跟随距离（在目标后方多远停下，单位 m）
 *   - update_distance (double, 默认 0.05):             最小位移阈值，新 goal 与上次距离低于此值时不更新
 *
 * 输出端口:
 *   - goal (Pose):                                      计算出的跟随目标位姿
 *
 * Usage in XML:
 * @code
 * <DynamicGoalFollower topic_name="/target_pose"
 *                      robot_pose="{robot_pose}"
 *                      goal="{goal}"
 *                      follow_distance="1.0"
 *                      update_distance="0.1">
 *     <ComputePlan action_name="/hello_finenav/mppi_layer"
 *                  goal="{goal}"
 *                  planner_id="finenav_mppi_controller"/>
 * </DynamicGoalFollower>
 * @endcode
 */
class DynamicGoalFollower : public BT::DecoratorNode
{
public:
  DynamicGoalFollower(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  ~DynamicGoalFollower() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "/target_pose", "Target pose topic"),
      BT::InputPort<geometry_msgs::msg::Pose>("robot_pose", "{robot_pose}", "Current robot pose"),
      BT::OutputPort<geometry_msgs::msg::Pose>("goal", "Computed follow goal"),
      BT::InputPort<double>("follow_distance", 0.0, "Distance behind target to follow [m]"),
      BT::InputPort<double>("update_distance", 0.05, "Min displacement to trigger update [m]"),
    };
  }

private:
  BT::NodeStatus tick() override;
  void halt() override;

  /**
   * @brief 创建 ROS 订阅，在首次 tick 或重新激活时调用
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
   * @brief 判断是否需要更新 blackboard goal
   * @param new_goal 新计算的跟随目标
   * @return true 若与上次 goal 距离超过 update_distance
   */
  bool shouldUpdate(const geometry_msgs::msg::Pose & new_goal) const;

  /**
   * @brief 话题订阅回调
   */
  void onTargetPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);

  // ROS
  std::weak_ptr<rclcpp::Node> node_;
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

}  // namespace finenav
