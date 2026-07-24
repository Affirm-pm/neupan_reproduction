// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <memory>
#include <string>

#include "finenav_navigator/plugins/decorator/dynamic_goal_follower.hpp"

namespace finenav::navigator
{

DynamicGoalFollower::DynamicGoalFollower(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::DecoratorNode(name, conf)
{
}

void DynamicGoalFollower::initialize()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("_node");
  if (!node) {
    throw BT::RuntimeError("DynamicGoalFollower: _node not found on blackboard.");
  }

  std::string topic_name = "/target_pose";
  getInput("topic_name", topic_name);

  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor_.add_callback_group(
    callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_name,
    rclcpp::QoS(1),
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      onTargetPose(msg);
    },
    options);
}

BT::NodeStatus DynamicGoalFollower::tick()
{
  // 首次进入或重新激活时初始化订阅
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  // 处理积压的 ROS 回调，获取最新目标位姿
  callback_executor_.spin_some();

  // 从 blackboard 获取机器人当前位姿
  geometry_msgs::msg::Pose robot_pose;
  getInput("robot_pose", robot_pose);

  // 如果有新目标且能获取机器人位姿，计算跟随目标
  if (last_msg_received_) {
    const auto follow_goal = computeFollowGoal(robot_pose, last_msg_.pose);

    if (shouldUpdate(follow_goal)) {
      setOutput("goal", follow_goal);
      last_goal_ = follow_goal;
      has_last_goal_ = true;
    }
  }

  // 透传子节点状态
  setStatus(BT::NodeStatus::RUNNING);
  const BT::NodeStatus child_status = child_node_->executeTick();
  return child_status;
}

void DynamicGoalFollower::halt()
{
  // 重置状态，但保留订阅（下次 tick 会重新激活）
  last_msg_received_ = false;
  has_last_goal_ = false;
  DecoratorNode::halt();
}

geometry_msgs::msg::Pose DynamicGoalFollower::computeFollowGoal(
  const geometry_msgs::msg::Pose & robot_pose,
  const geometry_msgs::msg::Pose & target_pose) const
{
  auto goal = target_pose;

  double follow_distance = 0.0;
  getInput("follow_distance", follow_distance);
  follow_distance = std::max(0.0, follow_distance);

  const double dx = target_pose.position.x - robot_pose.position.x;
  const double dy = target_pose.position.y - robot_pose.position.y;
  const double dist = std::hypot(dx, dy);

  // 如果设置了跟随距离且距离大于阈值，将目标沿 robot→target 方向回退
  if (follow_distance > 0.0 && dist > follow_distance) {
    const double scale = (dist - follow_distance) / dist;
    goal.position.x = robot_pose.position.x + dx * scale;
    goal.position.y = robot_pose.position.y + dy * scale;
  }

  // 让目标朝向指向原目标方向
  const double yaw = std::atan2(dy, dx);
  goal.orientation.w = std::cos(0.5 * yaw);
  goal.orientation.x = 0.0;
  goal.orientation.y = 0.0;
  goal.orientation.z = std::sin(0.5 * yaw);

  return goal;
}

bool DynamicGoalFollower::shouldUpdate(
  const geometry_msgs::msg::Pose & new_goal) const
{
  if (!has_last_goal_) {
    return true;
  }

  double update_distance = 0.05;
  getInput("update_distance", update_distance);
  update_distance = std::max(0.0, update_distance);

  const double dx = new_goal.position.x - last_goal_.position.x;
  const double dy = new_goal.position.y - last_goal_.position.y;
  const double dz = new_goal.position.z - last_goal_.position.z;
  return (dx * dx + dy * dy + dz * dz) >= update_distance * update_distance;
}

void DynamicGoalFollower::onTargetPose(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  last_msg_ = *msg;
  last_msg_received_ = true;
}

}  // namespace finenav::navigator

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::DynamicGoalFollower>("DynamicGoalFollower");
}
