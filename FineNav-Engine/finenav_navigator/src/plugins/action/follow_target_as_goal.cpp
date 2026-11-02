// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "finenav_navigator/plugins/action/follow_target_as_goal.hpp"
#include "finenav_navigator/pipeline_controller.hpp"

namespace finenav::navigator
{

FollowTargetAsGoal::FollowTargetAsGoal(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf)
{
}

BT::PortsList FollowTargetAsGoal::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "/target_pose", "Target pose topic"),
    BT::InputPort<geometry_msgs::msg::Pose>("robot_pose", "{@robot_pose}", "Current robot pose"),
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "{@_node}",
      "Node used to create the target pose subscription"),
    BT::InputPort<PipelineController*>("pipeline", "{@_pipeline}",
      "PipelineController re-injecting the follow goal as the navigation goal"),
    BT::OutputPort<std::vector<geometry_msgs::msg::Pose>>(
      "goal_poses", "{@goal_poses}", "Computed follow goal as single-element pose vector"),
    BT::InputPort<double>("follow_distance", 0.0, "Distance behind target to follow [m]"),
    BT::InputPort<double>("update_distance", 0.05, "Min displacement to trigger update [m]"),
  };
}

void FollowTargetAsGoal::initialize()
{
  rclcpp::Node::SharedPtr node;
  if (!getInput("node", node) || !node) {
    throw BT::RuntimeError("FollowTargetAsGoal: _node not found on blackboard.");
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

BT::NodeStatus FollowTargetAsGoal::onStart()
{
  // 首次启动时创建订阅：单独 callback_group + 独立 executor spin_some。
  initialize();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowTargetAsGoal::onRunning()
{
  // 处理积压的 ROS 回调，获取最新目标位姿
  callback_executor_.spin_some();

  // 从输入端口 robot_pose（默认 {@robot_pose}）获取机器人当前位姿
  geometry_msgs::msg::Pose robot_pose{};
  getInput("robot_pose", robot_pose);

  // 如果有新目标且能获取机器人位姿，计算跟随目标
  if (last_msg_received_) {
    const auto follow_goal = computeFollowGoal(robot_pose, last_msg_.pose);

    if (shouldUpdate(follow_goal)) {
      setOutput("goal_poses", std::vector<geometry_msgs::msg::Pose>{follow_goal});
      last_goal_ = follow_goal;
      has_last_goal_ = true;

      // 将跟随点重新注入 _root/__feedback，驱动下游层以新 ref_traj 重新规划。
      // （否则层里存储的 ref_traj 仍是初始静态 goal，机器人不会跟着移动目标走。）
      PipelineController* pipeline = nullptr;
      if (getInput("pipeline", pipeline) && pipeline) {
        pipeline->publishNavigationGoal({follow_goal});
      }
    }
  }

  // 动态跟随不自行结束 —— 始终 RUNNING，直到被外部 halt。
  return BT::NodeStatus::RUNNING;
}

void FollowTargetAsGoal::onHalted()
{
  // 复位订阅状态（保留订阅，节点重启时会重新 initialize 或重新开始接收）
  last_msg_received_ = false;
  has_last_goal_ = false;
}

geometry_msgs::msg::Pose FollowTargetAsGoal::computeFollowGoal(
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

bool FollowTargetAsGoal::shouldUpdate(
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

void FollowTargetAsGoal::onTargetPose(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  last_msg_ = *msg;
  last_msg_received_ = true;
}

}  // namespace finenav::navigator

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::FollowTargetAsGoal>("FollowTargetAsGoal");
}
