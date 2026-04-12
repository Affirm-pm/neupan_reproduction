#include "finenav_simulator/ground_truth_map_localizer.hpp"

#include <string>

namespace finenav_2d
{

GroundTruthMapLocalizer::GroundTruthMapLocalizer(rclcpp::Node * node, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster)
: node_(node), tf_broadcaster_(tf_broadcaster)
{
  enable_ground_truth_localizer_ = node_->declare_parameter("enable_ground_truth_localizer", false);
  
  if (!enable_ground_truth_localizer_) {
    return;
  }

  ground_truth_topic_ = node_->declare_parameter("ground_truth_topic", "/ground_truth_tf");
  odom_tf_topic_ = node_->declare_parameter("odom_tf_topic", "/ground_truth_odom_tf");
  robot_frame_ = node_->declare_parameter("robot_frame", "fine_robot");
  map_frame_ = node_->declare_parameter("map_frame", "map");
  base_frame_ = node_->declare_parameter("base_frame", "base_link");
  odom_frame_ = node_->declare_parameter("odom_frame", "odom");

  truth_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    ground_truth_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    std::bind(&GroundTruthMapLocalizer::OnGroundTruth, this, std::placeholders::_1));

  odom_tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    odom_tf_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    std::bind(&GroundTruthMapLocalizer::OnOdomTf, this, std::placeholders::_1));
}

void GroundTruthMapLocalizer::OnOdomTf(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  const geometry_msgs::msg::TransformStamped * odom_tf = nullptr;
  for (const auto & transform : msg->transforms) {
    if (transform.child_frame_id == base_frame_) {
      odom_tf = &transform;
      break;
    }
  }

  if (odom_tf == nullptr) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "No odometry transform targeting '%s' found on %s.",
      base_frame_.c_str(), odom_tf_topic_.c_str());
    return;
  }

  tf2::fromMsg(odom_tf->transform, latest_odom_to_base_);
  latest_odom_frame_ = odom_frame_;
  latest_stamp_ = odom_tf->header.stamp;
  has_odom_tf_ = true;

  if (has_ground_truth_tf_ && !map_frame_.empty()) {
    PublishResolvedTransform();
  }
}

void GroundTruthMapLocalizer::OnGroundTruth(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  const geometry_msgs::msg::TransformStamped * robot_tf = nullptr;
  for (const auto & transform : msg->transforms) {
    if (
      transform.child_frame_id == robot_frame_ ||
      transform.child_frame_id == "/" + robot_frame_)
    {
      robot_tf = &transform;
      break;
    }
  }

  if (robot_tf == nullptr) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "No ground-truth transform for '%s' found on %s.",
      robot_frame_.c_str(), ground_truth_topic_.c_str());
    return;
  }

  tf2::fromMsg(robot_tf->transform, latest_map_to_base_link_);
  latest_ground_truth_stamp_ = robot_tf->header.stamp;
  has_ground_truth_tf_ = true;

  if (has_odom_tf_ && !map_frame_.empty()) {
    PublishResolvedTransform();
  }
}

void GroundTruthMapLocalizer::PublishResolvedTransform()
{
  const tf2::Transform map_to_odom = latest_map_to_base_link_ * latest_odom_to_base_.inverse();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = latest_ground_truth_stamp_.nanoseconds() > 0 ?
    latest_ground_truth_stamp_ : latest_stamp_;
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform = tf2::toMsg(map_to_odom);
  tf_broadcaster_->sendTransform(transform);
  has_map_to_odom_solution_ = true;
}

}  // namespace finenav_2d