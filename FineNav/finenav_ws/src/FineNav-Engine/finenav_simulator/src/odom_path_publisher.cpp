#include "finenav_simulator/odom_path_publisher.hpp"

#include <string>

namespace finenav_2d
{

OdomPathPublisher::OdomPathPublisher(rclcpp::Node * node, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster)
: node_(node), tf_broadcaster_(tf_broadcaster)
{
  input_odom_topic_ = node_->declare_parameter("input_odom_topic", "/odom");
  path_topic_ = node_->declare_parameter("path_topic", "/path");
  max_path_points_ = node_->declare_parameter("max_path_points", 2000);
  publish_tf_ = node_->declare_parameter("publish_tf", true);
  path_odom_frame_ = node_->declare_parameter("path_odom_frame", "");
  path_base_frame_ = node_->declare_parameter("path_base_frame", "");

  odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
    std::bind(&OdomPathPublisher::OdomCallback, this, std::placeholders::_1));

  path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
}

void OdomPathPublisher::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const std::string odom_frame = path_odom_frame_.empty() ? msg->header.frame_id : path_odom_frame_;
  const std::string base_frame = path_base_frame_.empty() ? msg->child_frame_id : path_base_frame_;

  if (odom_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "Received odometry without a frame_id, skipping path update.");
    return;
  }

  if (publish_tf_ && base_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "Received odometry without a child_frame_id, skipping TF publication.");
  }

  if (publish_tf_ && !base_frame.empty()) {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = msg->header;
    odom_tf.header.frame_id = odom_frame;
    odom_tf.child_frame_id = base_frame;
    odom_tf.transform.translation.x = msg->pose.pose.position.x;
    odom_tf.transform.translation.y = msg->pose.pose.position.y;
    odom_tf.transform.translation.z = msg->pose.pose.position.z;
    odom_tf.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
  }

  if (!path_.header.frame_id.empty() && path_.header.frame_id != odom_frame) {
    path_.poses.clear();
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.header.frame_id = odom_frame;
  pose_stamped.pose = msg->pose.pose;

  path_.header = msg->header;
  path_.header.frame_id = odom_frame;
  path_.poses.push_back(pose_stamped);
  this->TrimPath();

  path_publisher_->publish(path_);
}

void OdomPathPublisher::TrimPath()
{
  if (max_path_points_ <= 0) {
    return;
  }

  if (path_.poses.size() <= static_cast<size_t>(max_path_points_)) {
    return;
  }

  const auto erase_count = path_.poses.size() - static_cast<size_t>(max_path_points_);
  path_.poses.erase(path_.poses.begin(), path_.poses.begin() + static_cast<std::ptrdiff_t>(erase_count));
}

}  // namespace finenav_2d