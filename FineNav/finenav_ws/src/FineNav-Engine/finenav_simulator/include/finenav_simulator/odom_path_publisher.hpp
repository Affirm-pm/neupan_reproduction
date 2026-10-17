#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace finenav_2d
{

class OdomPathPublisher
{
public:
  OdomPathPublisher(rclcpp::Node * node, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void TrimPath();

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string input_odom_topic_;
  std::string path_topic_;
  std::string path_odom_frame_;
  std::string path_base_frame_;
  int max_path_points_;
  bool publish_tf_;
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

}  // namespace finenav_2d