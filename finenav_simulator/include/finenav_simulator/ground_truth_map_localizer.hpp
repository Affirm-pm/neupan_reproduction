#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace finenav_2d
{

class GroundTruthMapLocalizer
{
public:
  GroundTruthMapLocalizer(rclcpp::Node * node, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

private:
  void OnOdomTf(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void OnGroundTruth(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void PublishResolvedTransform();

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool enable_ground_truth_localizer_;
  std::string ground_truth_topic_;
  std::string odom_tf_topic_;
  std::string robot_frame_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string latest_odom_frame_;
  bool has_odom_tf_{false};
  bool has_ground_truth_tf_{false};
  bool has_map_to_odom_solution_{false};
  rclcpp::Time latest_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_ground_truth_stamp_{0, 0, RCL_ROS_TIME};
  tf2::Transform latest_odom_to_base_;
  tf2::Transform latest_map_to_base_link_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr truth_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odom_tf_sub_;
};

}  // namespace finenav_2d