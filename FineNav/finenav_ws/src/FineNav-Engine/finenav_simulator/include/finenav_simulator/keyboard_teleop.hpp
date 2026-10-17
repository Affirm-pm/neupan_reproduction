#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace finenav_2d
{

class KeyboardTeleop
{
public:
  explicit KeyboardTeleop(rclcpp::Node * node);

private:
  void OnKeypress(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node * node_;
  std::string teleop_input_topic_;
  std::string teleop_output_topic_;
  double linear_speed_;
  double angular_speed_;
  double linear_step_;
  double angular_step_;
  double min_linear_speed_;
  double max_linear_speed_;
  double min_angular_speed_;
  double max_angular_speed_;
  geometry_msgs::msg::Twist current_cmd_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr key_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
};

}  // namespace finenav_2d