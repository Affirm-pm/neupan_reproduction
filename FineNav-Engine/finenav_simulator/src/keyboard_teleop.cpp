#include "finenav_simulator/keyboard_teleop.hpp"

#include <algorithm>
#include <string>
#include <cmath>

namespace finenav_2d
{

KeyboardTeleop::KeyboardTeleop(rclcpp::Node * node)
: node_(node)
{
  teleop_input_topic_ = node_->declare_parameter("teleop_input_topic", "/keyboard/keypress");
  teleop_output_topic_ = node_->declare_parameter("teleop_output_topic", "/cmd_vel");
  linear_speed_ = node_->declare_parameter("initial_linear_speed", 0.5);
  angular_speed_ = node_->declare_parameter("initial_angular_speed", 1.0);
  linear_step_ = node_->declare_parameter("linear_step", 0.1);
  angular_step_ = node_->declare_parameter("angular_step", 0.1);
  min_linear_speed_ = node_->declare_parameter("min_linear_speed", 0.0);
  max_linear_speed_ = node_->declare_parameter("max_linear_speed", 2.0);
  min_angular_speed_ = node_->declare_parameter("min_angular_speed", 0.0);
  max_angular_speed_ = node_->declare_parameter("max_angular_speed", 3.0);

  cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(teleop_output_topic_, 10);
  key_subscription_ = node_->create_subscription<std_msgs::msg::Int32>(
    teleop_input_topic_, 10,
    std::bind(&KeyboardTeleop::OnKeypress, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "Keyboard teleop listening on %s and publishing %s",
    teleop_input_topic_.c_str(), teleop_output_topic_.c_str());
}

void KeyboardTeleop::OnKeypress(const std_msgs::msg::Int32::SharedPtr msg)
{
  bool should_publish = true;

  switch (msg->data) {
    case 87:  // W
      current_cmd_.linear.x = linear_speed_;
      current_cmd_.angular.z = 0.0;
      break;
    case 83:  // S
      current_cmd_.linear.x = -linear_speed_;
      current_cmd_.angular.z = 0.0;
      break;
    case 65:  // A
      current_cmd_.linear.x = 0.0;
      current_cmd_.angular.z = angular_speed_;
      break;
    case 68:  // D
      current_cmd_.linear.x = 0.0;
      current_cmd_.angular.z = -angular_speed_;
      break;
    case 81:  // Q
      angular_speed_ = std::min(max_angular_speed_, angular_speed_ + angular_step_);
      if (current_cmd_.angular.z > 0.0) {
        current_cmd_.angular.z = angular_speed_;
      } else if (current_cmd_.angular.z < 0.0) {
        current_cmd_.angular.z = -angular_speed_;
      }
      break;
    case 69:  // E
      angular_speed_ = std::max(min_angular_speed_, angular_speed_ - angular_step_);
      if (current_cmd_.angular.z > 0.0) {
        current_cmd_.angular.z = angular_speed_;
      } else if (current_cmd_.angular.z < 0.0) {
        current_cmd_.angular.z = -angular_speed_;
      }
      break;
    case 90:  // Z
      linear_speed_ = std::min(max_linear_speed_, linear_speed_ + linear_step_);
      if (current_cmd_.linear.x > 0.0) {
        current_cmd_.linear.x = linear_speed_;
      } else if (current_cmd_.linear.x < 0.0) {
        current_cmd_.linear.x = -linear_speed_;
      }
      break;
    case 67:  // C
      linear_speed_ = std::max(min_linear_speed_, linear_speed_ - linear_step_);
      if (current_cmd_.linear.x > 0.0) {
        current_cmd_.linear.x = linear_speed_;
      } else if (current_cmd_.linear.x < 0.0) {
        current_cmd_.linear.x = -linear_speed_;
      }
      break;
    case 88:  // X
      current_cmd_ = geometry_msgs::msg::Twist();
      break;
    default:
      should_publish = false;
      break;
  }

  if (!should_publish) {
    return;
  }

  cmd_publisher_->publish(current_cmd_);
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "linear_speed=%.2f angular_speed=%.2f cmd=(%.2f, %.2f)",
    linear_speed_, angular_speed_, current_cmd_.linear.x, current_cmd_.angular.z);
}

}  // namespace finenav_2d