#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "finenav_simulator/ground_truth_map_localizer.hpp"
#include "finenav_simulator/keyboard_teleop.hpp"
#include "finenav_simulator/odom_path_publisher.hpp"

namespace finenav_2d
{

class SimulatorHelper
{
public:
  explicit SimulatorHelper(std::shared_ptr<rclcpp::Node> node);

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<KeyboardTeleop> teleop_;
  std::unique_ptr<OdomPathPublisher> path_publisher_;
  std::unique_ptr<GroundTruthMapPublisher> publisher_;
};

}  // namespace finenav_2d
