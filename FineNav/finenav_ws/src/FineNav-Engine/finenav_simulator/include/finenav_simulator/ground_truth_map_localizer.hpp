#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace finenav_2d
{

class GroundTruthMapPublisher
{
public:
  explicit GroundTruthMapPublisher(rclcpp::Node * node);

private:
  void PublishIdentityMapToOdom() const;

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  bool enable_identity_map_to_odom_{false};
  std::string map_frame_;
  std::string odom_frame_;
};

}  // namespace finenav_2d
