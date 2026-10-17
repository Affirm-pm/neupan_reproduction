#include "finenav_simulator/ground_truth_map_localizer.hpp"

#include <string>

namespace finenav_2d
{

namespace
{

template<typename T>
T DeclareOrGetParameter(rclcpp::Node * node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

}  // namespace

GroundTruthMapPublisher::GroundTruthMapPublisher(rclcpp::Node * node)
: node_(node),
  static_tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(node))
{
  enable_identity_map_to_odom_ =
    DeclareOrGetParameter<bool>(node_, "enable_identity_map_to_odom", false);
  map_frame_ = DeclareOrGetParameter<std::string>(node_, "map_frame", "map");
  odom_frame_ = DeclareOrGetParameter<std::string>(node_, "odom_frame", "odom");

  if (enable_identity_map_to_odom_) {
    PublishIdentityMapToOdom();
  }
}

void GroundTruthMapPublisher::PublishIdentityMapToOdom() const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->get_clock()->now();
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(transform);
}

}  // namespace finenav_2d
