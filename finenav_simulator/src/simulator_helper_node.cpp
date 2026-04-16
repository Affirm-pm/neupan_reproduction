#include "finenav_simulator/simulator_helper_node.hpp"

namespace finenav_2d
{

SimulatorHelper::SimulatorHelper(std::shared_ptr<rclcpp::Node> node)
: node_(node),
  tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(*node_))
{
  teleop_ = std::make_unique<KeyboardTeleop>(node_.get());
  path_publisher_ = std::make_unique<OdomPathPublisher>(node_.get(), tf_broadcaster_);
  localizer_ = std::make_unique<GroundTruthMapLocalizer>(node_.get(), tf_broadcaster_);
}

}  // namespace finenav_2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("simulator_helper_node");
  auto helper = std::make_unique<finenav_2d::SimulatorHelper>(node);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
