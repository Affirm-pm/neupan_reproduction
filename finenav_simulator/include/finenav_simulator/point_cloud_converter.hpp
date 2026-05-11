// Copyright 2025 Lihan Chen
//
// Licensed under the Mozilla Public License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.mozilla.org/en-US/MPL/2.0/
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                 uint16_t, ring, ring)(float, time, time))

namespace ign_sim_pointcloud_tool
{
class PointCloudConverter : public rclcpp::Node
{
public:
  explicit PointCloudConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);
  bool LookupOutputTransform(
    const std::string & source_frame, const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform);
  void ApplyFrameTransform(
    pcl::PointCloud<PointXYZIRT>::Ptr & cloud,
    const geometry_msgs::msg::TransformStamped & transform);

  template <typename T>
  void publishPoints(
    const typename pcl::PointCloud<T>::Ptr & new_pc, const sensor_msgs::msg::PointCloud2 & old_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

  int vertical_samples_;
  int horizontal_samples_;
  float vertical_min_angle_;
  float vertical_max_angle_;
  float horizontal_min_angle_;
  float horizontal_max_angle_;
  float scan_period_;
  bool synthesize_time_;
  bool self_filter_box_enable_;
  float self_filter_box_min_x_;
  float self_filter_box_max_x_;
  float self_filter_box_min_y_;
  float self_filter_box_max_y_;
  float self_filter_box_min_z_;
  float self_filter_box_max_z_;
  std::string pcd_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool transform_output_to_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace ign_sim_pointcloud_tool
