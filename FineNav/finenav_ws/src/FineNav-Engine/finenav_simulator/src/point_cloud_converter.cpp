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

#include "finenav_simulator/point_cloud_converter.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace
{
constexpr float kTwoPi = 2.0F * static_cast<float>(M_PI);

bool HasField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name)
{
  return std::any_of(
    msg.fields.begin(), msg.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {
      return field.name == field_name;
    });
}

size_t ClampIndex(int index, int size)
{
  if (size <= 1) {
    return 0;
  }
  return static_cast<size_t>(std::clamp(index, 0, size - 1));
}

float NormalizeAnglePositive(float angle)
{
  float normalized = std::fmod(angle, kTwoPi);
  if (normalized < 0.0F) {
    normalized += kTwoPi;
  }
  return normalized;
}

size_t ComputeRingFromAngle(
  float x, float y, float z, float vertical_min_angle, float vertical_max_angle,
  int vertical_samples)
{
  if (vertical_samples <= 1) {
    return 0;
  }

  const float vertical_span = vertical_max_angle - vertical_min_angle;
  if (vertical_span <= 0.0F) {
    return 0;
  }

  const float vertical_step = vertical_span / static_cast<float>(vertical_samples - 1);
  const float vertical_angle = std::atan2(z, std::hypot(x, y));
  const int ring = static_cast<int>(std::lround((vertical_angle - vertical_min_angle) / vertical_step));
  return ClampIndex(ring, vertical_samples);
}

size_t ComputeColumnFromAngle(
  float x, float y, float horizontal_min_angle, float horizontal_max_angle,
  int horizontal_samples)
{
  if (horizontal_samples <= 1) {
    return 0;
  }

  const float horizontal_span = horizontal_max_angle - horizontal_min_angle;
  if (horizontal_span <= 0.0F) {
    return 0;
  }

  const float horizontal_step = horizontal_span / static_cast<float>(horizontal_samples);
  const float azimuth = NormalizeAnglePositive(std::atan2(y, x) - horizontal_min_angle);
  const int column = static_cast<int>(std::floor(azimuth / horizontal_step));
  return ClampIndex(column, horizontal_samples);
}

float ComputeRelativeTime(size_t column, int horizontal_samples, float scan_period)
{
  if (horizontal_samples <= 0) {
    return 0.0F;
  }
  return static_cast<float>(column) * scan_period / static_cast<float>(horizontal_samples);
}

bool IsInsideBox(
  float x, float y, float z,
  float min_x, float max_x,
  float min_y, float max_y,
  float min_z, float max_z)
{
  return x >= min_x && x <= max_x &&
         y >= min_y && y <= max_y &&
         z >= min_z && z <= max_z;
}

void ApplyTransformToPoint(
  const geometry_msgs::msg::TransformStamped & transform,
  float & x, float & y, float & z)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;

  tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
  tf2::Matrix3x3 basis(quaternion);
  const tf2::Vector3 point(x, y, z);
  const tf2::Vector3 transformed = basis * point;

  x = static_cast<float>(transformed.x() + translation.x);
  y = static_cast<float>(transformed.y() + translation.y);
  z = static_cast<float>(transformed.z() + translation.z);
}
}  // namespace

namespace ign_sim_pointcloud_tool
{
PointCloudConverter::PointCloudConverter(const rclcpp::NodeOptions & options)
: rclcpp::Node("point_cloud_converter", options)
{
  this->declare_parameter<std::string>("pcd_topic", "livox/lidar");
  this->declare_parameter<int>("vertical_samples", 0);
  this->declare_parameter<int>("horizontal_samples", 0);
  this->declare_parameter<float>("vertical_min_angle", -0.26F);
  this->declare_parameter<float>("vertical_max_angle", 1.57F);
  this->declare_parameter<float>("horizontal_min_angle", 0.0F);
  this->declare_parameter<float>("horizontal_max_angle", 6.2831852F);
  this->declare_parameter<int>("n_scan", 0);
  this->declare_parameter<int>("horizon_scan", 0);
  this->declare_parameter<float>("scan_period", 0.1F);
  this->declare_parameter<bool>("synthesize_time", true);
  this->declare_parameter<std::string>("output_topic", "/cloud_for_lio");
  this->declare_parameter<std::string>("output_frame_id", "");
  this->declare_parameter<bool>("transform_output_to_frame", false);
  // Self-filter box in lidar_frame, sized from the current robot body + wheels.
  this->declare_parameter<bool>("self_filter_box.enable", true);
  this->declare_parameter<float>("self_filter_box.min_x", -0.50F);
  this->declare_parameter<float>("self_filter_box.max_x", 0.26F);
  this->declare_parameter<float>("self_filter_box.min_y", -0.32F);
  this->declare_parameter<float>("self_filter_box.max_y", 0.32F);
  this->declare_parameter<float>("self_filter_box.min_z", -0.42F);
  this->declare_parameter<float>("self_filter_box.max_z", 0.05F);

  pcd_topic_ = this->get_parameter("pcd_topic").as_string();
  vertical_samples_ = this->get_parameter("vertical_samples").as_int();
  horizontal_samples_ = this->get_parameter("horizontal_samples").as_int();
  vertical_min_angle_ = this->get_parameter("vertical_min_angle").as_double();
  vertical_max_angle_ = this->get_parameter("vertical_max_angle").as_double();
  horizontal_min_angle_ = this->get_parameter("horizontal_min_angle").as_double();
  horizontal_max_angle_ = this->get_parameter("horizontal_max_angle").as_double();
  scan_period_ = this->get_parameter("scan_period").as_double();
  synthesize_time_ = this->get_parameter("synthesize_time").as_bool();
  output_topic_ = this->get_parameter("output_topic").as_string();
  output_frame_id_ = this->get_parameter("output_frame_id").as_string();
  transform_output_to_frame_ = this->get_parameter("transform_output_to_frame").as_bool();
  self_filter_box_enable_ = this->get_parameter("self_filter_box.enable").as_bool();
  self_filter_box_min_x_ = this->get_parameter("self_filter_box.min_x").as_double();
  self_filter_box_max_x_ = this->get_parameter("self_filter_box.max_x").as_double();
  self_filter_box_min_y_ = this->get_parameter("self_filter_box.min_y").as_double();
  self_filter_box_max_y_ = this->get_parameter("self_filter_box.max_y").as_double();
  self_filter_box_min_z_ = this->get_parameter("self_filter_box.min_z").as_double();
  self_filter_box_max_z_ = this->get_parameter("self_filter_box.max_z").as_double();

  if (vertical_samples_ <= 0) {
    vertical_samples_ = this->get_parameter("n_scan").as_int();
  }
  if (horizontal_samples_ <= 0) {
    horizontal_samples_ = this->get_parameter("horizon_scan").as_int();
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pcd_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PointCloudConverter::lidarHandle, this, std::placeholders::_1));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

  RCLCPP_INFO(
    this->get_logger(),
    "Listening to lidar topic: %s, expected layout: %d x %d, h_angle:[%.4f, %.4f], v_angle:[%.4f, %.4f], scan_period: %.4f, synthesize_time: %s, output_topic: %s, output_frame_id: %s, transform_output_to_frame: %s, self_filter_box: %s [x: %.2f..%.2f, y: %.2f..%.2f, z: %.2f..%.2f]",
    pcd_topic_.c_str(), vertical_samples_, horizontal_samples_,
    horizontal_min_angle_, horizontal_max_angle_, vertical_min_angle_, vertical_max_angle_,
    scan_period_, synthesize_time_ ? "true" : "false", output_topic_.c_str(),
    output_frame_id_.empty() ? "<input>" : output_frame_id_.c_str(),
    transform_output_to_frame_ ? "true" : "false",
    self_filter_box_enable_ ? "enabled" : "disabled",
    self_filter_box_min_x_, self_filter_box_max_x_,
    self_filter_box_min_y_, self_filter_box_max_y_,
    self_filter_box_min_z_, self_filter_box_max_z_);
}

bool PointCloudConverter::LookupOutputTransform(
  const std::string & source_frame, const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = tf_buffer_->lookupTransform(output_frame_id_, source_frame, stamp);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to transform point cloud from '%s' to '%s': %s",
      source_frame.c_str(), output_frame_id_.c_str(), ex.what());
    return false;
  }
}

void PointCloudConverter::ApplyFrameTransform(
  pcl::PointCloud<PointXYZIRT>::Ptr & cloud,
  const geometry_msgs::msg::TransformStamped & transform)
{
  for (auto & point : cloud->points) {
    ApplyTransformToPoint(transform, point.x, point.y, point.z);
  }
}

void PointCloudConverter::lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg)
{
  pcl::PointCloud<PointXYZIRT>::Ptr converted_pc(new pcl::PointCloud<PointXYZIRT>());
  const size_t point_count = static_cast<size_t>(pc_msg->width) * static_cast<size_t>(pc_msg->height);
  converted_pc->points.reserve(point_count);

  const bool organized_cloud = pc_msg->height > 1 && pc_msg->width > 1;
  int vertical_samples = vertical_samples_;
  int horizontal_samples = horizontal_samples_;

  if (organized_cloud) {
    if (
      vertical_samples_ > 0 && horizontal_samples_ > 0 &&
      (vertical_samples_ != static_cast<int>(pc_msg->height) ||
      horizontal_samples_ != static_cast<int>(pc_msg->width)))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Configured GPU LiDAR layout %d x %d, but incoming organized cloud is %u x %u. Using incoming layout.",
        vertical_samples_, horizontal_samples_, pc_msg->height, pc_msg->width);
    }
    vertical_samples = static_cast<int>(pc_msg->height);
    horizontal_samples = static_cast<int>(pc_msg->width);
  }

  if (vertical_samples <= 0 || horizontal_samples <= 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Invalid lidar layout: vertical_samples=%d, horizontal_samples=%d",
      vertical_samples, horizontal_samples);
    return;
  }

  const size_t required_data_size =
    static_cast<size_t>(pc_msg->width) * static_cast<size_t>(pc_msg->height) *
    static_cast<size_t>(pc_msg->point_step);
  if (pc_msg->data.size() < required_data_size) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Malformed PointCloud2 data: data.size()=%zu, required=%zu",
      pc_msg->data.size(), required_data_size);
    return;
  }

  const bool has_intensity = HasField(*pc_msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<float> x_iter(*pc_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_iter(*pc_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_iter(*pc_msg, "z");
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> intensity_iter;
  if (has_intensity) {
    intensity_iter = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*pc_msg, "intensity");
  }

  bool removed_points = false;

  for (size_t point_id = 0; point_id < point_count; ++point_id, ++x_iter, ++y_iter, ++z_iter) {
    PointXYZIRT new_point{};
    new_point.x = *x_iter;
    new_point.y = *y_iter;
    new_point.z = *z_iter;
    new_point.intensity = has_intensity ? **intensity_iter : 0.0F;

    if (has_intensity) {
      ++(*intensity_iter);
    }

    if (
      self_filter_box_enable_ &&
      IsInsideBox(
        new_point.x, new_point.y, new_point.z,
        self_filter_box_min_x_, self_filter_box_max_x_,
        self_filter_box_min_y_, self_filter_box_max_y_,
        self_filter_box_min_z_, self_filter_box_max_z_))
    {
      removed_points = true;
      continue;
    }

    size_t ring = 0;
    size_t column = 0;

    if (organized_cloud) {
      column = point_id % static_cast<size_t>(horizontal_samples);
      ring = point_id / static_cast<size_t>(horizontal_samples);
    } else {
      ring = ComputeRingFromAngle(
        new_point.x, new_point.y, new_point.z,
        vertical_min_angle_, vertical_max_angle_, vertical_samples);
      if (synthesize_time_) {
        column = ComputeColumnFromAngle(
          new_point.x, new_point.y,
          horizontal_min_angle_, horizontal_max_angle_, horizontal_samples);
      }
    }

    new_point.ring = static_cast<uint16_t>(ring);
    new_point.time = synthesize_time_ ?
      ComputeRelativeTime(column, horizontal_samples, scan_period_) : 0.0F;
    converted_pc->points.push_back(new_point);
  }

  const bool keep_organized_output = organized_cloud && !removed_points;
  converted_pc->width = keep_organized_output ? static_cast<uint32_t>(horizontal_samples) :
    static_cast<uint32_t>(converted_pc->points.size());
  converted_pc->height = keep_organized_output ? static_cast<uint32_t>(vertical_samples) : 1;

  geometry_msgs::msg::TransformStamped output_transform;
  if (transform_output_to_frame_) {
    if (output_frame_id_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "transform_output_to_frame is enabled but output_frame_id is empty.");
      return;
    }
    if (pc_msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Incoming point cloud has empty frame_id; cannot transform to '%s'.",
        output_frame_id_.c_str());
      return;
    }
    if (!LookupOutputTransform(pc_msg->header.frame_id, pc_msg->header.stamp, output_transform)) {
      return;
    }
    ApplyFrameTransform(converted_pc, output_transform);
  }

  publishPoints<PointXYZIRT>(converted_pc, *pc_msg);
}

template <typename T>
void PointCloudConverter::publishPoints(
  const typename pcl::PointCloud<T>::Ptr & new_pc, const sensor_msgs::msg::PointCloud2 & old_msg)
{
  new_pc->is_dense = false;

  sensor_msgs::msg::PointCloud2 pc_new_msg;
  pcl::toROSMsg(*new_pc, pc_new_msg);
  pc_new_msg.header = old_msg.header;
  if (!output_frame_id_.empty()) {
    pc_new_msg.header.frame_id = output_frame_id_;
  }

  pcd_pub_->publish(pc_new_msg);
}

}  // namespace ign_sim_pointcloud_tool

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<ign_sim_pointcloud_tool::PointCloudConverter>(options));
  rclcpp::shutdown();
  return 0;
}
