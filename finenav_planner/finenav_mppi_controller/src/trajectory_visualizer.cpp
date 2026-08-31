// Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"

namespace mppi
{
namespace
{

constexpr float kNeutralRed = 0.5F;
constexpr float kNeutralGreen = 0.5F;
constexpr float kNeutralBlue = 0.5F;
constexpr float kNeutralAlpha = 0.4F;

struct Color
{
  float r;
  float g;
  float b;
};

bool isFiniteFloat(const float value)
{
  constexpr std::uint32_t exponent_mask = 0x7F800000U;
  return (std::bit_cast<std::uint32_t>(value) & exponent_mask) != exponent_mask;
}

bool isPositiveInfinity(const float value)
{
  return std::bit_cast<std::uint32_t>(value) == 0x7F800000U;
}

Color colorForCritic(const std::string & name)
{
  static constexpr std::array<Color, 8> palette = {{
    {0.12F, 0.47F, 0.71F},    // blue
    {0.17F, 0.63F, 0.17F},    // green
    {0.58F, 0.40F, 0.74F},    // purple
    {0.09F, 0.75F, 0.81F},    // cyan
    {0.89F, 0.47F, 0.76F},    // pink
    {0.55F, 0.34F, 0.12F},    // brown
    {0.30F, 0.30F, 0.30F},    // dark gray
    {0.93F, 0.68F, 0.12F},    // gold
  }};

  if (name == "CostCritic") {
    return palette[0];
  }
  if (name == "ConstraintCritic") {
    return palette[1];
  }
  if (name == "PathFollowCritic") {
    return palette[2];
  }
  if (name == "PathAlignCritic") {
    return palette[3];
  }

  std::size_t hash = 1469598103934665603ULL;
  for (const unsigned char character : name) {
    hash ^= character;
    hash *= 1099511628211ULL;
  }
  return palette[hash % palette.size()];
}

std_msgs::msg::ColorRGBA neutralColor()
{
  std_msgs::msg::ColorRGBA color;
  color.r = kNeutralRed;
  color.g = kNeutralGreen;
  color.b = kNeutralBlue;
  color.a = kNeutralAlpha;
  return color;
}

std_msgs::msg::ColorRGBA diagnosticColor(
  const CriticCostBreakdown & breakdown, const Eigen::Index row)
{
  if (row >= 0 && static_cast<std::size_t>(row) < breakdown.collision_flags.size() &&
    breakdown.collision_flags[static_cast<std::size_t>(row)] != 0U)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0F;
    color.g = 0.1F;
    color.b = 0.1F;
    color.a = 1.0F;
    return color;
  }

  if (!breakdown.valid || breakdown.names.empty() || breakdown.contributions.empty()) {
    return neutralColor();
  }

  float total = 0.0F;
  float dominant = 0.0F;
  std::string dominant_name;
  bool has_finite_contribution = false;

  for (std::size_t critic_index = 0;
    critic_index < breakdown.contributions.size() && critic_index < breakdown.names.size();
    ++critic_index)
  {
    const auto & contribution = breakdown.contributions[critic_index];
    if (row < 0 || row >= contribution.size()) {
      continue;
    }

    const float value = contribution[static_cast<std::size_t>(row)];
    if (isPositiveInfinity(value)) {
      std_msgs::msg::ColorRGBA color;
      color.r = 1.0F;
      color.g = 0.1F;
      color.b = 0.1F;
      color.a = 1.0F;
      return color;
    }
    if (!isFiniteFloat(value)) {
      continue;
    }

    const float positive_value = std::max(value, 0.0F);
    if (positive_value > 0.0F) {
      has_finite_contribution = true;
    }
    total += positive_value;
    if (positive_value > dominant) {
      dominant = positive_value;
      dominant_name = breakdown.names[critic_index];
    }
  }

  if (!has_finite_contribution || !isFiniteFloat(total) ||
    total <= std::numeric_limits<float>::epsilon())
  {
    return neutralColor();
  }

  const auto base = colorForCritic(dominant_name);
  std_msgs::msg::ColorRGBA color;
  color.r = base.r;
  color.g = base.g;
  color.b = base.b;
  color.a = std::clamp(dominant / total, 0.2F, 1.0F);
  return color;
}

void setZeroHeader(std_msgs::msg::Header & header, const std::string & frame_id)
{
  header.frame_id = frame_id;
  header.stamp = builtin_interfaces::msg::Time{};
}

}  // namespace

void LatestFrameChannel::submit(VisualizationFrame::SharedPtr frame)
{
  if (!frame) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) {
      return;
    }
    latest_frame_ = std::move(frame);
  }
  condition_.notify_one();
}

void LatestFrameChannel::clear(const std::uint64_t generation)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_) {
      return;
    }
    latest_frame_.reset();
    if (!clear_generation_ || generation > *clear_generation_) {
      clear_generation_ = generation;
    }
  }
  condition_.notify_one();
}

std::optional<VisualizationEvent> LatestFrameChannel::wait()
{
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this]() {
    return stopped_ || latest_frame_ || clear_generation_.has_value();
  });

  if (clear_generation_) {
    VisualizationEvent event;
    event.type = VisualizationEvent::Type::Clear;
    event.generation = *clear_generation_;
    clear_generation_.reset();
    return event;
  }

  if (latest_frame_) {
    VisualizationEvent event;
    event.type = VisualizationEvent::Type::Frame;
    event.frame = std::move(latest_frame_);
    event.generation = event.frame->generation;
    return event;
  }

  if (stopped_ && !stop_delivered_) {
    stop_delivered_ = true;
    VisualizationEvent event;
    event.type = VisualizationEvent::Type::Stop;
    return event;
  }

  return std::nullopt;
}

void LatestFrameChannel::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    latest_frame_.reset();
    clear_generation_.reset();
  }
  condition_.notify_all();
}

void LatestFrameChannel::restart()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopped_ = false;
  stop_delivered_ = false;
  latest_frame_.reset();
  clear_generation_.reset();
}

TrajectoryVisualizer::~TrajectoryVisualizer()
{
  // FineNavVisualizer is injected as a non-owning pointer. During framework
  // teardown it may be released before the planner instance, so destruction
  // must stop the worker without publishing through that pointer.
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  stopWorker();
  std::lock_guard<std::mutex> lock(mutex_);
  visualizer_ = nullptr;
}

void TrajectoryVisualizer::stopWorker()
{
  channel_.stop();
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void TrajectoryVisualizer::configure(
  finenav::viz::FineNavVisualizer & visualizer, const VisualizerConfig & config)
{
  if (config.trajectory_step == 0 || config.time_step == 0) {
    throw std::invalid_argument("TrajectoryVisualizer sampling steps must be greater than zero");
  }
  if (config.max_trajectories == 0 || config.publish_rate <= 0.0) {
    throw std::invalid_argument(
      "TrajectoryVisualizer maximum trajectories and publish rate must be greater than zero");
  }

  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  stopWorker();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    visualizer_ = &visualizer;
    config_ = config;
  }
  visualizer.markers().setBaseFrame(config.frame_id);

  channel_.restart();
  publish_thread_ = std::thread(&TrajectoryVisualizer::run, this);
}

void TrajectoryVisualizer::submit(
  const models::Trajectories & candidates,
  const CriticCostBreakdown & critic_breakdown,
  const models::BatchArray & optimal,
  const nav_msgs::msg::Path & reference_path,
  const std::uint64_t generation)
{
  VisualizerConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!visualizer_) {
      return;
    }
    config = config_;
  }
  auto frame = buildFrame(
    candidates, critic_breakdown, optimal, reference_path, config, generation);
  channel_.submit(std::move(frame));
}

void TrajectoryVisualizer::clear(const std::uint64_t generation)
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  channel_.clear(generation);
}

void TrajectoryVisualizer::reset()
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  stopWorker();
  if (rclcpp::ok()) {
    publishClear();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  visualizer_ = nullptr;
}

void TrajectoryVisualizer::run()
{
  std::uint64_t current_generation = 0;
  auto next_publish = std::chrono::steady_clock::now();

  while (true) {
    auto event = channel_.wait();
    if (!event || event->type == VisualizationEvent::Type::Stop) {
      return;
    }

    if (event->type == VisualizationEvent::Type::Clear) {
      current_generation = std::max(current_generation, event->generation);
      publishClear();
      event.reset();
      continue;
    }

    if (!event->frame || event->frame->generation < current_generation) {
      continue;
    }

    current_generation = event->frame->generation;
    const auto now = std::chrono::steady_clock::now();
    if (now < next_publish) {
      continue;
    }
    publishFrame(*event->frame);
    event.reset();
    next_publish = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / config_.publish_rate));
  }
}

void TrajectoryVisualizer::publishFrame(const VisualizationFrame & frame)
{
  finenav::viz::FineNavVisualizer * visualizer = nullptr;
  VisualizerConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    visualizer = visualizer_;
    config = config_;
  }
  if (!visualizer) {
    return;
  }

  auto & markers = visualizer->markers();
  markers.deleteAllMarkers(config.trajectory_topic);

  for (const auto & marker : frame.trajectories.markers) {
    if (marker.action != visualization_msgs::msg::Marker::ADD || marker.points.empty()) {
      continue;
    }

    std::vector<Eigen::Vector3d> points;
    points.reserve(marker.points.size());
    for (const auto & point : marker.points) {
      points.emplace_back(point.x, point.y, point.z);
    }
    markers.addLineStrip(points, marker.color, marker.scale.x, marker.ns);
  }

  if (!frame.reference_path.poses.empty()) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(frame.reference_path.poses.size());
    for (const auto & pose : frame.reference_path.poses) {
      points.emplace_back(
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0F;
    color.g = 0.8F;
    color.b = 1.0F;
    color.a = 1.0F;
    markers.addLineStrip(points, color, 0.04, "reference_path");
  }

  markers.publish(config.trajectory_topic);
}

void TrajectoryVisualizer::publishClear()
{
  finenav::viz::FineNavVisualizer * visualizer = nullptr;
  VisualizerConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    visualizer = visualizer_;
    config = config_;
  }
  if (!visualizer) {
    return;
  }
  visualizer->markers().deleteAllMarkers(config.trajectory_topic);
}

visualization_msgs::msg::MarkerArray TrajectoryVisualizer::buildTrajectoryMarkers(
  const models::Trajectories & candidates,
  const CriticCostBreakdown & critic_breakdown,
  const VisualizerConfig & config)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker delete_all;
  setZeroHeader(delete_all.header, config.frame_id);
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  if (config.trajectory_step == 0 || config.time_step == 0) {
    return markers;
  }

  const Eigen::Index batch_size = std::min(
    {
      candidates.x.rows(), candidates.y.rows(), candidates.yaws.rows()});
  const Eigen::Index time_steps = std::min(
    {
      candidates.x.cols(), candidates.y.cols(), candidates.yaws.cols()});
  int marker_id = 0;
  std::size_t emitted_trajectories = 0;
  for (Eigen::Index batch = 0; batch < batch_size;
    batch += static_cast<Eigen::Index>(config.trajectory_step))
  {
    if (emitted_trajectories >= config.max_trajectories) {
      break;
    }

    visualization_msgs::msg::Marker marker;
    setZeroHeader(marker.header, config.frame_id);
    marker.ns = "candidate_trajectories";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.color = diagnosticColor(critic_breakdown, batch);

    for (Eigen::Index time = 0; time < time_steps;
      time += static_cast<Eigen::Index>(config.time_step))
    {
      geometry_msgs::msg::Point point;
      point.x = candidates.x(batch, time);
      point.y = candidates.y(batch, time);
      point.z = 0.0;
      if (isFiniteFloat(point.x) && isFiniteFloat(point.y)) {
        marker.points.push_back(point);
      }
    }

    if (!marker.points.empty()) {
      markers.markers.push_back(marker);
      ++emitted_trajectories;
    }
  }
  return markers;
}

VisualizationFrame::SharedPtr TrajectoryVisualizer::buildFrame(
  const models::Trajectories & candidates,
  const CriticCostBreakdown & critic_breakdown,
  const models::BatchArray & optimal,
  const nav_msgs::msg::Path & reference,
  const VisualizerConfig & config,
  const std::uint64_t generation)
{
  auto frame = std::make_shared<VisualizationFrame>();
  frame->generation = generation;
  frame->trajectories = buildTrajectoryMarkers(candidates, critic_breakdown, config);
  if (optimal.rows() > 0 && optimal.cols() >= 2) {
    frame->trajectories.markers.push_back(buildOptimalMarker(optimal, config));
  }
  frame->optimal_path = buildOptimalPath(optimal, config);
  frame->reference_path = buildReferencePath(reference, config);
  return frame;
}

nav_msgs::msg::Path TrajectoryVisualizer::buildOptimalPath(
  const models::BatchArray & optimal, const VisualizerConfig & config)
{
  nav_msgs::msg::Path path;
  setZeroHeader(path.header, config.frame_id);
  if (optimal.cols() < 2) {
    return path;
  }

  path.poses.reserve(static_cast<std::size_t>(optimal.rows()));
  for (Eigen::Index row = 0; row < optimal.rows(); ++row) {
    if (!isFiniteFloat(optimal(row, 0)) || !isFiniteFloat(optimal(row, 1))) {
      continue;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = optimal(row, 0);
    pose.pose.position.y = optimal(row, 1);
    pose.pose.orientation.w = 1.0;
    if (optimal.cols() >= 3 && isFiniteFloat(optimal(row, 2))) {
      const float half_yaw = optimal(row, 2) * 0.5F;
      pose.pose.orientation.z = std::sin(half_yaw);
      pose.pose.orientation.w = std::cos(half_yaw);
    }
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path TrajectoryVisualizer::buildReferencePath(
  const nav_msgs::msg::Path & reference, const VisualizerConfig & config)
{
  nav_msgs::msg::Path path;
  setZeroHeader(path.header, config.frame_id);
  path.poses = reference.poses;
  for (auto & pose : path.poses) {
    pose.header = path.header;
  }
  return path;
}

visualization_msgs::msg::Marker TrajectoryVisualizer::buildOptimalMarker(
  const models::BatchArray & optimal, const VisualizerConfig & config)
{
  visualization_msgs::msg::Marker marker;
  setZeroHeader(marker.header, config.frame_id);
  marker.ns = "optimal_trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.06;
  marker.color.r = 1.0F;
  marker.color.g = 0.45F;
  marker.color.b = 0.0F;
  marker.color.a = 1.0F;

  for (Eigen::Index row = 0; row < optimal.rows(); ++row) {
    if (optimal.cols() >= 2 && isFiniteFloat(optimal(row, 0)) &&
      isFiniteFloat(optimal(row, 1)))
    {
      geometry_msgs::msg::Point point;
      point.x = optimal(row, 0);
      point.y = optimal(row, 1);
      point.z = 0.03;
      marker.points.push_back(point);
    }
  }
  return marker;
}

}  // namespace mppi
