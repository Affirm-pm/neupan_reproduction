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

#ifndef FINENAV_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
#define FINENAV_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_

#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"

namespace mppi
{

struct VisualizerConfig
{
  std::string frame_id{"map"};
  std::string trajectory_topic{"/mppi_debug/trajectories"};
  std::string optimal_path_topic{"/mppi_debug/optimal_trajectory"};
  std::string reference_path_topic{"/mppi_debug/reference_path"};
  std::size_t trajectory_step{5};
  std::size_t time_step{3};
  std::size_t max_trajectories{500};
  double publish_rate{10.0};
};

struct VisualizationFrame
{
  using SharedPtr = std::shared_ptr<const VisualizationFrame>;

  std::uint64_t generation{0};
  visualization_msgs::msg::MarkerArray trajectories;
  nav_msgs::msg::Path optimal_path;
  nav_msgs::msg::Path reference_path;
};

struct VisualizationEvent
{
  enum class Type {Frame, Clear, Stop};

  Type type{Type::Frame};
  VisualizationFrame::SharedPtr frame;
  std::uint64_t generation{0};
};

/**
 * @brief Capacity-one producer/consumer channel for immutable visualization frames.
 */
class LatestFrameChannel
{
public:
  void submit(VisualizationFrame::SharedPtr frame);
  void clear(std::uint64_t generation);
  std::optional<VisualizationEvent> wait();
  void stop();
  void restart();

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  VisualizationFrame::SharedPtr latest_frame_;
  std::optional<std::uint64_t> clear_generation_;
  bool stopped_{false};
  bool stop_delivered_{false};
};

/**
 * @brief Publishes Nav2-style MPPI diagnostic trajectories for RViz.
 *
 * The component owns publishers and, when configured, one latest-frame
 * publishing thread. It has no subscriptions, timers, or executor.
 */
class TrajectoryVisualizer
{
public:
  ~TrajectoryVisualizer();

  void configure(const rclcpp::Node::SharedPtr & node, const VisualizerConfig & config);

  void submit(
    const models::Trajectories & candidates,
    const CriticCostBreakdown & critic_breakdown,
    const models::BatchArray & optimal,
    const nav_msgs::msg::Path & reference_path,
    std::uint64_t generation);

  void clear(std::uint64_t generation);

  void reset();

private:
  static visualization_msgs::msg::MarkerArray buildTrajectoryMarkers(
    const models::Trajectories & candidates,
    const CriticCostBreakdown & critic_breakdown,
    const VisualizerConfig & config);

  static VisualizationFrame::SharedPtr buildFrame(
    const models::Trajectories & candidates,
    const CriticCostBreakdown & critic_breakdown,
    const models::BatchArray & optimal,
    const nav_msgs::msg::Path & reference,
    const VisualizerConfig & config,
    std::uint64_t generation);

  static nav_msgs::msg::Path buildOptimalPath(
    const models::BatchArray & optimal,
    const VisualizerConfig & config);

  static nav_msgs::msg::Path buildReferencePath(
    const nav_msgs::msg::Path & reference,
    const VisualizerConfig & config);

  void stopWorker();
  void run();
  void publishFrame(const VisualizationFrame & frame);
  void publishClear();

  static visualization_msgs::msg::Marker buildOptimalMarker(
    const models::BatchArray & optimal,
    const VisualizerConfig & config);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectories_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_publisher_;
  VisualizerConfig config_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_mutex_;
  LatestFrameChannel channel_;
  std::thread publish_thread_;
};

}  // namespace mppi

#endif  // FINENAV_MPPI_CONTROLLER__TOOLS__TRAJECTORY_VISUALIZER_HPP_
