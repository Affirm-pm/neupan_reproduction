// Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "finenav_visualizer/finenav_visualizer.hpp"
#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"

namespace
{

const visualization_msgs::msg::Marker * findMarker(
  const visualization_msgs::msg::MarkerArray & markers, const std::string & ns)
{
  const auto it = std::find_if(
    markers.markers.begin(), markers.markers.end(),
    [&ns](const auto & marker) {return marker.ns == ns;});
  return it == markers.markers.end() ? nullptr : &*it;
}

class TrajectoryVisualizerTest : public ::testing::Test
{
protected:
  struct CapturedMessages
  {
    std::shared_ptr<visualization_msgs::msg::MarkerArray> markers;
  };

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("trajectory_visualizer_test_node");
    framework_visualizer_ = std::make_shared<finenav::viz::FineNavVisualizer>(node_);
    config_ = makeConfig();
    const auto suffix = std::to_string(next_topic_id_.fetch_add(1));
    config_.trajectory_topic += suffix;
    visualizer_.configure(*framework_visualizer_, config_);
  }

  void TearDown() override
  {
    visualizer_.reset();
    framework_visualizer_.reset();
    node_.reset();
  }

  static mppi::VisualizerConfig makeConfig()
  {
    mppi::VisualizerConfig config;
    config.frame_id = "map";
    config.trajectory_topic = "/mppi_visualizer_test/trajectories_";
    config.optimal_path_topic = "/mppi_visualizer_test/optimal_";
    config.reference_path_topic = "/mppi_visualizer_test/reference_";
    config.trajectory_step = 2;
    config.time_step = 2;
    config.max_trajectories = 100;
    config.publish_rate = 1000.0;
    return config;
  }

  static mppi::models::Trajectories makeCandidates()
  {
    mppi::models::Trajectories candidates;
    candidates.reset(5, 5);
    for (Eigen::Index row = 0; row < candidates.x.rows(); ++row) {
      for (Eigen::Index col = 0; col < candidates.x.cols(); ++col) {
        candidates.x(row, col) = static_cast<float>(row + col * 0.1F);
        candidates.y(row, col) = static_cast<float>(row * 0.01F);
        candidates.yaws(row, col) = static_cast<float>(col * 0.05F);
      }
    }
    return candidates;
  }

  static mppi::CriticCostBreakdown makeBreakdown()
  {
    mppi::CriticCostBreakdown breakdown;
    breakdown.names = {"CostCritic", "PathFollowCritic"};
    breakdown.contributions.resize(2);
    breakdown.contributions[0] = std::vector<float>(5, 1.0F);
    breakdown.contributions[1] = std::vector<float>(5, 0.0F);
    breakdown.valid = true;
    return breakdown;
  }

  CapturedMessages capture(
    const mppi::models::Trajectories & candidates,
    const mppi::CriticCostBreakdown & breakdown,
    const mppi::models::BatchArray & optimal,
    const nav_msgs::msg::Path & reference)
  {
    CapturedMessages captured{
      std::make_shared<visualization_msgs::msg::MarkerArray>()};
    auto marker_subscription = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      config_.trajectory_topic, rclcpp::QoS(10),
      [messages = captured.markers](visualization_msgs::msg::MarkerArray::SharedPtr message) {
        *messages = *message;
      });
    visualizer_.submit(candidates, breakdown, optimal, reference, 42);

    for (int attempt = 0; attempt < 100; ++attempt) {
      rclcpp::spin_some(node_);
      const auto has_optimal = std::any_of(
        captured.markers->markers.begin(), captured.markers->markers.end(),
        [](const auto & marker) {return marker.ns == "optimal_trajectory";});
      if (has_optimal) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    (void)marker_subscription;
    return captured;
  }

  mppi::VisualizerConfig config_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<finenav::viz::FineNavVisualizer> framework_visualizer_;
  mppi::TrajectoryVisualizer visualizer_;
  inline static std::atomic<std::uint64_t> next_topic_id_{0};
};

TEST_F(TrajectoryVisualizerTest, CandidateMarkersStartWithDeleteAllAndDownsample)
{
  const auto captured = capture(
    makeCandidates(), makeBreakdown(), mppi::models::BatchArray::Ones(3, 3),
    nav_msgs::msg::Path{});

  ASSERT_EQ(captured.markers->markers.size(), 4U);
  for (std::size_t index = 0; index < 3; ++index) {
    const auto & marker = captured.markers->markers[index];
    EXPECT_EQ(marker.header.frame_id, "map");
    EXPECT_EQ(marker.ns, "candidate_trajectories");
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::LINE_STRIP);
    EXPECT_EQ(marker.points.size(), 3U);
  }
  EXPECT_EQ(captured.markers->markers.back().ns, "optimal_trajectory");
  EXPECT_EQ(captured.markers->markers.back().type, visualization_msgs::msg::Marker::LINE_STRIP);
}

TEST_F(TrajectoryVisualizerTest, CandidateMarkersRespectMaximumCount)
{
  config_.max_trajectories = 2;
  visualizer_.reset();
  visualizer_.configure(*framework_visualizer_, config_);
  const auto captured = capture(
    makeCandidates(), makeBreakdown(), mppi::models::BatchArray::Ones(3, 3),
    nav_msgs::msg::Path{});

  ASSERT_EQ(captured.markers->markers.size(), 3U);
  EXPECT_EQ(captured.markers->markers[0].ns, "candidate_trajectories");
  EXPECT_EQ(captured.markers->markers[1].ns, "candidate_trajectories");
  EXPECT_EQ(captured.markers->markers.back().ns, "optimal_trajectory");
}

TEST_F(TrajectoryVisualizerTest, SubmitBuildsImmutableFrameBeforeSourceMutation)
{
  auto candidates = makeCandidates();
  mppi::models::BatchArray optimal = mppi::models::BatchArray::Ones(3, 3);
  nav_msgs::msg::Path reference;
  reference.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 4.0;
  reference.poses.push_back(pose);

  const auto captured = capture(candidates, makeBreakdown(), optimal, reference);
  candidates.x.setConstant(-100.0F);
  optimal.setConstant(-100.0F);
  reference.poses.clear();

  const auto * candidate = findMarker(*captured.markers, "candidate_trajectories");
  const auto * optimal_marker = findMarker(*captured.markers, "optimal_trajectory");
  const auto * reference_marker = findMarker(*captured.markers, "reference_path");
  ASSERT_NE(candidate, nullptr);
  ASSERT_NE(optimal_marker, nullptr);
  ASSERT_NE(reference_marker, nullptr);
  EXPECT_FLOAT_EQ(candidate->points.front().x, 0.0F);
  EXPECT_EQ(optimal_marker->points.size(), 3U);
  EXPECT_FLOAT_EQ(optimal_marker->points.front().x, 1.0F);
  ASSERT_EQ(reference_marker->points.size(), 1U);
  EXPECT_FLOAT_EQ(reference_marker->points.front().x, 4.0F);
}

TEST_F(TrajectoryVisualizerTest, ZeroContributionUsesNeutralColor)
{
  auto breakdown = makeBreakdown();
  std::fill(breakdown.contributions[0].begin(), breakdown.contributions[0].end(), 0.0F);
  std::fill(breakdown.contributions[1].begin(), breakdown.contributions[1].end(), 0.0F);
  const auto captured = capture(
    makeCandidates(), breakdown, mppi::models::BatchArray::Ones(3, 3), nav_msgs::msg::Path{});

  const auto * marker = findMarker(*captured.markers, "candidate_trajectories");
  ASSERT_NE(marker, nullptr);
  EXPECT_NEAR(marker->color.r, 0.5F, 1e-6F);
  EXPECT_NEAR(marker->color.g, 0.5F, 1e-6F);
  EXPECT_NEAR(marker->color.b, 0.5F, 1e-6F);
  EXPECT_NEAR(marker->color.a, 0.4F, 1e-6F);
}

TEST_F(TrajectoryVisualizerTest, DominantCriticControlsDeterministicColorAndAlpha)
{
  auto breakdown = makeBreakdown();
  breakdown.contributions[0][0] = 1.0F;
  breakdown.contributions[1][0] = 3.0F;
  breakdown.contributions[0][2] = 4.0F;
  breakdown.contributions[1][2] = 0.0F;
  const auto captured = capture(
    makeCandidates(), breakdown, mppi::models::BatchArray::Ones(3, 3), nav_msgs::msg::Path{});

  ASSERT_GE(captured.markers->markers.size(), 4U);
  EXPECT_NEAR(captured.markers->markers[0].color.r, 0.58F, 1e-6F);
  EXPECT_NEAR(captured.markers->markers[0].color.g, 0.40F, 1e-6F);
  EXPECT_NEAR(captured.markers->markers[0].color.b, 0.74F, 1e-6F);
  EXPECT_NEAR(captured.markers->markers[0].color.a, 0.75F, 1e-6F);

  EXPECT_NEAR(captured.markers->markers[1].color.r, 0.12F, 1e-6F);
  EXPECT_NEAR(captured.markers->markers[1].color.g, 0.47F, 1e-6F);
  EXPECT_NEAR(captured.markers->markers[1].color.b, 0.71F, 1e-6F);
  EXPECT_NEAR(captured.markers->markers[1].color.a, 1.0F, 1e-6F);
}

TEST_F(TrajectoryVisualizerTest, NonFiniteContributionUsesCollisionColor)
{
  auto breakdown = makeBreakdown();
  breakdown.contributions[0][0] = std::numeric_limits<float>::infinity();
  const auto captured = capture(
    makeCandidates(), breakdown, mppi::models::BatchArray::Ones(3, 3), nav_msgs::msg::Path{});

  const auto * marker = findMarker(*captured.markers, "candidate_trajectories");
  ASSERT_NE(marker, nullptr);
  EXPECT_NEAR(marker->color.r, 1.0F, 1e-6F);
  EXPECT_NEAR(marker->color.g, 0.1F, 1e-6F);
  EXPECT_NEAR(marker->color.b, 0.1F, 1e-6F);
  EXPECT_TRUE(std::isfinite(marker->color.a));
}

TEST_F(TrajectoryVisualizerTest, ExplicitCollisionFlagUsesCollisionColor)
{
  mppi::CriticCostBreakdown breakdown;
  breakdown.collision_flags = {1U, 0U, 0U, 0U, 0U};
  const auto captured = capture(
    makeCandidates(), breakdown, mppi::models::BatchArray::Ones(3, 3), nav_msgs::msg::Path{});

  const auto * marker = findMarker(*captured.markers, "candidate_trajectories");
  ASSERT_NE(marker, nullptr);
  EXPECT_NEAR(marker->color.r, 1.0F, 1e-6F);
  EXPECT_NEAR(marker->color.g, 0.1F, 1e-6F);
  EXPECT_NEAR(marker->color.b, 0.1F, 1e-6F);
  EXPECT_FLOAT_EQ(marker->color.a, 1.0F);
}

TEST_F(TrajectoryVisualizerTest, ConvertsOptimalTrajectoryToLineStrip)
{
  mppi::models::BatchArray optimal(3, 3);
  optimal << 1.0F, 2.0F, 0.1F,
    2.0F, 3.0F, 0.2F,
    3.0F, 4.0F, 0.3F;
  const auto captured = capture(
    makeCandidates(), makeBreakdown(), optimal, nav_msgs::msg::Path{});

  const auto * marker = findMarker(*captured.markers, "optimal_trajectory");
  ASSERT_NE(marker, nullptr);
  ASSERT_EQ(marker->points.size(), 3U);
  EXPECT_EQ(marker->header.frame_id, "map");
  EXPECT_FLOAT_EQ(marker->points[1].x, 2.0F);
  EXPECT_FLOAT_EQ(marker->points[1].y, 3.0F);
}

TEST_F(TrajectoryVisualizerTest, ConvertsReferencePathToLineStrip)
{
  nav_msgs::msg::Path reference;
  reference.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 4.0;
  pose.pose.position.y = -2.0;
  reference.poses.push_back(pose);

  const auto captured = capture(
    makeCandidates(), makeBreakdown(), mppi::models::BatchArray::Ones(3, 3), reference);

  const auto * marker = findMarker(*captured.markers, "reference_path");
  ASSERT_NE(marker, nullptr);
  ASSERT_EQ(marker->points.size(), 1U);
  EXPECT_EQ(marker->header.frame_id, "map");
  EXPECT_FLOAT_EQ(marker->points.front().x, 4.0F);
  EXPECT_FLOAT_EQ(marker->points.front().y, -2.0F);
}

}  // namespace
