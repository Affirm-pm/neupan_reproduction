// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Pulled in by the finenav header below, so it has to be parsed BEFORE the
// access hack (see marker_viz_test.cpp for the same pattern).
#include <fmt/format.h>

#define private public
#include "finenav_visualizer/point_cloud_viz.hpp"
#undef private

namespace {

using finenav::viz::Color;
using finenav::viz::ColorName;
using finenav::viz::ColorScale;
using finenav::viz::PointCloudViz;

// A colour channel in [0, 1] lands in the cloud as a byte, truncating — the one
// narrowing on the way to the wire, and the reason the expectations below are
// integers.
constexpr uint8_t byte(float channel) {
    return static_cast<uint8_t>(channel * 255.0f);
}

}  // namespace

// A point staged with no colour is still a point: the default is the palette's
// grey, which is what an algorithm that never thought about colour gets.
TEST(PointCloudVizTest, AddPointDefaultsToGrey) {
    PointCloudViz cloud("finenav_test_cloud_default");

    cloud.addPoint(1.0f, 2.0f, 3.0f);

    ASSERT_EQ(cloud.cloud_.points.size(), 1U);
    EXPECT_FLOAT_EQ(cloud.cloud_.points[0].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud.cloud_.points[0].y, 2.0f);
    EXPECT_FLOAT_EQ(cloud.cloud_.points[0].z, 3.0f);
    EXPECT_EQ(cloud.cloud_.points[0].r, byte(0.7f));
    EXPECT_EQ(cloud.cloud_.points[0].g, byte(0.7f));
    EXPECT_EQ(cloud.cloud_.points[0].b, byte(0.7f));
}

TEST(PointCloudVizTest, AddPointStagesExactColor) {
    PointCloudViz cloud("finenav_test_cloud_exact");

    cloud.addPoint(0.0f, 0.0f, 0.0f, Color{1.0f, 0.1f, 0.1f});

    ASSERT_EQ(cloud.cloud_.points.size(), 1U);
    EXPECT_EQ(cloud.cloud_.points[0].r, byte(1.0f));
    EXPECT_EQ(cloud.cloud_.points[0].g, byte(0.1f));
    EXPECT_EQ(cloud.cloud_.points[0].b, byte(0.1f));
}

// The scale is a property of the dataset, not of the point: the same object is
// handed to each call, and the colour staged is the one it answers with.
TEST(PointCloudVizTest, AddScalarPointUsesTheScale) {
    PointCloudViz cloud("finenav_test_cloud_scalar");
    const ColorScale scale{0.0, 1.0, ColorName::BLACK, ColorName::WHITE};

    cloud.addScalarPoint(0.0f, 0.0f, 0.0f, 1.0, scale);

    ASSERT_EQ(cloud.cloud_.points.size(), 1U);
    EXPECT_EQ(cloud.cloud_.points[0].r, byte(1.0f));
    EXPECT_EQ(cloud.cloud_.points[0].g, byte(1.0f));
    EXPECT_EQ(cloud.cloud_.points[0].b, byte(1.0f));
}

TEST(PointCloudVizTest, ReserveGrowsCapacityWithoutStagingPoints) {
    PointCloudViz cloud("t");

    cloud.reserve(100);

    EXPECT_TRUE(cloud.cloud_.empty());
    EXPECT_GE(cloud.cloud_.points.capacity(), 100U);
}

TEST(PointCloudVizTest, ClearEmptiesTheBuffer) {
    PointCloudViz cloud("finenav_test_cloud_clear");

    cloud.addPoint(1.0f, 2.0f, 3.0f);
    ASSERT_EQ(cloud.cloud_.points.size(), 1U);

    cloud.clear();

    EXPECT_TRUE(cloud.cloud_.points.empty());
}
