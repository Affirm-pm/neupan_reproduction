// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "finenav_visualizer/color.hpp"
#include "finenav_visualizer/viz_channel.hpp"

namespace finenav::viz {

/// Staging buffer for a coloured point cloud — somewhere to accumulate points,
/// not a way to publish them.  Same contract as MarkerViz: staging and clearing
/// are public, publishing is not, and the only thing that reaches the cached
/// publisher is FineNavVisualizer::publish().
///
/// The frame is per buffer and the caller sets it (see VizChannel) — and here
/// it is not merely a convention: a PointCloud2 carries exactly one
/// header.frame_id, so all the points staged in one buffer must genuinely live
/// in the same frame.
///
/// Publishes to `<topic_name>/cloud` as sensor_msgs/PointCloud2.
class PointCloudViz : public TypedChannel<sensor_msgs::msg::PointCloud2> {
public:
    PointCloudViz() = default;

    void reserve(size_t n) { cloud_.reserve(n); }

    // ── Points (accumulate; nothing leaves until publish()) ──────────

    /// Add a point in the given color — named or computed, one type either way.
    /// Alpha is not carried into the cloud: a PointCloud2 holds RGB, so a
    /// translucent Color is drawn opaque here.  Use a marker when the alpha
    /// matters.
    void addPoint(float x, float y, float z, const Color& color);

    /// Add a point colored by a scalar value (e.g. height, cost).
    /// @param vmin, vmax — value range for the gradient.
    /// @param low, high — the color at each end of the range.
    void addColoredPoint(float x, float y, float z, double value,
                         double vmin, double vmax,
                         const Color& low = ColorName::BLUE,
                         const Color& high = ColorName::RED);

    // ── Staging buffer ─────────────────────────────────────────────

    bool empty() const override { return cloud_.empty(); }

    /// Discard accumulated points without publishing.
    void clear() override { cloud_.clear(); }

private:
    std::string subtopic() const override { return "/cloud"; }

    sensor_msgs::msg::PointCloud2 buildMessage(const rclcpp::Time& stamp) const override;

    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
};

}  // namespace finenav::viz
