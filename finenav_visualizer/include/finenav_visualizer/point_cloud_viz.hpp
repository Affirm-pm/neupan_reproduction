// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include <utility>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "finenav_visualizer/color.hpp"
#include "finenav_visualizer/viz_channel.hpp"

namespace finenav::viz {

/// Staging buffer for a coloured point cloud — somewhere to accumulate points,
/// not a way to publish them.  Same contract as MarkerViz: the public surface
/// stages, and nothing else; publishing is reached only through
/// FineNavVisualizer::publish().
///
/// Each point takes the channel's mutex, and addScalarPoint takes it once for
/// the whole of what it does — the mix and the stage — so a caller's call is
/// atomic against another thread staging into the same cloud.
///
/// The frame is chosen when the buffer is published, not when it is created
/// (see VizChannel) — and here it is not merely a convention: a PointCloud2
/// carries exactly one header.frame_id, so all the points staged in one buffer
/// must genuinely live in the same frame, and the publish is where that is
/// asserted.
///
/// Publishes to the topic it was created with, as sensor_msgs/PointCloud2.
class PointCloudViz : public TypedChannel<sensor_msgs::msg::PointCloud2> {
public:
    explicit PointCloudViz(std::string topic)
        : TypedChannel<sensor_msgs::msg::PointCloud2>(std::move(topic)) {}

    /// Promise room for @p n points.  Purely an optimisation and never
    /// required — a buffer grows on its own — so it is here for the one caller
    /// that already knows the count, and nothing else depends on it.
    void reserve(size_t n) { cloud_.reserve(n); }

    // ── Points (accumulate; nothing leaves until publish()) ──────────

    /// Add a point in the given color — named or computed, one type either way.
    /// Alpha is not carried into the cloud: a PointCloud2 holds RGB, so a
    /// translucent Color is drawn opaque here.  Use a marker when the alpha
    /// matters.
    void addPoint(float x, float y, float z, const Color& color = ColorName::DEFAULT);

    /// Add a point colored by a scalar value (height, cost, …) through a
    /// ColorScale.  The scale — range plus colour band — is a property of the
    /// dataset, so it is built once and reused; each point names only its value.
    void addScalarPoint(float x, float y, float z, double value, const ColorScale& scale);

private:
    /// The staged points AS a message, stamped — and handed over.  The points
    /// genuinely cannot move into the wire form (PCL's own conversion rewrites
    /// them, field by field, into PointCloud2's layout), so this is the one
    /// conversion that must copy.  What it does not do is make that copy twice:
    /// the source is emptied here, in the same step, so the buffer behind it
    /// keeps only its capacity for the next cycle.
    ///
    /// Called with mutex_ held (from publishStaged), so it locks nothing itself.
    sensor_msgs::msg::PointCloud2 takeMessage(common::Frame frame,
                                              const rclcpp::Time& stamp) override;

    bool empty() const override { return cloud_.empty(); }

    void clearUnlocked() override { cloud_.clear(); }

    /// addPoint()'s body, without the lock: addScalarPoint() mixes a colour and
    /// then stages exactly one point, so the mix and the stage are one step under
    /// one lock rather than a lock each.  Called with mutex_ held.
    void addPointUnlocked(float x, float y, float z, const Color& color);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
};

}  // namespace finenav::viz
