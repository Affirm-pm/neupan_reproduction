// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/point_cloud_viz.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <pcl_conversions/pcl_conversions.h>

namespace finenav::viz {

namespace {

/// PCL stores color as bytes; Color carries it as float channels in [0, 1].
/// This is the one place the two meet — and so the one place a channel narrows,
/// because a byte is genuinely a different representation, not a smaller float.
uint8_t toByte(float channel) {
    return static_cast<uint8_t>(std::clamp(channel, 0.0f, 1.0f) * 255.0f);
}

}  // namespace

// ── Basic addPoint ──────────────────────────────────────────────────

void PointCloudViz::addPoint(float x, float y, float z, const Color& color) {
    pcl::PointXYZRGB pt;
    pt.x = x; pt.y = y; pt.z = z;
    pt.r = toByte(color.r);
    pt.g = toByte(color.g);
    pt.b = toByte(color.b);
    cloud_.push_back(pt);
}

// ── Value-colored point ─────────────────────────────────────────────

void PointCloudViz::addColoredPoint(float x, float y, float z, double value,
                                     double vmin, double vmax,
                                     const Color& low, const Color& high) {
    const double range = vmax - vmin;
    const double clamped = (range > 0.0) ? std::clamp((value - vmin) / range, 0.0, 1.0) : 0.5;

    // The gradient position is a scalar input, not a colour channel, so it is
    // the one thing here that narrows: quantize it once, then mix in float so
    // the channels themselves convert nothing.
    const float t = static_cast<float>(clamped);

    const Color mixed{
        low.r + (high.r - low.r) * t,
        low.g + (high.g - low.g) * t,
        low.b + (high.b - low.b) * t,
    };

    addPoint(x, y, z, mixed);
}

// ── Batch control ──────────────────────────────────────────────────

sensor_msgs::msg::PointCloud2 PointCloudViz::takeMessage(const rclcpp::Time& stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud_, msg);
    msg.header.frame_id = frame_;
    msg.header.stamp = stamp;

    // Handed over: the message owns its own copy of the points, so the staging
    // buffer has nothing left to keep.  Emptying it here instead of in the
    // clear() that follows a publish is what makes that clear() a no-op on a
    // cycle that published — and the capacity survives, so the next cycle
    // stages into memory that is already there.
    cloud_.clear();
    return msg;
}

}  // namespace finenav::viz
