// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/point_cloud_viz.hpp"

#include <cmath>

#include <pcl_conversions/pcl_conversions.h>

namespace finenav::viz {

// ── Basic addPoint overloads ────────────────────────────────────────

void PointCloudViz::addPoint(float x, float y, float z, ColorName c) {
    const auto color = getColor(c);
    addPoint(x, y, z,
             static_cast<uint8_t>(color.r * 255),
             static_cast<uint8_t>(color.g * 255),
             static_cast<uint8_t>(color.b * 255));
}

void PointCloudViz::addPoint(float x, float y, float z,
                              uint8_t r, uint8_t g, uint8_t b) {
    pcl::PointXYZRGB pt;
    pt.x = x; pt.y = y; pt.z = z;
    pt.r = r; pt.g = g; pt.b = b;
    cloud_.push_back(pt);
}

// ── Value-colored point ─────────────────────────────────────────────

void PointCloudViz::addColoredPoint(float x, float y, float z, double value,
                                     double vmin, double vmax,
                                     ColorName low, ColorName high) {
    const double range = vmax - vmin;
    const double t = (range > 0.0) ? std::clamp((value - vmin) / range, 0.0, 1.0) : 0.5;

    const auto c_lo = getColor(low);
    const auto c_hi = getColor(high);

    const float r = static_cast<float>(c_lo.r + (c_hi.r - c_lo.r) * t);
    const float g = static_cast<float>(c_lo.g + (c_hi.g - c_lo.g) * t);
    const float b = static_cast<float>(c_lo.b + (c_hi.b - c_lo.b) * t);

    addPoint(x, y, z,
             static_cast<uint8_t>(r * 255),
             static_cast<uint8_t>(g * 255),
             static_cast<uint8_t>(b * 255));
}

// ── Batch control ──────────────────────────────────────────────────

sensor_msgs::msg::PointCloud2 PointCloudViz::buildMessage(const rclcpp::Time& stamp) const {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud_, msg);
    msg.header.frame_id = frame_;
    msg.header.stamp = stamp;
    return msg;
}

}  // namespace finenav::viz
