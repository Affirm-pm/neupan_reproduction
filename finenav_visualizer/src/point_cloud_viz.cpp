// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/point_cloud_viz.hpp"

#include <cmath>

#include <pcl_conversions/pcl_conversions.h>

namespace finenav::viz {

PointCloudViz::PointCloudViz(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void PointCloudViz::reserve(size_t n) { cloud_.reserve(n); }

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

void PointCloudViz::clear() { cloud_.clear(); }

bool PointCloudViz::hasSubscribers(const std::string& topic) const {
    std::shared_lock lock(pubs_mutex_);
    auto it = pubs_.find(topic);
    if (it == pubs_.end()) return false;
    return it->second->get_subscription_count() > 0;
}

void PointCloudViz::publish(const std::string& topic) {
    if (cloud_.empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud_, msg);
    msg.header.frame_id = base_frame_;
    msg.header.stamp = node_->get_clock()->now();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    {
        std::shared_lock lock(pubs_mutex_);
        auto it = pubs_.find(topic);
        if (it != pubs_.end()) pub = it->second;
    }
    if (!pub) {
        std::unique_lock lock(pubs_mutex_);
        auto& p = pubs_[topic];
        if (!p) p = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);
        pub = p;
    }
    pub->publish(msg);
    cloud_.clear();
}

}  // namespace finenav::viz
