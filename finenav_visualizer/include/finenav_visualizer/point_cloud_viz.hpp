// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "finenav_visualizer/color.hpp"

namespace finenav::viz {

class PointCloudViz {
public:
    explicit PointCloudViz(rclcpp::Node::SharedPtr node);

    void setBaseFrame(const std::string& frame) { base_frame_ = frame; }
    const std::string& baseFrame() const { return base_frame_; }

    void reserve(size_t n);

    // ── Add points (accumulate, no publish until publish()) ──────────

    /// Add a point with a preset color name.
    void addPoint(float x, float y, float z, ColorName c);

    /// Add a point with raw RGB bytes (0–255).
    void addPoint(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);

    /// Add a point colored by a scalar value (e.g. height, cost).
    /// @param vmin, vmax — value range for the gradient.
    void addColoredPoint(float x, float y, float z, double value,
                         double vmin, double vmax,
                         ColorName low = ColorName::BLUE,
                         ColorName high = ColorName::RED);

    // ── Batch control ──────────────────────────────────────────────

    /// Publish and clear accumulated points.
    void publish(const std::string& topic = "finenav_cloud");

    /// Discard accumulated points without publishing.
    void clear();

    /// Check for subscribers (avoid needless computation).
    bool hasSubscribers(const std::string& topic) const;

private:
    rclcpp::Node::SharedPtr node_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    // No default frame: the framework does not choose one for the visualizer,
    // the caller supplies it via setBaseFrame().
    std::string base_frame_;

    std::unordered_map<std::string,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubs_;
    mutable std::shared_mutex pubs_mutex_;
};

}  // namespace finenav::viz
