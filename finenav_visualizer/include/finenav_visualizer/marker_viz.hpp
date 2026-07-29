// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace finenav::viz {

class MarkerViz {
public:
    explicit MarkerViz(rclcpp::Node::SharedPtr node);

    void setBaseFrame(const std::string& frame) { base_frame_ = frame; }
    const std::string& baseFrame() const { return base_frame_; }

    void setLifetime(double sec);
    void setAlpha(float a) { alpha_ = a; }

    // ── Shapes (accumulate, no publish until publish()) ───────────────

    void addSphere(const Eigen::Vector3d& center, double radius,
                   const std_msgs::msg::ColorRGBA& color);

    void addSpheres(const std::vector<Eigen::Vector3d>& centers,
                    double radius, const std::vector<std_msgs::msg::ColorRGBA>& colors);

    void addArrow(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                  const std_msgs::msg::ColorRGBA& color,
                  double shaft_d = 0.02, double head_d = 0.06, double head_len = 0.08);

    void addCuboid(const Eigen::Isometry3d& pose, double depth, double width, double height,
                   const std_msgs::msg::ColorRGBA& color);

    void addWireframeCuboid(const Eigen::Isometry3d& pose, double depth, double width,
                            double height, const std_msgs::msg::ColorRGBA& color);

    /// Cone along +Z (body-frame sensor axis), opening angle from Z-axis.
    void addCone(const Eigen::Isometry3d& pose, double angle, double max_range,
                 const std_msgs::msg::ColorRGBA& color);

    /// Cylinder from p1 to p2.
    void addCylinder(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                     double radius, const std_msgs::msg::ColorRGBA& color);

    void addLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                 const std_msgs::msg::ColorRGBA& color, double width = 0.01);

    /// Connected cylinders along points.
    void addPath(const std::vector<Eigen::Vector3d>& points,
                 const std_msgs::msg::ColorRGBA& color, double radius = 0.02);

    /// RGB axis (X=red, Y=green, Z=blue).
    void addAxis(const Eigen::Isometry3d& pose, double length = 0.3, double radius = 0.01);

    void addText(const Eigen::Vector3d& pos, const std::string& text,
                 const std_msgs::msg::ColorRGBA& color, double height = 0.1);

    // ── Batch control ──────────────────────────────────────────────

    /// Publish and clear accumulated markers.
    void publish(const std::string& topic = "finenav_visualization");

    /// Discard accumulated markers without publishing.
    void clear() { markers_.markers.clear(); }

    /// Delete all current markers from RViz.
    void deleteAllMarkers(const std::string& topic = "finenav_visualization");

    /// Check for subscribers (avoid needless computation).
    bool hasSubscribers(const std::string& topic) const;

private:
    rclcpp::Node::SharedPtr node_;
    visualization_msgs::msg::MarkerArray markers_;
    std::string base_frame_ = "map";
    float alpha_ = 1.0f;

    std::unordered_map<std::string,
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> pubs_;
    mutable std::shared_mutex pubs_mutex_;
};

}  // namespace finenav::viz
