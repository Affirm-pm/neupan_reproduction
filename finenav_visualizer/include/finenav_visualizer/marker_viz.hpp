// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "finenav_visualizer/viz_channel.hpp"

namespace finenav::viz {

/// Staging buffer for rigid-body markers — somewhere to accumulate shapes, not
/// a way to publish them.  Its public surface stages and clears; the publisher
/// it caches comes from TypedChannel, and the only thing that reaches it is
/// FineNavVisualizer::publish().  So reading an algorithm tells you exactly
/// which topic its data reaches, and a buffer that is never published costs
/// nothing but memory.
///
/// The frame is per buffer and the caller sets it (see VizChannel): a frame
/// belongs to the DATA, and only the algorithm knows whether what it staged is
/// world-referenced (a global path, a costmap, a goal) or body-referenced
/// (a footprint, a velocity arrow).  Mixing the two in one buffer is not
/// expressible, by construction — one buffer is one message, and a message has
/// one frame.
///
/// Publishes to `<topic_name>/markers` as visualization_msgs/MarkerArray.
class MarkerViz : public TypedChannel<visualization_msgs::msg::MarkerArray> {
public:
    MarkerViz() = default;

    void setLifetime(double sec);
    void setAlpha(float a) { alpha_ = a; }

    // ── Shapes (accumulate; nothing leaves until publish()) ───────────

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

    /// Connected line strip represented by one Marker.
    void addLineStrip(const std::vector<Eigen::Vector3d>& points,
                      const std_msgs::msg::ColorRGBA& color,
                      double width = 0.02,
                      const std::string& ns = "LineStrip");

    /// Connected cylinders along points.
    void addPath(const std::vector<Eigen::Vector3d>& points,
                 const std_msgs::msg::ColorRGBA& color, double radius = 0.02);

    /// RGB axis (X=red, Y=green, Z=blue).
    void addAxis(const Eigen::Isometry3d& pose, double length = 0.3, double radius = 0.01);

    void addText(const Eigen::Vector3d& pos, const std::string& text,
                 const std_msgs::msg::ColorRGBA& color, double height = 0.1);

    // ── Staging buffer ─────────────────────────────────────────────

    bool empty() const override { return markers_.markers.empty(); }

    /// Discard accumulated markers without publishing.  Ids restart at 0, so
    /// the next cycle overwrites this cycle's shapes in the viewer instead of
    /// accumulating beside them.
    void clear() override {
        markers_.markers.clear();
        next_marker_id_ = 0;
    }

private:
    std::string subtopic() const override { return "/markers"; }

    visualization_msgs::msg::MarkerArray buildMessage(const rclcpp::Time& stamp) const override;

    std::int32_t allocateId();

    visualization_msgs::msg::MarkerArray markers_;
    float alpha_ = 1.0f;
    std::int32_t next_marker_id_{0};
};

}  // namespace finenav::viz
