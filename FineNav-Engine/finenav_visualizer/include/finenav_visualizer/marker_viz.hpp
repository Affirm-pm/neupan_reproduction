// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "finenav_visualizer/color.hpp"
#include "finenav_visualizer/detail/viz_channel.hpp"

namespace finenav::viz {

/// Staging buffer for rigid-body markers — somewhere to accumulate shapes, not
/// a way to publish them.  Its public surface stages, and nothing else; the
/// publisher it caches comes from detail::TypedChannel, and the only thing that reaches it is
/// FineNavVisualizer::publish().  So reading an algorithm tells you exactly
/// which topic its data reaches, and a buffer that is never published costs
/// nothing but memory.
///
/// Each shape takes the channel's mutex for its whole body, so a caller's add* is
/// atomic: a shape lands complete, and a composite (addPath, addAxis) lands as the
/// one picture it is meant to be rather than as pieces another thread could
/// interleave its own shapes into.
///
/// The frame is chosen when the buffer is published, not when it is created: a
/// frame belongs to the DATA, and only the algorithm knows whether what it
/// staged is world-referenced (a global path, a costmap, a goal) or
/// body-referenced (a footprint, a velocity arrow) — so it is said at the
/// publish, where the data is being handed over.  One buffer is still one
/// message, and a message has one frame.
///
/// Every shape takes its colour as Color, alpha included — a marker's
/// transparency is a property of the colour it is drawn in, so there is no
/// separate buffer-wide alpha to combine with it.
///
/// A shape lasts until the next publish replaces it.  There is no lifetime
/// knob: what an algorithm stages in one cycle IS the picture, and anything it
/// stops staging must disappear rather than linger — so every message leads
/// with a DELETEALL and the viewer is reset before the shapes arrive.  Nothing
/// here decides when a shape goes away; the next call to publish() does.
///
/// Publishes to the topic it was created with, as visualization_msgs/MarkerArray.
class MarkerViz : public detail::TypedChannel<visualization_msgs::msg::MarkerArray> {
public:
    explicit MarkerViz(std::string topic)
        : detail::TypedChannel<visualization_msgs::msg::MarkerArray>(std::move(topic)) {}

    // ── Shapes (accumulate; nothing leaves until publish()) ───────────
    //
    // One order everywhere: what defines the shape (required), then what has a
    // natural default (stroke width, font size), then the color — last, and
    // itself defaulted to grey.  So a call can always stop early, and the color
    // is the one argument that is never mistaken for a size.
    //
    // C++ has no named arguments, so the order has a cost worth knowing: naming
    // a color means also passing the defaulted sizes ahead of it.
    //
    // Every shape is stamped with the namespace of its kind (Sphere, Arrow,
    // Cuboid, …), which is how the viewer groups and toggles them.

    void addSphere(const Eigen::Vector3d& center, double radius,
                   const Color& color = ColorName::DEFAULT);

    void addArrow(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                  double shaft_d = 0.02, double head_d = 0.06, double head_len = 0.08,
                  const Color& color = ColorName::DEFAULT);

    void addCuboid(const Eigen::Isometry3d& pose, double depth, double width, double height,
                   const Color& color = ColorName::DEFAULT);

    void addWireframeCuboid(const Eigen::Isometry3d& pose, double depth, double width,
                            double height, const Color& color = ColorName::DEFAULT);

    /// Cone along +Z (body-frame sensor axis), opening angle from Z-axis.
    void addCone(const Eigen::Isometry3d& pose, double angle, double max_range,
                 const Color& color = ColorName::DEFAULT);

    /// Cylinder from p1 to p2.
    void addCylinder(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                     double radius = 0.02, const Color& color = ColorName::DEFAULT);

    void addLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                 double width = 0.02, const Color& color = ColorName::DEFAULT);

    /// Connected line strip represented by one Marker.
    void addLineStrip(const std::vector<Eigen::Vector3d>& points,
                      double width = 0.02,
                      const Color& color = ColorName::DEFAULT);

    /// Connected cylinders along points.
    void addPath(const std::vector<Eigen::Vector3d>& points,
                 double radius = 0.02, const Color& color = ColorName::DEFAULT);

    /// RGB axis (X=red, Y=green, Z=blue) — the one shape whose color IS its
    /// meaning, so it takes no color argument.
    void addAxis(const Eigen::Isometry3d& pose, double length = 0.3, double radius = 0.02);

    void addText(const Eigen::Vector3d& pos, const std::string& text,
                 double height = 0.1, const Color& color = ColorName::DEFAULT);

private:
    // ── Staging buffer — machinery, reached only through detail::VizChannel ──

    /// The staged markers as a message, stamped — and handed over: the array
    /// and every point vector in it move into the message, so a publish copies
    /// nothing.  Ids restart at 0 behind it, so the same shapes staged twice
    /// produce the same message twice; the viewer is cleaned up by the DELETEALL
    /// that leads the message, not by the numbering.  Called with mutex_ held
    /// (from publishStaged), so it locks nothing itself — and the hand-over below
    /// is what drains the channel on a cycle that published.
    visualization_msgs::msg::MarkerArray takeMessage(common::Frame frame,
                                                     const rclcpp::Time& stamp) override;

    bool empty() const override { return markers_.markers.empty(); }

    void clearUnlocked() override {
        markers_.markers.clear();
        next_marker_id_ = 0;
    }

    // ── Unlocked shapes — what a composite calls without re-locking ───
    //
    // addPath() and addAxis() are one shape to a caller and many markers to the
    // viewer, and the lock has to span the whole of what they stage: a parcel of
    // arrows another thread could interleave into would not be the picture either
    // caller asked for.  So a composite holds mutex_ once and calls these, which
    // are the same code the public method runs.  No defaults here on purpose —
    // these are called from inside, where every argument is already in hand.

    void addArrowUnlocked(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                          double shaft_d, double head_d, double head_len,
                          const Color& color);

    void addCylinderUnlocked(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                             double radius, const Color& color);

    /// Next marker id.  Unlocked: called with mutex_ held, from the shape bodies.
    std::int32_t allocateId();

    visualization_msgs::msg::MarkerArray markers_;
    std::int32_t next_marker_id_{0};
};

}  // namespace finenav::viz
