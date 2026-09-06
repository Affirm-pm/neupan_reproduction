// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "finenav_visualizer/color.hpp"
#include "finenav_visualizer/marker_viz.hpp"
#include "finenav_visualizer/point_cloud_viz.hpp"

namespace finenav::viz {

/// Value-semantic handle to the framework's visualization backend.  Copying it
/// shares the same backend (like std::shared_ptr), so algorithms copy it by
/// value in setVisualizer() — symmetric with setLogger — and never store a raw
/// pointer into the framework.
class FineNavVisualizer {
public:
    /// Empty handle; the framework assigns a real one via setVisualizer().
    FineNavVisualizer() = default;

    FineNavVisualizer(rclcpp::Node::SharedPtr node,
                      const std::string& topic_name = "finenav_viz");

    ~FineNavVisualizer();
    FineNavVisualizer(const FineNavVisualizer&) = default;
    FineNavVisualizer& operator=(const FineNavVisualizer&) = default;
    FineNavVisualizer(FineNavVisualizer&&) = default;
    FineNavVisualizer& operator=(FineNavVisualizer&&) = default;

    /// True once a backend has been injected.
    explicit operator bool() const { return static_cast<bool>(impl_); }

    // ── Channels (public, for individual addXxx / publish) ─────────

    MarkerViz&     markers();
    PointCloudViz& pointCloud();

    // ── Topic configuration ───────────────────────────────────────

    /// Set the base topic name.  Markers → <name>/markers, cloud → <name>/cloud.
    void setTopicName(const std::string& n);

    std::string markerTopic() const;
    std::string cloudTopic()  const;

    // ── Publish ───────────────────────────────────────────────────

    void publishAll();

    rclcpp::Node::SharedPtr node() const;

    // Future: framework-specific visualizations based on the channels.

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}  // namespace finenav::viz
