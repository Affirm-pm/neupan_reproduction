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

class FineNavVisualizer {
public:
    FineNavVisualizer(rclcpp::Node::SharedPtr node,
                      const std::string& topic_name = "finenav_viz");

    // ── Channels (public, for individual addXxx / publish) ─────────

    MarkerViz&     markers()    { return marker_viz_; }
    PointCloudViz& pointCloud() { return cloud_viz_; }

    // ── Topic configuration ───────────────────────────────────────

    /// Set the base topic name.  Markers → <name>/markers, cloud → <name>/cloud.
    void setTopicName(const std::string& n) { topic_name_ = n; }

    std::string markerTopic() const { return topic_name_ + "/markers"; }
    std::string cloudTopic()  const { return topic_name_ + "/cloud";  }

    // ── Publish ───────────────────────────────────────────────────

    void publishAll() {
        marker_viz_.publish(markerTopic());
        cloud_viz_.publish(cloudTopic());
    }

    rclcpp::Node::SharedPtr node() const { return node_; }

    // Future: framework-specific visualizations based on the channels.

private:
    rclcpp::Node::SharedPtr node_;
    MarkerViz     marker_viz_;
    PointCloudViz cloud_viz_;
    std::string   topic_name_;
};

}  // namespace finenav::viz
