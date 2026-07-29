// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/finenav_visualizer.hpp"

namespace finenav::viz {

FineNavVisualizer::FineNavVisualizer(rclcpp::Node::SharedPtr node,
                                     const std::string& topic_name)
    : node_(std::move(node)), marker_viz_(node_), cloud_viz_(node_),
      topic_name_(topic_name) {}

}  // namespace finenav::viz
