// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/finenav_visualizer.hpp"

#include <utility>

namespace finenav::viz {

struct FineNavVisualizer::Impl {
    Impl(rclcpp::Node::SharedPtr n, std::string topic)
        : node_(std::move(n)),
          marker_viz_(node_),
          cloud_viz_(node_),
          topic_name_(std::move(topic)) {}

    rclcpp::Node::SharedPtr node_;
    MarkerViz     marker_viz_;
    PointCloudViz cloud_viz_;
    std::string   topic_name_;
};

FineNavVisualizer::FineNavVisualizer(rclcpp::Node::SharedPtr node,
                                     const std::string& topic_name)
    : impl_(std::make_shared<Impl>(node, topic_name)) {}

FineNavVisualizer::~FineNavVisualizer() = default;

MarkerViz& FineNavVisualizer::markers() { return impl_->marker_viz_; }
PointCloudViz& FineNavVisualizer::pointCloud() { return impl_->cloud_viz_; }

void FineNavVisualizer::setTopicName(const std::string& n) { impl_->topic_name_ = n; }

std::string FineNavVisualizer::markerTopic() const { return impl_->topic_name_ + "/markers"; }
std::string FineNavVisualizer::cloudTopic()  const { return impl_->topic_name_ + "/cloud";  }

void FineNavVisualizer::publishAll() {
    impl_->marker_viz_.publish(markerTopic());
    impl_->cloud_viz_.publish(cloudTopic());
}

rclcpp::Node::SharedPtr FineNavVisualizer::node() const { return impl_->node_; }

}  // namespace finenav::viz
