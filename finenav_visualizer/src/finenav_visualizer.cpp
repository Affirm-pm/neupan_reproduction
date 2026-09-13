// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/finenav_visualizer.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include <rclcpp/node.hpp>

#include "finenav_visualizer/marker_viz.hpp"
#include "finenav_visualizer/point_cloud_viz.hpp"
#include "finenav_visualizer/viz_channel.hpp"

namespace finenav::viz {

struct FineNavVisualizer::Impl {
    /// A null node IS the inert state: channels exist and accept staging, but
    /// there is no node to create a publisher on, so publish() has nowhere to
    /// send.  That is the whole of what "no backend" means — no flag to keep in
    /// sync with node_.
    explicit Impl(rclcpp::Node::SharedPtr n = nullptr) : node_(std::move(n)) {}

    /// Every channel, keyed by the topic that IS its identity — one map per
    /// message type, so a topic names a channel unambiguously.
    ///
    /// Unique-ptr, not value: a channel is handed out by reference and lives as
    /// long as the backend, so the object a caller holds must not move.  A map
    /// that rehashed its elements would invalidate it; pointing at them cannot.
    std::unordered_map<std::string, std::unique_ptr<MarkerViz>> marker_channels_;
    std::unordered_map<std::string, std::unique_ptr<PointCloudViz>> cloud_channels_;

    /// Guards both maps.  One mutex, not two: the two are looked up by the same
    /// call shapes and are never held against each other, so a second would only
    /// be a second thing to get wrong.  A channel found here is then locked by its
    /// OWN mutex for staging — this one is released as soon as the reference is
    /// handed back, so holding it never blocks a publish.
    std::mutex channels_mutex_;

    rclcpp::Node::SharedPtr node_;
};

FineNavVisualizer::FineNavVisualizer()
    : impl_(std::make_shared<Impl>()) {}

FineNavVisualizer::FineNavVisualizer(rclcpp::Node::SharedPtr node)
    : impl_(std::make_shared<Impl>(std::move(node))) {}

FineNavVisualizer::~FineNavVisualizer() = default;

MarkerViz& FineNavVisualizer::markers(const std::string& topic) {
    std::lock_guard<std::mutex> lock(impl_->channels_mutex_);
    auto it = impl_->marker_channels_.find(topic);
    if (it == impl_->marker_channels_.end()) {
        it = impl_->marker_channels_.emplace(topic, std::make_unique<MarkerViz>(topic)).first;
    }
    // The reference outlives the lock: the channel is heap-allocated, so it stays
    // where it is however the map rehashes around it.
    return *it->second;
}

PointCloudViz& FineNavVisualizer::pointCloud(const std::string& topic) {
    std::lock_guard<std::mutex> lock(impl_->channels_mutex_);
    auto it = impl_->cloud_channels_.find(topic);
    if (it == impl_->cloud_channels_.end()) {
        it = impl_->cloud_channels_.emplace(topic, std::make_unique<PointCloudViz>(topic)).first;
    }
    return *it->second;
}

void FineNavVisualizer::publish(MarkerViz& channel, common::Frame frame) {
    // Bound to VizChannel on purpose: publishStaged() and clear() are declared
    // protected there and narrowed to private again by each concrete channel,
    // and friendship names VizChannel alone — so the channel driven here is the
    // one declaration the visualizer is a friend of.  It is the same object
    // either way; the reference only says which declaration is being reached.
    VizChannel& viz = channel;
    // No backend means no node to publish through.  It is not an error state —
    // it is the state a member sits in until setVisualizer() fills it — so the
    // send is simply skipped; the staged data is still consumed, exactly as a
    // real publish consumes it.
    if (rclcpp::Node* node = impl_->node_.get()) {
        viz.publishStaged(*node, frame, node->get_clock()->now());  // drains internally
    } else {
        viz.clear();
    }
}

void FineNavVisualizer::publish(PointCloudViz& channel, common::Frame frame) {
    VizChannel& viz = channel;
    if (rclcpp::Node* node = impl_->node_.get()) {
        viz.publishStaged(*node, frame, node->get_clock()->now());  // drains internally
    } else {
        viz.clear();
    }
}

void FineNavVisualizer::clear(MarkerViz& channel) {
    static_cast<VizChannel&>(channel).clear();
}

void FineNavVisualizer::clear(PointCloudViz& channel) {
    static_cast<VizChannel&>(channel).clear();
}

}  // namespace finenav::viz
