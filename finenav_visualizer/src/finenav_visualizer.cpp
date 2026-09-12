// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/finenav_visualizer.hpp"

#include <array>
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

    /// Every channel, as the one interface they share.  The only place that
    /// knows how many there are — publish() walks this, so a new channel needs
    /// no change here beyond the array.
    std::array<VizChannel*, 2> channels() {
        return {&marker_viz_, &cloud_viz_};
    }

    rclcpp::Node::SharedPtr node_;
    MarkerViz     marker_viz_;
    PointCloudViz cloud_viz_;
};

FineNavVisualizer::FineNavVisualizer()
    : impl_(std::make_shared<Impl>()) {}

FineNavVisualizer::FineNavVisualizer(rclcpp::Node::SharedPtr node)
    : impl_(std::make_shared<Impl>(std::move(node))) {}

FineNavVisualizer::~FineNavVisualizer() = default;

MarkerViz& FineNavVisualizer::markers() {
    return impl_->marker_viz_;
}

PointCloudViz& FineNavVisualizer::pointCloud() {
    return impl_->cloud_viz_;
}

void FineNavVisualizer::publish(const std::string& topic_name) {
    // No backend means no node to publish through.  It is not an error state —
    // it is the state a member sits in until setVisualizer() fills it — so the
    // send is simply skipped, exactly as it is for an empty channel.
    rclcpp::Node* node = impl_->node_.get();
    const rclcpp::Time now = node ? node->get_clock()->now() : rclcpp::Time{};

    for (VizChannel* channel : impl_->channels()) {
        if (node) {
            channel->publishStaged(*node, topic_name, now);
        }
        // Publishing is the boundary: what was staged does not survive it, so a
        // cycle that forgets to re-stage shows up as nothing rather than as
        // last cycle's leftovers.  That holds with no backend too, which is why
        // the clear sits outside the branch above.
        channel->clear();
    }
}

}  // namespace finenav::viz
