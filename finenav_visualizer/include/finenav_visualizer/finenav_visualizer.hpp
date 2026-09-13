// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

namespace rclcpp {
class Node;
}  // namespace rclcpp

namespace finenav::viz {

class MarkerViz;
class PointCloudViz;

/// Value-semantic handle to the framework's visualization backend.
///
/// It separates the two things that used to be tangled:
///
///   * the CHANNELS — markers() and pointCloud() — are pure staging buffers,
///     each a VizChannel.  You add shapes to them and set the frame those
///     shapes live in; nothing is published and nothing leaves the process.  A
///     channel has no publish of its own on purpose: reading an algorithm then
///     tells you where its data goes — nowhere, until publish() is called.
///   * publish() is the single exit, and it names the topic.  It walks the
///     channels and, for each, sends what is staged and then clears it — staged
///     data does not cross a publish boundary.
///
/// The backend sits behind a pimpl, and here that is load-bearing rather than
/// stylistic: setVisualizer() takes the handle and stores it, so COPYING the
/// handle must SHARE the channels and their publishers, not duplicate them.
/// Held by value, every copy the framework hands to an algorithm would carry
/// its own buffers and its own publishers on the same topics — two states
/// drifting apart, with each side blind to the other's staging.  Sharing is
/// also why it is a handle at all: algorithms take it by value in
/// setVisualizer() — symmetric with setLogger — and never store a raw pointer
/// into the framework.
///
/// The channel types are declared here, not included: include
/// finenav_visualizer/marker_viz.hpp or point_cloud_viz.hpp to stage into one.
/// A plugin that only receives the handle in setVisualizer() — the common case
/// — pays for neither.
class FineNavVisualizer {
public:
    /// A handle with no backend: it was built without a node, so there is
    /// nowhere to publish to.
    ///
    /// It exists for one reason: an algorithm holds the handle as a MEMBER, and
    /// C++ requires that member to be constructible before setVisualizer() can
    /// assign to it.  Such a handle is a working object, just an inert one —
    /// staging goes nowhere and publish() does nothing — so no caller ever has
    /// to test for it.  The framework is the only producer of a handle with a
    /// backend, and it fills in the one it hands out before any algorithm code
    /// runs.
    FineNavVisualizer();

    explicit FineNavVisualizer(std::shared_ptr<rclcpp::Node> node);

    ~FineNavVisualizer();
    FineNavVisualizer(const FineNavVisualizer&) = default;
    FineNavVisualizer& operator=(const FineNavVisualizer&) = default;
    FineNavVisualizer(FineNavVisualizer&&) = default;
    FineNavVisualizer& operator=(FineNavVisualizer&&) = default;

    // ── Staging channels — add shapes, set their frame ────────────────

    /// Rigid-body markers → @c topic_name @c + @c "/markers".
    /// Set the frame of what you stage with markers().setFrame(...) — a Frame
    /// tag (finenav_common/frames.hpp), resolved at publish time.
    MarkerViz&     markers();

    /// Coloured point cloud → @c topic_name @c + @c "/cloud".
    PointCloudViz& pointCloud();

    // ── Publish — the only exit ───────────────────────────────────────

    /// Send everything staged on every channel, then clear every channel.
    ///
    /// Each channel appends its own sub-topic, because they carry different
    /// message types and may carry different frames:
    ///     markers()    → @p topic_name + "/markers"   (visualization_msgs/MarkerArray)
    ///     pointCloud() → @p topic_name + "/cloud"     (sensor_msgs/PointCloud2)
    /// A channel with nothing staged is skipped — no message, and no publisher
    /// is created for it.  Every channel is cleared either way, so after this
    /// call nothing is staged anywhere.
    ///
    /// On a handle with no backend (see the default constructor) the send is
    /// skipped for the same reason it is for an empty channel — there is
    /// nowhere to send to — but the clear still happens, so staging is consumed
    /// in every state.
    void publish(const std::string& topic_name);

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}  // namespace finenav::viz
