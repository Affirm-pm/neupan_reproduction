// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "finenav_common/frames.hpp"

namespace rclcpp {
class Node;
}  // namespace rclcpp

namespace finenav::viz {

class MarkerViz;
class PointCloudViz;

/// Value-semantic handle to the framework's visualization backend, and a
/// FACTORY of channels.
///
/// It separates the three things that used to be tangled:
///
///   * a CHANNEL is pure staging buffer — somewhere to accumulate shapes, not a
///     way to publish them.  You ask for one with markers() or pointCloud(),
///     naming the topic it publishes on; you then add shapes to it, and nothing
///     is published and nothing leaves the process.  A channel has no publish of
///     its own on purpose: reading an algorithm then tells you where its data
///     goes — nowhere, until one of the publish() overloads is called.
///   * the TOPIC is the channel's identity, and it is the exact ROS topic: one
///     channel is one topic, nothing is appended to the name you give.  Asking
///     for the same topic twice returns the SAME channel — the same buffers and
///     the same publisher.  A topic is therefore fixed to one message type
///     (markers() and pointCloud() are separate maps); the FRAME is not fixed,
///     because a frame belongs to the data rather than to the buffer — the same
///     channel can carry a world-referenced path one cycle and a body-referenced
///     footprint the next.
///   * publish() is the single exit, and it takes the CHANNEL and the FRAME.
///     It sends what that one channel has staged, stamped with that frame, and
///     then clears it — staged data does not cross a publish boundary.  clear()
///     is the other half of the boundary: it discards without sending.
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
/// THREAD SAFETY: every channel is guarded by its own mutex and the factory by
/// another, so staging and publishing are safe from any thread — including
/// several threads staging into ONE topic, which is the shape a parallel
/// trajectory sampler has.  Each public call is atomic and whole: an add* that
/// builds several markers (addPath, addAxis) lands as the one picture it was
/// asked for, a publish takes its message and drains the channel in one step,
/// and a clear discards in one step.  What the framework does NOT provide is a
/// cross-call transaction — if you need "publish only after every worker has
/// staged this cycle", that barrier is yours to place, exactly as it is with any
/// thread-safe publisher.
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

    // ── Channels — created by topic, never replaced ───────────────────

    /// The marker channel on @p topic — rigid-body markers, published as
    /// visualization_msgs/MarkerArray.
    ///
    /// @p topic is the channel's identity AND the exact ROS topic it lands on:
    /// nothing is appended.  The same topic always returns the same channel, so
    /// a caller may hold the reference; it stays valid for the life of the
    /// backend.
    MarkerViz& markers(const std::string& topic);

    /// The point-cloud channel on @p topic — a coloured cloud, published as
    /// sensor_msgs/PointCloud2.  Same contract as markers().
    PointCloudViz& pointCloud(const std::string& topic);

    // ── Publish / clear — the only exits, one channel at a time ───────

    /// Send what @p channel has staged, stamped with @p frame, then clear it.
    ///
    /// The frame is a publish-time argument because it belongs to the DATA, and
    /// only the algorithm knows whether what it just staged is world-referenced
    /// (a global path, a costmap, a goal) or body-referenced (a footprint, a
    /// velocity arrow).  It is a Frame tag (finenav_common/frames.hpp), resolved
    /// to a name on the way out — so a system that renamed the frames at startup
    /// gets its own names, and no spelling of a frame id exists to get wrong.
    /// A channel that carries world-referenced data almost always can leave the
    /// default alone; robot-frame data passes Frame::Robot.
    ///
    /// A channel with nothing staged is skipped — no message, and no publisher
    /// is created for it — but it is cleared either way, so after this call
    /// nothing is staged there.  Publishing is the boundary: what was staged
    /// does not survive it, so a cycle that forgets to re-stage shows up as
    /// nothing rather than as last cycle's leftovers.
    ///
    /// On a handle with no backend (see the default constructor) the send is
    /// skipped for the same reason it is for an empty channel — there is
    /// nowhere to send to — but the clear still happens, so staging is consumed
    /// in every state.
    void publish(MarkerViz& channel, common::Frame frame = common::Frame::World);
    void publish(PointCloudViz& channel, common::Frame frame = common::Frame::World);

    /// Discard what @p channel has staged, publishing nothing.
    void clear(MarkerViz& channel);
    void clear(PointCloudViz& channel);

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}  // namespace finenav::viz
