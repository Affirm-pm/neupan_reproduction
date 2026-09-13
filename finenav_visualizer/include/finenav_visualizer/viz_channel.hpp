// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

#include "finenav_common/frames.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::viz {

class FineNavVisualizer;

/// What every staging channel is, regardless of the shape of the data it
/// stages: the topic it lands on, whether anything is staged, how to discard it,
/// and how to send it.  FineNavVisualizer drives exactly two calls on a channel:
///
///     channel->publishStaged(node, frame, now);
///     channel->clear();
///
/// and those two cover every channel there will ever be.
///
/// Identity is fixed at construction and never moves: a channel IS its topic, so
/// there is nothing left on this interface for a caller to set.  A caller who
/// could ask whether a channel is empty, or reset one, would be writing against
/// the publish boundary rather than through it — and the boundary is what the
/// whole design is for.  Widening any of it later is source-compatible; that is
/// why it starts closed.
///
/// The FRAME is the one thing a publish brings with it, because it belongs to
/// the data rather than to the buffer the data was collected in: the same
/// channel can carry a world-referenced path one cycle and a body-referenced
/// footprint the next, and the caller says which as it publishes.
///
/// Publishing is protected and the visualizer is a friend, so a channel has no
/// publish() a caller could reach: reading an algorithm tells you where its data
/// goes — nowhere, until FineNavVisualizer::publish(channel, frame) is called.
class VizChannel {
public:
    virtual ~VizChannel() = default;

protected:
    /// A channel is its topic (see topic_), and nothing else.
    explicit VizChannel(std::string topic) : topic_(std::move(topic)) {}

    /// True when nothing is staged — the state a publish leaves the channel in.
    /// Unlocked: the caller holds mutex_.
    virtual bool empty() const = 0;

    /// Discard everything staged, publishing nothing.  Unlocked: the caller holds
    /// mutex_, which is what makes clearing the buffer and resetting the id
    /// counter one step rather than two.
    virtual void clearUnlocked() = 0;

    /// Send what is staged, if anything, on topic_, stamped with @p frame — and
    /// drain the channel, whether or not anything was sent, so that a publish
    /// boundary is ONE step.  A send followed by a separate clear() would leave a
    /// window for another thread to stage into, and that staging would then be
    /// thrown away by the clear.  Locks mutex_ itself; called only by
    /// FineNavVisualizer::publish(channel, frame).
    virtual void publishStaged(rclcpp::Node& node, common::Frame frame,
                               const rclcpp::Time& stamp) = 0;

    /// The exact ROS topic this channel publishes on — its identity, and the key
    /// FineNavVisualizer keeps it under.  Nothing is ever appended to it.
    std::string topic_;

    /// Guards everything mutable on a concrete channel: the staging buffer, the
    /// marker id counter, and the publisher cache.  Every public entry takes it,
    /// and the virtuals above — empty(), clearUnlocked(), takeMessage() — are all
    /// called with it already held.
    ///
    /// Not recursive, deliberately: a public method that needs another one's work
    /// calls the *Unlocked helper rather than re-entering, so a composite shape
    /// (addPath, addAxis) is one lock from its first marker to its last instead of
    /// one lock per sub-shape.
    std::mutex mutex_;

private:
    friend class FineNavVisualizer;

    /// Locked wrapper: discard everything staged, publishing nothing.  The
    /// visualizer reaches this through friendship; a test reaches it through the
    /// private->public hack.  The unlocked work is clearUnlocked().
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        clearUnlocked();
    }
};

/// The layer that answers "a channel of MsgT": the lazily-created publisher and the
/// empty-check live here, once, so neither the visualizer nor a concrete channel
/// repeats them.
///
/// A concrete channel is then only what is genuinely its own — the message it
/// shapes; the topic it lands on is the one it was created with.
///
/// A send also depends on someone listening: with no subscriber, the message is
/// built, serialised and then dropped by the middleware, so it is never built.
/// That is the visualizer's business, not the caller's — an algorithm stages
/// what it wants drawn and never asks whether anyone is watching.  The cost of
/// an unwatched cycle is one subscription count per channel; the cost of a
/// watched one is what it always was.
template <typename MsgT>
class TypedChannel : public VizChannel {
protected:
    explicit TypedChannel(std::string topic) : VizChannel(std::move(topic)) {}

    /// The staged data AS a message, stamped with @p frame and @p stamp — and
    /// handed over, not copied: what was staged now belongs to the message, and
    /// the buffer is left empty behind it.  Handing it over is why a publish
    /// costs no more than the conversion the message type itself needs; the
    /// clearing that follows from FineNavVisualizer is then a no-op on a cycle
    /// that published, and the whole of it on one that did not.
    ///
    /// Called only on a cycle that has someone to receive it, and with mutex_
    /// held — from publishStaged() — so it locks nothing itself.
    virtual MsgT takeMessage(common::Frame frame, const rclcpp::Time& stamp) = 0;

private:
    void publishStaged(rclcpp::Node& node, common::Frame frame,
                       const rclcpp::Time& stamp) final {
        // One boundary, one atomic step: the staged check, the publisher,
        // the subscription count, the take and the send all happen with the
        // channel locked, and a cycle that sends nothing still drains it below.
        // The lock is held across the send on purpose — a channel is published
        // once per planning cycle, not at bus rates, and releasing it here would
        // be the very window this exists to close.
        std::lock_guard<std::mutex> lock(mutex_);
        if (empty()) {
            return;  // nothing staged: no message, and no publisher created
        }
        if (!pub_) {
            pub_ = node.create_publisher<MsgT>(topic_, 10);
        }

        // Nobody subscribed: stop before taking the message.  The conversion is
        // the expensive half — the staged data copied into its wire form, then
        // serialised — so a gate that only skipped the send would save almost
        // nothing.
        //
        // The publisher has to exist before it can be asked, which is why the
        // count is read here and not before its creation: the first cycle pays
        // for the publisher, and every cycle after it is already gated.  Both
        // counts matter — a subscriber in this process is invisible to the
        // DDS-level count, so testing that one alone would starve it forever.
        const bool watched = pub_->get_subscription_count() > 0 ||
                             pub_->get_intra_process_subscription_count() > 0;
        if (!watched) {
            // Debug, not warn: staging into a viewer nobody has open is the
            // normal state of a robot at rest, not a fault.  Throttled and
            // level-gated, so a cycle that publishes pays nothing for it.
            FINENAV_LOG_DEBUG_THROTTLE(node.get_logger(), 5000,
                                       "Nothing subscribed to '{}' — publish skipped.", topic_);
            // The gate skips the send, not the boundary: a publish boundary
            // drains the channel whether or not anyone was watching, so what a
            // caller staged is consumed either way.
            clearUnlocked();
            return;
        }

        pub_->publish(takeMessage(frame, stamp));
    }

    std::shared_ptr<rclcpp::Publisher<MsgT>> pub_;
};

}  // namespace finenav::viz
