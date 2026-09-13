// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
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

    /// True when nothing is staged — the state publish() leaves the channel in.
    virtual bool empty() const = 0;

    /// Discard everything staged, publishing nothing.  FineNavVisualizer calls
    /// this on the channel after each publish, so staged data never crosses a
    /// publish boundary.
    virtual void clear() = 0;

    /// Send what is staged, if anything, on topic_, stamped with @p frame.
    /// Called only by FineNavVisualizer::publish(channel, frame).
    virtual void publishStaged(rclcpp::Node& node, common::Frame frame,
                               const rclcpp::Time& stamp) = 0;

    /// The exact ROS topic this channel publishes on — its identity, and the key
    /// FineNavVisualizer keeps it under.  Nothing is ever appended to it.
    std::string topic_;

private:
    friend class FineNavVisualizer;
};

/// The layer that answers "a channel of MsgT": the per-topic publisher cache,
/// the lazy creation and the empty-check live here, once, so neither the
/// visualizer nor a concrete channel repeats them.
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
    /// Called only on a cycle that has someone to receive it.
    virtual MsgT takeMessage(common::Frame frame, const rclcpp::Time& stamp) = 0;

private:
    void publishStaged(rclcpp::Node& node, common::Frame frame,
                       const rclcpp::Time& stamp) final {
        if (empty()) {
            return;  // nothing staged: no message, and no publisher created
        }
        const std::string& topic = topic_;
        auto& pub = pubs_[topic];
        if (!pub) {
            pub = node.create_publisher<MsgT>(topic, 10);
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
        const bool watched = pub->get_subscription_count() > 0 ||
                             pub->get_intra_process_subscription_count() > 0;
        if (!watched) {
            // Debug, not warn: staging into a viewer nobody has open is the
            // normal state of a robot at rest, not a fault.  Throttled and
            // level-gated, so a cycle that publishes pays nothing for it.
            FINENAV_LOG_DEBUG_THROTTLE(node.get_logger(), 5000,
                                       "Nothing subscribed to '{}' — publish skipped.", topic);
            return;
        }

        pub->publish(takeMessage(frame, stamp));
    }

    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<MsgT>>> pubs_;
};

}  // namespace finenav::viz
