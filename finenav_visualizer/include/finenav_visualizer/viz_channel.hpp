// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::viz {

class FineNavVisualizer;

/// What every staging channel is, regardless of the shape of the data it
/// stages: which frame that data belongs to, whether anything is staged, how to
/// discard it, and how to send it.  One loop in FineNavVisualizer::publish()
///
///     for (VizChannel* channel : channels()) {
///         channel->publishStaged(node, topic_name, now);
///         channel->clear();
///     }
///
/// covers every channel there will ever be.
///
/// Only setFrame() is public, and deliberately so: everything else on this
/// interface is machinery the visualizer drives.  A caller who could ask
/// whether a channel is empty, or reset one, would be writing against the
/// publish boundary rather than through it — and the boundary is what the
/// whole design is for.  Widening any of it later is source-compatible; that is
/// why it starts closed.
///
/// Publishing is protected and the visualizer is a friend, so a channel still
/// has no publish() a caller could reach: reading an algorithm tells you where
/// its data goes — nowhere, until FineNavVisualizer::publish() is called.
class VizChannel {
public:
    virtual ~VizChannel() = default;

    /// Frame the staged data will be stamped with.  There is no default: the
    /// framework does not choose a frame for the algorithm, because a frame
    /// belongs to the DATA — a global path is world-referenced, a footprint is
    /// body-referenced — and only the algorithm knows which it just staged.
    void setFrame(const std::string& frame) { frame_ = frame; }

protected:
    VizChannel() = default;

    /// True when nothing is staged — the state publish() leaves the channel in.
    virtual bool empty() const = 0;

    /// Discard everything staged, publishing nothing.  FineNavVisualizer calls
    /// this on every channel after each publish, so staged data never crosses a
    /// publish boundary.
    virtual void clear() = 0;

    /// Send what is staged, if anything, to @p topic_name + subtopic().  Called
    /// only by FineNavVisualizer::publish().
    virtual void publishStaged(rclcpp::Node& node, const std::string& topic_name,
                               const rclcpp::Time& stamp) = 0;

    std::string frame_;

private:
    friend class FineNavVisualizer;
};

/// The layer that answers "a channel of MsgT": the per-topic publisher cache,
/// the lazy creation and the empty-check live here, once, so neither the
/// visualizer nor a concrete channel repeats them.
///
/// A concrete channel is then only what is genuinely its own — the message it
/// shapes and the sub-topic it lands on.
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
    /// Sub-topic appended to the name given to FineNavVisualizer::publish().
    virtual std::string subtopic() const = 0;

    /// The staged data AS a message, stamped with frame_ and @p stamp — and
    /// handed over, not copied: what was staged now belongs to the message, and
    /// the buffer is left empty behind it.  Handing it over is why a publish
    /// costs no more than the conversion the message type itself needs; the
    /// clearing that follows from FineNavVisualizer is then a no-op on a cycle
    /// that published, and the whole of it on one that did not.
    ///
    /// Called only on a cycle that has someone to receive it.
    virtual MsgT takeMessage(const rclcpp::Time& stamp) = 0;

private:
    void publishStaged(rclcpp::Node& node, const std::string& topic_name,
                       const rclcpp::Time& stamp) final {
        if (empty()) {
            return;  // nothing staged: no message, and no publisher created
        }
        const std::string topic = topic_name + subtopic();
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

        pub->publish(takeMessage(stamp));
    }

    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<MsgT>>> pubs_;
};

}  // namespace finenav::viz
