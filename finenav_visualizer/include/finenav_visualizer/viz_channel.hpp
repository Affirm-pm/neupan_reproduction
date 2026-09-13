// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

#include "finenav_common/frames.hpp"
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

    /// Frame the staged data will be stamped with.
    ///
    /// A tag, not a name: `setFrame("map")` and `setFrame("odom")` would both
    /// compile, and a typo would draw into a frame the framework never publishes
    /// — which then reads as a localization bug.  common::Frame cannot be
    /// misspelled, and its NAME is resolved from frames() at publish time, so a
    /// system that renamed the frames at startup gets the new names.  (A plugin
    /// names this type finenav::common::Frame — the one spelling, because a
    /// frame tag is a framework convention rather than a finenav_core type, and
    /// this header sits below finenav_core and could not name a core type
    /// anyway.)
    ///
    /// There is no default: the framework does not choose a frame for the
    /// algorithm, because a frame belongs to the DATA — a global path is
    /// world-referenced, a footprint is body-referenced — and only the algorithm
    /// knows which it just staged.  A channel nobody gave a frame publishes an
    /// empty frame_id, which a viewer reads as its fixed frame.
    void setFrame(common::Frame frame) { frame_ = frame; }

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

    /// The name to stamp with — resolved HERE, on the way out, not in
    /// setFrame(): the frame names are the surrounding system's choice and can
    /// be renamed up to the moment anything is assembled, so a tag is only
    /// turned into a name when a message is actually being made.  Empty when
    /// nobody set a frame.
    const std::string& frameId() const {
        static const std::string none;
        return frame_ ? common::frameName(*frame_) : none;
    }

    std::optional<common::Frame> frame_;

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
