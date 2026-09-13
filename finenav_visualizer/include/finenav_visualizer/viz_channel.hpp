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
    const std::string& frame() const { return frame_; }

    /// True when nothing is staged — the state publish() leaves the channel in.
    virtual bool empty() const = 0;

    /// Discard everything staged, publishing nothing.  FineNavVisualizer calls
    /// this on every channel after each publish, so staged data never crosses a
    /// publish boundary.
    virtual void clear() = 0;

protected:
    VizChannel() = default;

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

    /// A copy of what is staged, stamped with frame() and @p stamp.  Copy, not
    /// move: clearing is the visualizer's step, after the send.  Called only on
    /// a cycle that has someone to receive it.
    virtual MsgT buildMessage(const rclcpp::Time& stamp) const = 0;

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

        // Nobody subscribed: stop before building.  The build is the expensive
        // half — a full copy of the staged data, then serialisation — so a gate
        // that only skipped the send would save almost nothing.
        //
        // The publisher has to exist before it can be asked, which is why the
        // count is read here and not before its creation: the first cycle pays
        // for the publisher, and every cycle after it is already gated.  Both
        // counts matter — a subscriber in this process is invisible to the
        // DDS-level count, so testing that one alone would starve it forever.
        const bool watched = pub->get_subscription_count() > 0 ||
                             pub->get_intra_process_subscription_count() > 0;
        if (!watched) {
            return;
        }

        pub->publish(buildMessage(stamp));
    }

    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<MsgT>>> pubs_;
};

}  // namespace finenav::viz
