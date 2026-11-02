// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of map/detail/observation_source.hpp.
// It contains the out-of-line template method definitions for ObservationSource.
// Do NOT include this file directly — include map/detail/observation_source.hpp instead.

#pragma once

namespace finenav::core::detail {

template <typename MsgT>
ObservationSource<MsgT>::ObservationSource(
    const std::string& source_name, ObservationBufferPolicy policy,
    Fuser fuser, rclcpp::Logger logger)
    : source_name_(source_name), policy_(policy), fuser_(std::move(fuser)),
      logger_(std::move(logger)) {}

template <typename MsgT>
void ObservationSource<MsgT>::enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    enqueue(std::move(msg), stamp);
}

template <typename MsgT>
void ObservationSource<MsgT>::applyBuffered(
    const rclcpp::Time& now, RobotStateProvider* state_provider) {

    std::deque<BufferedMessage> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pruneExpired(now);
        if (policy_.source_timeout_sec > 0.0 && has_seen_message_) {
            const auto elapsed = (now - last_seen_stamp_).seconds();
            if (elapsed > policy_.source_timeout_sec) {
                if (!buffer_.empty()) {
                    FINENAV_LOG_WARN(logger_,
                        "ObservationSource '{}' has timed out ({} s). Buffer cleared.",
                        source_name_, elapsed);
                }
                buffer_.clear();
            }
        }
        // Data starvation check
        if (policy_.expected_update_rate_hz > 0.0 && has_seen_message_) {
            const double since_last = (now - last_seen_stamp_).seconds();
            const double expected_period = 1.0 / policy_.expected_update_rate_hz;
            if (since_last > expected_period) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 5000,
                    "ObservationSource '{}': no data for {:.2f} s (expected every "
                    "{:.2f} s at {:.1f} Hz). Sensor may be offline.",
                    source_name_, since_last, expected_period,
                    policy_.expected_update_rate_hz);
            }
        }

        pending.swap(buffer_);
    }

    if (!fuser_) {
        FINENAV_LOG_WARN_ONCE(logger_,
            "ObservationSource '{}': no Fuser was provided — messages are "
            "enqueued but never applied.", source_name_);
        return;
    }
    if (!pending.empty()) {
        FINENAV_LOG_DEBUG(logger_,
            "ObservationSource '{}' applying {} buffered messages.",
            source_name_, pending.size());
    }

    for (const auto& item : pending) {
        auto maybe_state = state_provider
            ? state_provider->getRobotState(item.stamp)
            : std::nullopt;
        if (!maybe_state) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 5000,
                "ObservationSource '{}': no robot state available at stamp "
                "{:.3f} s — message dropped.",
                source_name_, item.stamp.seconds());
            continue;
        }
        fuser_(item.msg, item.stamp, *maybe_state);
    }
}

template <typename MsgT>
void ObservationSource<MsgT>::enqueue(MsgT&& msg, const rclcpp::Time& stamp) {
    if (policy_.max_buffer_size > 0 && buffer_.size() >= policy_.max_buffer_size) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "ObservationSource '{}': buffer full ({}/{}); oldest message dropped. "
            "Increase max_buffer_size or the update rate.",
            source_name_, buffer_.size(), policy_.max_buffer_size);
        buffer_.pop_front();
    }
    buffer_.push_back(BufferedMessage{stamp, std::move(msg)});
    last_seen_stamp_  = stamp;
    has_seen_message_ = true;
}

template <typename MsgT>
void ObservationSource<MsgT>::pruneExpired(const rclcpp::Time& now) {
    if (policy_.observation_keep_time_sec <= 0.0) return;
    size_t n = 0;
    while (!buffer_.empty()) {
        if ((now - buffer_.front().stamp).seconds() <= policy_.observation_keep_time_sec)
            break;
        ++n;
        buffer_.pop_front();
    }
    if (n > 0) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "ObservationSource '{}': {} message(s) expired (keep time {:.1f} s). "
            "The update rate may be too slow for the incoming observation rate.",
            source_name_, n, policy_.observation_keep_time_sec);
    }
}

} // namespace finenav::core::detail
