// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "finenav_util/common/log_macros.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/robot_state/robot_state.hpp"

namespace finenav::core {

// Forward declaration — ObservationSource is owned by MapServer.
template <SpatialContainer MapT>
class MapServer;

/**
 * @brief ObservationBufferPolicy — buffer and timeout parameters for one sensor stream.
 */
struct ObservationBufferPolicy {
    std::size_t max_buffer_size{32};           ///< Max buffered frames (0 = unbounded)
    double observation_keep_time_sec{0.0};     ///< Max message age before eviction (0 = keep forever)
    double source_timeout_sec{0.0};            ///< Source timeout threshold (0 = disabled)
    double expected_update_rate_hz{0.0};       ///< Expected sensor rate; warns on starvation (0 = disabled)
};


template <SpatialContainer MapT>
class IObservationSource {
public:
    virtual ~IObservationSource() = default;

    virtual void applyBuffered(MapT& map, const rclcpp::Time& now,
                                RobotStateProvider* state_provider) = 0;
};

/**
 * @brief ObservationSource — manages the data buffer for one sensor stream.
 *
 * @tparam MapT  Map type.
 * @tparam MsgT  Sensor message type.
 */
template <SpatialContainer MapT, typename MsgT>
class ObservationSource final : public IObservationSource<MapT> {
public:

    using Fuser = std::function<void(
        const MsgT& msg, const rclcpp::Time& stamp,
        const RobotState& state, MapT& map)>;

    ObservationSource(const std::string& source_name, ObservationBufferPolicy policy,
                      Fuser fuser, rclcpp::Logger logger);

    void enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp);

    /// Drain buffered messages and fuse each one into the map.
    /// Skips messages whose timestamped robot state is unavailable.
    void applyBuffered(MapT& map, const rclcpp::Time& now,
                        RobotStateProvider* state_provider) override;

private:

    struct BufferedMessage { // Explicitly use RCL_ROS_TIME to prevent time source mismatch
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        MsgT msg;
    };

    void enqueue(MsgT&& msg, const rclcpp::Time& stamp);
    void pruneExpired(const rclcpp::Time& now);

    std::string source_name_;
    ObservationBufferPolicy policy_;
    Fuser fuser_;
    rclcpp::Logger logger_;
    std::mutex mutex_;
    std::deque<BufferedMessage> buffer_;
    rclcpp::Time last_seen_stamp_{0, 0, RCL_ROS_TIME};
    bool has_seen_message_{false};
};

// ==============================================================================
// ObservationSource Implementation
// ==============================================================================
template <SpatialContainer MapT, typename MsgT>
ObservationSource<MapT, MsgT>::ObservationSource(
    const std::string& source_name, ObservationBufferPolicy policy,
    Fuser fuser, rclcpp::Logger logger)
    : source_name_(source_name), policy_(policy), fuser_(std::move(fuser)),
      logger_(std::move(logger)) {}

template <SpatialContainer MapT, typename MsgT>
void ObservationSource<MapT, MsgT>::enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    enqueue(std::move(msg), stamp);
}

template <SpatialContainer MapT, typename MsgT>
void ObservationSource<MapT, MsgT>::applyBuffered(
    MapT& map, const rclcpp::Time& now, RobotStateProvider* state_provider) {

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
        fuser_(item.msg, item.stamp, *maybe_state, map);
    }
}

template <SpatialContainer MapT, typename MsgT>
void ObservationSource<MapT, MsgT>::enqueue(MsgT&& msg, const rclcpp::Time& stamp) {
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

template <SpatialContainer MapT, typename MsgT>
void ObservationSource<MapT, MsgT>::pruneExpired(const rclcpp::Time& now) {
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

} // namespace finenav::core
