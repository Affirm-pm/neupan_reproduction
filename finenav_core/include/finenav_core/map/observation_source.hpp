// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "finenav_core/map/map_concept.hpp"

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
};


template <SpatialContainer MapT>
class IObservationSource {
public:
    virtual ~IObservationSource() = default;
    virtual void applyBuffered(MapT& map, const rclcpp::Time& now) = 0;
};

/**
 * @brief ObservationSource — manage data buffer for one sensor stream.
 *
 * @tparam MapT  Map type.
 * @tparam MsgT  Sensor message type.
 */
template <SpatialContainer MapT, typename MsgT>
class ObservationSource final : public IObservationSource<MapT> {
public:

    using Fuser = std::function<void(const MsgT& msg, const rclcpp::Time& stamp, MapT& map)>;

    ObservationSource(const std::string& source_name, ObservationBufferPolicy policy, Fuser fuser);
    
    void enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp);

    /**
     * @brief Drain buffered messages and fuse each one into the map.
     */
    void applyBuffered(MapT& map, const rclcpp::Time& now) override;

private:

    struct BufferedMessage {
        rclcpp::Time stamp;
        MsgT msg;
    };

    void enqueue(MsgT&& msg, const rclcpp::Time& stamp);
    void pruneExpired(const rclcpp::Time& now);

    std::string source_name_;
    ObservationBufferPolicy policy_;
    Fuser fuser_;
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
    const std::string& source_name, ObservationBufferPolicy policy, Fuser fuser)
    : source_name_(source_name), policy_(policy), fuser_(std::move(fuser)) {}

template <SpatialContainer MapT, typename MsgT>
void ObservationSource<MapT, MsgT>::enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    enqueue(std::move(msg), stamp);
}

template <SpatialContainer MapT, typename MsgT>
void ObservationSource<MapT, MsgT>::applyBuffered(MapT& map, const rclcpp::Time& now) {

    std::deque<BufferedMessage> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pruneExpired(now);
        if (policy_.source_timeout_sec > 0.0 && has_seen_message_) {
            const auto elapsed = (now - last_seen_stamp_).seconds();
            if (elapsed > policy_.source_timeout_sec) {
                if (!buffer_.empty()) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("MapServer"),
                        "ObservationSource '" << source_name_ << "' has timed out ("
                        << elapsed << "s). Buffer cleared!");
                }
                buffer_.clear();
            }
        }
        pending.swap(buffer_);
    }

    if (!fuser_) {
        RCLCPP_WARN_ONCE(rclcpp::get_logger("MapServer"),
            "ObservationSource '%s': no Fuser was provided — messages are "
            "enqueued but never applied.", source_name_.c_str());
        return;
    }
    if (!pending.empty()) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("MapServer"),
            "ObservationSource '" << source_name_ << "' applying "
            << pending.size() << " buffered messages.");
    }

    for (const auto& item : pending) {
        fuser_(item.msg, item.stamp, map);
    }
}

template <SpatialContainer MapT, typename MsgT>
void ObservationSource<MapT, MsgT>::enqueue(MsgT&& msg, const rclcpp::Time& stamp) {
    if (policy_.max_buffer_size > 0 && buffer_.size() >= policy_.max_buffer_size) {
        static auto throttle_clock = rclcpp::Clock{RCL_STEADY_TIME};
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MapServer"),
            throttle_clock, 2000,
            "ObservationSource '%s': buffer full (%zu / %zu) — oldest message "
            "dropped.  Increase max_buffer_size or the update rate.",
            source_name_.c_str(), buffer_.size(), policy_.max_buffer_size);
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
        static auto throttle_clock = rclcpp::Clock{RCL_STEADY_TIME};
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MapServer"),
            throttle_clock, 2000,
            "ObservationSource '%s': %zu message(s) expired (keep time %.1f s). "
            "The update rate may be too slow for the incoming observation rate.",
            source_name_.c_str(), n, policy_.observation_keep_time_sec);
    }
}

} // namespace finenav::core
