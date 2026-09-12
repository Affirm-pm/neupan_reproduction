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
#include "finenav_core/robot_state/robot_state.hpp"

namespace finenav::core {

/**
 * @brief ObservationBufferPolicy — buffer and timeout parameters for one sensor stream.
 */
struct ObservationBufferPolicy {
    std::size_t max_buffer_size{32};           ///< Max buffered frames (0 = unbounded)
    double observation_keep_time_sec{0.0};     ///< Max message age before eviction (0 = keep forever)
    double source_timeout_sec{0.0};            ///< Source timeout threshold (0 = disabled)
    double expected_update_rate_hz{0.0};       ///< Expected sensor rate; warns on starvation (0 = disabled)
};

/**
 * @brief IObservationSource — type-erased base for all observation sources.
 *
 * Non-template so MapRuntime can store a heterogeneous collection.
 */
class IObservationSource {
public:
    virtual ~IObservationSource() = default;

    virtual void applyBuffered(const rclcpp::Time& now,
                                RobotStateProvider* state_provider) = 0;
};

/**
 * @brief ObservationSource — manages the data buffer for one sensor stream.
 *
 * @tparam MsgT  Sensor message type.
 */
template <typename MsgT>
class ObservationSource final : public IObservationSource {
public:

    using Fuser = std::function<void(
        const MsgT& msg, const rclcpp::Time& stamp,
        const RobotState& state)>;

    ObservationSource(const std::string& source_name, ObservationBufferPolicy policy,
                      Fuser fuser, rclcpp::Logger logger);

    void enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp);

    /// Drain buffered messages and fuse each one into the map.
    /// Skips messages whose timestamped robot state is unavailable.
    void applyBuffered(const rclcpp::Time& now,
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

} // namespace finenav::core

#include "finenav_core/map/detail/observation_source_impl.hpp"
