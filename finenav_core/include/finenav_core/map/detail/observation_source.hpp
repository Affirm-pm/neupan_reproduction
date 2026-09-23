// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "finenav_core/map/observation_buffer_policy.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::core::detail {

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
 * Created only by MapRuntime::addObservationSource(); a Map developer never
 * names it, and the handle they do keep is the type-erased push callable that
 * MapRuntime returns.
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

}  // namespace finenav::core::detail

#include "finenav_core/map/detail/observation_source_impl.hpp"
