// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of map/map_runtime.hpp.
// It contains the out-of-line template definitions for MapRuntime's
// addObservationSource() overloads.
// Do NOT include this file directly — include map_runtime.hpp instead.

#pragma once

namespace finenav::core {

template <typename MsgT>
std::function<void(MsgT&&, const rclcpp::Time&)> MapRuntime::addObservationSource(
    const std::string& source_name,
    ObservationBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    using Source = detail::ObservationSource<MsgT>;
    auto source = std::make_shared<Source>(
        source_name, policy, std::move(fuser), logger_);
    observation_sources_.push_back(source);
    FINENAV_LOG_DEBUG(logger_,
        "MapRuntime: added observation source '{}'.", source_name);
    std::weak_ptr<Source> weak_source = source;
    return [weak_source](MsgT&& msg, const rclcpp::Time& stamp) {
        if (auto src = weak_source.lock()) {
            src->enqueueThreadSafe(std::move(msg), stamp);
        }
    };
}

template <typename MsgT>
void MapRuntime::addObservationSource(
    const std::string& source_name,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    ObservationBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    auto injector = addObservationSource<MsgT>(
        source_name, policy, std::move(fuser));

    auto sub = node_->create_subscription<MsgT>(
        topic_name, qos,
        [injector = std::move(injector), source_name,
         logger = logger_](std::shared_ptr<MsgT> msg) {
            rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
            if constexpr (requires { msg->header.stamp; }) {
                stamp = msg->header.stamp;
            } else {
                FINENAV_LOG_WARN_ONCE(logger,
                    "ObservationSource '{}': message type has no header.stamp — "
                    "using arrival time as stamp.  Sensor fusion accuracy may be "
                    "degraded by time-skew.", source_name);
                stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
            }
            injector(std::move(*msg), stamp);
        });
    held_subscriptions_.push_back(sub);

    FINENAV_LOG_INFO(logger_,
        "MapRuntime: subscribed to '{}' for observation source '{}'.",
        topic_name, source_name);
}

}  // namespace finenav::core
