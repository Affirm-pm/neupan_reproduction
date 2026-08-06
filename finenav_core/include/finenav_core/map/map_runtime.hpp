// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "finenav_core/map/observation_source.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::core {

/**
 * @brief MapRuntime — non-template host API for Map developers.
 *
 * MapRuntime is the ONLY type a Map developer needs to know about from the
 * framework.  It provides a simplified API for declaring observation sources
 * and lifecycle hooks — without MapT& parameters, because the developer is
 * writing inside their own Map class and can capture `this` directly.
 *
 * MapServer<MapT> owns a MapRuntime and delegates its own user-facing
 * registrations (which still carry MapT& for external users) into it by
 * wrapping the callbacks with [this] capture.
 *
 * Usage inside selfManage():
 * @code
 *   void MyMap::selfManage(MapRuntime& runtime) {
 *       runtime.addObservationSource<PointCloud2>(
 *           "laser", "/scan", rclcpp::SensorDataQoS(),
 *           policy,
 *           [this](const PointCloud2& msg, const rclcpp::Time& stamp,
 *                  const RobotState& state) {
 *               this->insertPointCloud(msg, stamp, state);
 *           });
 *       runtime.registerPreUpdateHook([this](const RobotState&) {
 *           this->pruneExpiredCells();
 *       });
 *   }
 * @endcode
 */
class MapRuntime {
public:
    template <typename MsgT>
    using Fuser = std::function<void(
        const MsgT& msg, const rclcpp::Time& stamp, const RobotState& state)>;

    using Hook = std::function<void(const RobotState& state)>;

    /**
     * @brief Construct a MapRuntime.
     * @param node   ROS node used to create internal subscriptions.
     * @param logger Logger for diagnostic messages.
     */
    explicit MapRuntime(rclcpp::Node::SharedPtr node, rclcpp::Logger logger)
        : node_(std::move(node)), logger_(std::move(logger)) {}

    // ── Observation sources ──────────────────────────────────────────────────

    /**
     * @brief Register a sensor data source and return a thread-safe push handle.
     *
     * The returned callable can be invoked from any thread. Messages are
     * buffered internally and fused into the map by the update timer.
     *
     * @tparam MsgT   Message type
     * @param source_name Human-readable source identifier
     * @param policy  Buffer size and timeout configuration
     * @param fuser   Fusion callback: (const MsgT& msg, const rclcpp::Time& stamp,
     *                                    const RobotState& state)
     * @return Thread-safe push handle: void(MsgT&& msg, const rclcpp::Time& stamp)
     */
    template <typename MsgT>
    std::function<void(MsgT&&, const rclcpp::Time&)> addObservationSource(
        const std::string& source_name,
        ObservationBufferPolicy policy,
        Fuser<MsgT> fuser)
    {
        using Source = ObservationSource<MsgT>;
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

    /**
     * @brief Register a sensor data source backed by a ROS topic subscription.
     *
     * Messages are buffered internally and fused into the map by the update timer.
     *
     * @tparam MsgT   ROS message type (e.g., sensor_msgs::msg::PointCloud2)
     * @param source_name Human-readable source identifier
     * @param topic_name  ROS topic to subscribe to
     * @param qos         Subscription QoS
     * @param policy      Buffer size and timeout configuration
     * @param fuser       Fusion callback: (const MsgT& msg, const rclcpp::Time& stamp,
     *                                      const RobotState& state)
     */
    template <typename MsgT>
    void addObservationSource(
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

    // ── Lifecycle hooks ──────────────────────────────────────────────────────

    /** @brief Register a hook called BEFORE sensor fusion, with write lock held. */
    void registerPreUpdateHook(Hook hook) {
        pre_hooks_.push_back(std::move(hook));
    }

    /** @brief Register a hook called AFTER sensor fusion, with write lock held. */
    void registerPostUpdateHook(Hook hook) {
        post_hooks_.push_back(std::move(hook));
    }

    // ── Framework interface (called by MapServer) ────────────────────────────

    /** @internal Called by MapServer during the update cycle. */
    void applyPreHooks(const RobotState& state) {
        for (auto& hook : pre_hooks_) {
            hook(state);
        }
    }

    /** @internal Called by MapServer during the update cycle. */
    void applyPostHooks(const RobotState& state) {
        for (auto& hook : post_hooks_) {
            hook(state);
        }
    }

    /** @internal Called by MapServer during the update cycle. */
    void applyObservationSources(const rclcpp::Time& now,
                                  RobotStateProvider* state_provider) {
        for (auto& src : observation_sources_) {
            src->applyBuffered(now, state_provider);
        }
    }

    rclcpp::Logger getLogger() const { return logger_; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::vector<Hook> pre_hooks_;
    std::vector<Hook> post_hooks_;
    std::vector<std::shared_ptr<IObservationSource>> observation_sources_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> held_subscriptions_;
};

}  // namespace finenav::core
