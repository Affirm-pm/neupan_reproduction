// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "finenav_util/algo_configurator.hpp"
#include "finenav_util/node_thread.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_core/map/locked_map_view.hpp"
#include "finenav_core/map/observation_source.hpp"

namespace finenav::core {


/**
 * @brief MapServer — manages the lifecycle, concurrent access, and periodic update
 * of a SpatialContainer instance.
 *
 * @tparam MapT  Map type; must satisfy SpatialContainer at minimum.
 */
template <SpatialContainer MapT>
class MapServer {
public:
    /// Re-exported for convenience at call sites.
    using SourceBufferPolicy = finenav::core::SourceBufferPolicy;

    /// Re-exported from ObservationSource.
    template <typename MsgT>
    using Fuser = typename ObservationSource<MapT, MsgT>::Fuser;

    /// Hook executed BEFORE sensor fusion, with write lock held.
    /// Receives the latest robot state (or a zero state if no provider is set).
    using PreUpdateHook  = std::function<void(const RobotState& robot_state, MapT& map)>;

    /// Hook executed AFTER sensor fusion, with write lock held.
    using PostUpdateHook = std::function<void(const RobotState& robot_state, MapT& map)>;

    /**
     * @brief Construct a MapServer and launch its internal update timer.
     *
     * Creates a private ROS node (named @p server_name under the parent node's
     * namespace) running on its own dedicated thread via NodeThread.
     *
     * @param node           Parent ROS node (used for clock and logging)
     * @param server_name    Unique name; also used as the private node name
     * @param update_rate_hz Sensor fusion timer frequency (Hz), default 10.0
     * @param state_provider Source of robot pose for window tracking and health
     *                       checks. Only meaningful when MapT satisfies ShiftableMap;
     *                       otherwise logged and ignored. May be nullptr.
     * @param shift_rate_hz  Map window shift check frequency (Hz), default 50.0.
     *                       Only meaningful when MapT satisfies ShiftableMap.
     */
    explicit MapServer(
        rclcpp::Node::SharedPtr node,
        const std::string& server_name,
        double update_rate_hz = 10.0,
        IRobotStateProvider* state_provider = nullptr,
        double shift_rate_hz = 50.0);

    MapServer(const MapServer&) = delete;
    MapServer& operator=(const MapServer&) = delete;
    MapServer(MapServer&&) = delete;
    MapServer& operator=(MapServer&&) = delete;
    ~MapServer() = default;

    /** @brief Register a hook called BEFORE sensor fusion, with write lock held. */
    void registerPreUpdateHook(PreUpdateHook hook);

    /** @brief Register a hook called AFTER sensor fusion, with write lock held. */
    void registerPostUpdateHook(PostUpdateHook hook);

    /**
     * @brief Acquire a RAII read-only (shared-lock) view of the map.
     * @return LockedMapRO<MapT> providing const access via operator-> and get()
     */
    LockedMapRO<MapT> getLockedReadView();

    /**
     * @brief Acquire a RAII read-write (exclusive-lock) view of the map.
     *
     * @note Use sparingly — every concurrent reader blocks this acquisition.
     * Prefer addObservationSource() for routine data injection.
     *
     * @return LockedMapRW<MapT> providing mutable access via operator-> and get()
     */
    LockedMapRW<MapT> getLockedWriteView();

    /**
     * @brief Register a sensor data source and return a thread-safe push handle.
     *
     * The returned callable can be invoked from any thread. Messages are
     * buffered internally and fused into the map by the update timer.
     *
     * @tparam MsgT   Message type
     * @param source_name Human-readable source identifier
     * @param policy  Buffer size and timeout configuration
     * @param fuser   Fusion callback: (const MsgT& msg, const rclcpp::Time& stamp, MapT& map)
     * @return Thread-safe push handle: void(MsgT&& msg, const rclcpp::Time& stamp)
     */
    template <typename MsgT>
    std::function<void(MsgT&&, const rclcpp::Time&)> addObservationSource(
        const std::string& source_name, SourceBufferPolicy policy, Fuser<MsgT> fuser);

    /**
     * @brief Register a sensor data source backed by a ROS topic subscription.
     *
     * MapServer creates the subscription internally — no separate subscriber or
     * injector variable is needed.  The fuser receives the ROS message type
     * directly and can convert inside the callback (e.g., pcl::fromROSMsg).
     *
     * @tparam MsgT   ROS message type (e.g., sensor_msgs::msg::PointCloud2)
     * @param source_name Human-readable source identifier
     * @param topic_name  ROS topic to subscribe to
     * @param qos         Subscription QoS
     * @param policy      Buffer size and timeout configuration
     * @param fuser       Fusion callback: (const MsgT& msg, const rclcpp::Time& stamp, MapT& map)
     */
    template <typename MsgT>
    void addObservationSource(
        const std::string& source_name,
        const std::string& topic_name,
        const rclcpp::QoS& qos,
        SourceBufferPolicy policy,
        Fuser<MsgT> fuser);

private:
    void runUpdateCycle(const rclcpp::Time& now);
    void runShiftCycle(const rclcpp::Time& now);

    template <typename T>
    void keep_alive(std::shared_ptr<T> handle);

    // ---- ROS infrastructure ----
    rclcpp::Node::SharedPtr     node_;
    std::string                 server_name_;
    rclcpp::Node::SharedPtr     pnode_;
    std::unique_ptr<NodeThread> pnode_thread_;

    // ---- Map data & lock ----
    std::unique_ptr<MapT> map_instance_;
    std::shared_mutex     rw_mutex_;

    // ---- Optional robot state provider for map shifting ----
    IRobotStateProvider*         state_provider_{nullptr};
    rclcpp::TimerBase::SharedPtr shift_timer_;

    // ---- Hooks, sources & timers ----
    PreUpdateHook                                    pre_update_hook_;
    PostUpdateHook                                   post_update_hook_;
    std::vector<std::shared_ptr<IObservationSource<MapT>>> observation_sources_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> held_subscriptions_;  ///< topic-based sources
    std::vector<std::shared_ptr<void>>               held_param_handles_;
    rclcpp::TimerBase::SharedPtr                     update_timer_;
};

}  // namespace finenav::core

#include "finenav_core/map/map_server_impl.hpp"

