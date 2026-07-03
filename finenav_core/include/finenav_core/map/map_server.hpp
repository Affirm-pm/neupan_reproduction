// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_core/map/locked_map_view.hpp"
#include "finenav_core/map/observation_source.hpp"
#include "finenav_util/node_thread.hpp"

namespace finenav::core {

/**
 * @brief MapServer — runtime orchestrator for a single SpatialContainer instance.
 *
 * MapServer owns the map, runs the sensor-fusion update loop, provides
 * thread-safe RAII access for planner threads
 *
 * Sensor data is NOT fused synchronously on arrival.  It is moved into a
 * per-source buffer and deferred until the next update cycle.  The update
 * timer drains all buffers and fuses each observation into the map.
 *
 * @tparam MapT  Map type; must satisfy SpatialContainer at minimum.  If MapT
 *               also satisfies ShiftableMap and a state provider is given, the
 *               optional window-tracking feature is enabled at compile time.
 */
template <SpatialContainer MapT>
class MapServer {
public:

    /// Re-exported from ObservationSource.
    template <typename MsgT>
    using Fuser = typename ObservationSource<MapT, MsgT>::Fuser;

    /// Hook executed BEFORE / AFTER sensor fusion, with write lock held.
    using PreUpdateHook  = std::function<void(const RobotState& robot_state, MapT& map)>;
    using PostUpdateHook = std::function<void(const RobotState& robot_state, MapT& map)>;

    /**
     * @brief MapServer Constructor.
     * @param node           Parent ROS node
     * @param server_name    Unique name
     * @param update_rate_hz Sensor fusion timer frequency (Hz), default 10.0
     * @param state_provider Source of robot pose
     * @param shift_rate_hz  Map window shift check frequency (Hz), default 50.0.
     */
    explicit MapServer(
        rclcpp::Node::SharedPtr node,
        const std::string& server_name,
        IRobotStateProvider* state_provider,
        double update_rate_hz = 10.0,
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
     * @brief Acquire a RAII read-only view of the map.
     * @return LockedMapRO<MapT> providing const access via operator-> and get()
     */
    LockedMapRO<MapT> getLockedReadView();

    /**
     * @brief Acquire a RAII read-write view of the map.
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
        const std::string& source_name, ObservationBufferPolicy policy, Fuser<MsgT> fuser);

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
     * @param fuser       Fusion callback: (const MsgT& msg, const rclcpp::Time& stamp, MapT& map)
     */
    template <typename MsgT>
    void addObservationSource(
        const std::string& source_name,
        const std::string& topic_name,
        const rclcpp::QoS& qos,
        ObservationBufferPolicy policy,
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
    std::vector<rclcpp::SubscriptionBase::SharedPtr> held_subscriptions_;
    std::vector<std::shared_ptr<void>>               held_param_handles_;
    double                                           update_rate_hz_{0.0};
    double                                           shift_rate_hz_{0.0};
    rclcpp::TimerBase::SharedPtr                     update_timer_;
};

}  // namespace finenav::core

#include "finenav_core/map/map_server_impl.hpp"

