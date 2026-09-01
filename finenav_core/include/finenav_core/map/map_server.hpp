// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>

#include "../common/param_pollable_keeper.hpp"
#include "finenav_core/map/i_map_resource.hpp"
#include "finenav_core/map/locked_map_view.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/map/map_runtime.hpp"
#include "finenav_core/map/observation_source.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_util/ros/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"

namespace finenav::core {

/**
 * @brief MapServer — runtime host for a Map.
 *
 * MapServer owns the map, runs the sensor-fusion update loop, provides
 * thread-safe RAII access for planner threads, and tracks the robot window
 * (if the map is shiftable).
 *
 * All Maps are self-managing: during construction, MapServer calls
 * map.selfManage(runtime_), passing a MapRuntime& so the map can wire its
 * own data sources and hooks without ever knowing about MapServer.
 *
 * User-facing hooks (registerPreUpdateHook / registerPostUpdateHook) carry
 * MapT& for external consumers (e.g. visualization).  Map-developer hooks
 * registered through MapRuntime use [this] capture instead.
 *
 * Sensor data is NOT fused synchronously on arrival.  It is moved into a
 * per-source buffer and deferred until the next update cycle.  The update
 * timer drains all buffers and fuses each observation into the map.
 *
 * @tparam MapT  Map type; must satisfy Map.  If MapT also satisfies
 *               ShiftableMap and a state provider is given, the optional
 *               window-tracking feature is enabled at compile time.
 */
template <Map MapT>
class MapServer : public IMapResource {
public:

    /// User-facing Fuser: includes MapT& for external consumers.
    /// Map developers should use MapRuntime::Fuser instead (no MapT&).
    template <typename MsgT>
    using Fuser = std::function<void(
        const MsgT& msg, const rclcpp::Time& stamp,
        const RobotState& state, MapT& map)>;

    /// User-facing hooks: include MapT& for external consumers.
    /// Map developers should use MapRuntime::Hook instead (no MapT&).
    using PreUpdateHook  = std::function<void(const RobotState& robot_state, MapT& map)>;
    using PostUpdateHook = std::function<void(const RobotState& robot_state, MapT& map)>;

    /**
     * @brief MapServer Constructor.
     * @param node           Parent ROS node
     * @param algo_name      Algorithm registry key (e.g. "grid_map")
     * @param server_name    Unique name
     * @param state_provider Source of robot pose
     */
    explicit MapServer(
        rclcpp::Node::SharedPtr node,
        const std::string& algo_name,
        const std::string& server_name,
        RobotStateProvider* state_provider);

    MapServer(const MapServer&) = delete;
    MapServer& operator=(const MapServer&) = delete;
    MapServer(MapServer&&) = delete;
    MapServer& operator=(MapServer&&) = delete;
    ~MapServer() = default;

    // ── IMapResource introspection (type-erased, for MapView assembly) ─────
    std::string       mapTypeId() const override;
    const void*       mapData() const override;
    std::shared_ptr<void> lockRead() override;

    /** @brief Register a user-facing hook called BEFORE sensor fusion, with write lock held. */
    void registerPreUpdateHook(PreUpdateHook hook);

    /** @brief Register a user-facing hook called AFTER sensor fusion, with write lock held. */
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
     * @brief Register a user-facing sensor data source and return a push handle.
     *
     * The returned callable can be invoked from any thread. Messages are
     * buffered internally and fused into the map by the update timer.
     *
     * @tparam MsgT   Message type
     * @param source_name Human-readable source identifier
     * @param policy  Buffer size and timeout configuration
     * @param fuser   Fusion callback: (const MsgT&, const rclcpp::Time&,
     *                                   const RobotState&, MapT&)
     * @return Thread-safe push handle: void(MsgT&&, const rclcpp::Time&)
     */
    template <typename MsgT>
    std::function<void(MsgT&&, const rclcpp::Time&)> addObservationSource(
        const std::string& source_name, ObservationBufferPolicy policy, Fuser<MsgT> fuser);

    /**
     * @brief Register a user-facing sensor data source backed by a ROS topic.
     *
     * Messages are buffered internally and fused into the map by the update timer.
     *
     * @tparam MsgT   ROS message type
     * @param source_name Human-readable source identifier
     * @param topic_name  ROS topic to subscribe to
     * @param qos         Subscription QoS
     * @param policy      Buffer size and timeout configuration
     * @param fuser       Fusion callback: (const MsgT&, const rclcpp::Time&,
     *                                      const RobotState&, MapT&)
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

    // ---- ROS infrastructure ----
    rclcpp::Node::SharedPtr     node_;
    std::string                 server_name_;
    rclcpp::Node::SharedPtr     pnode_;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread_;

    // ---- Map data & lock ----
    std::unique_ptr<MapT> map_instance_;
    std::shared_mutex     rw_mutex_;

    // ---- MapRuntime: owns dev hooks, observation sources, subscriptions ----
    MapRuntime runtime_;

    // ---- Optional robot state provider for map shifting ----
    RobotStateProvider*         state_provider_{nullptr};
    rclcpp::TimerBase::SharedPtr shift_timer_;

    // ---- User-facing hooks (with MapT&) for external consumers ----
    PreUpdateHook                                    pre_update_hook_;
    PostUpdateHook                                   post_update_hook_;

    ParamPollableKeeper                              param_keeper_;
    double                                           update_rate_hz_{10.0};
    double                                           shift_rate_hz_{50.0};
    double                                           shift_margin_ratio_{0.5};
    rclcpp::TimerBase::SharedPtr                     update_timer_;
};

}  // namespace finenav::core

#include "finenav_core/map/map_server_impl.hpp"
