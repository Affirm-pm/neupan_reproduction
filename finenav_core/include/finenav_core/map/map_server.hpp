// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>

#include "finenav_core/common/param_pollable_keeper.hpp"
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
 * Both update-cycle hooks AND observation sources are declared through
 * MapRuntime only, from inside the map's own `selfManage()`.  This class used
 * to mirror both as "user-facing" overloads carrying `MapT&` for external
 * consumers; they were removed before 1.0.  The hooks were a same-shaped
 * duplicate whose single-slot storage silently dropped an earlier
 * registration, and their stated purpose (external visualisation) is now
 * served by `finenav::viz::FineNavVisualizer`.  The observation-source
 * overloads were pure forwarding wrappers around the MapRuntime ones.
 *
 * The rule this encodes: only the map's author decides what the map is fed and
 * when it derives.  A downstream application tunes parameters; it does not
 * reach into a map's update cycle.
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

    // ── IMapResource introspection (type-erased, for WorldView assembly) ─────
    std::string       mapTypeId() const override;
    const void*       mapData() const override;
    std::shared_ptr<void> lockRead() override;

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

private:
    void runUpdateCycle(const rclcpp::Time& now);
    void runShiftCycle(const rclcpp::Time& now);

    // ---- ROS infrastructure ----
    rclcpp::Node::SharedPtr     node_;
    std::string                 server_name_;
    rclcpp::Node::SharedPtr     pnode_;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread_;

    // ---- Framework resource (visualizer lives here) ----
    // Declared BEFORE map_instance_ so it is destroyed AFTER it: the map holds
    // a raw pointer into the visualizer, so it must die first.
    ParamPollableKeeper param_keeper_;

    // ---- Map data & lock ----
    std::unique_ptr<MapT> map_instance_;
    std::shared_mutex     rw_mutex_;

    // ---- MapRuntime: owns dev hooks, observation sources, subscriptions ----
    MapRuntime runtime_;

    // ---- Optional robot state provider for map shifting ----
    RobotStateProvider*         state_provider_{nullptr};
    rclcpp::TimerBase::SharedPtr shift_timer_;

    double                                           update_rate_hz_{10.0};
    double                                           shift_rate_hz_{50.0};
    double                                           shift_margin_ratio_{0.5};
    rclcpp::TimerBase::SharedPtr                     update_timer_;
};

}  // namespace finenav::core

#include "finenav_core/map/detail/map_server_impl.hpp"
