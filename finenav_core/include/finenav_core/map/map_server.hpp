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
    /**
     * @brief Per-source buffer and timeout policy.
     *
     * Each update source (laser, RGB-D, etc.) can have independent buffering
     * parameters tuned to its publication rate and latency characteristics.
     */
    struct SourceBufferPolicy {
        std::size_t max_buffer_size{32};       ///< Max buffered frames (0 = unbounded)
        double observation_keep_time_sec{0.0}; ///< Max message age before eviction (0 = keep forever)
        double source_timeout_sec{0.0};        ///< Source timeout threshold (0 = disabled)
    };

    /// Hook executed BEFORE sensor fusion, with write lock held. Map is mutable.
    using PreUpdateHook  = std::function<void(MapT& map)>;

    /// Hook executed AFTER sensor fusion, with write lock held. Map is read-only.
    using PostUpdateHook = std::function<void(const MapT& map)>;

    template <typename MsgT>
    using Fuser = std::function<void(const MsgT& msg, const rclcpp::Time& stamp, MapT& map)>;

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

    /** @brief Register a hook called BEFORE sensor fusion (write lock held, map is mutable). */
    void registerPreUpdateHook(std::function<void(MapT& map)> hook);

    /** @brief Register a hook called AFTER sensor fusion (write lock held, map is read-only). */
    void registerPostUpdateHook(std::function<void(MapT& map)> hook);

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
     * The returned callable can be invoked from any thread (e.g., a ROS
     * subscription callback). Messages are buffered internally and fused into
     * the map by the update timer during the next runUpdateCycle().
     *
     * @tparam MsgT       Message type (e.g., pcl::PointCloud<pcl::PointXYZ>)
     * @param source_name Human-readable source identifier (used in log messages)
     * @param policy      Buffer size and timeout configuration
     * @param fuser       Fusion callback: (const MsgT& msg, const rclcpp::Time& stamp, MapT& map)
     * @return Thread-safe push handle: void(MsgT&& msg, const rclcpp::Time& stamp)
     */
    template <typename MsgT>
    std::function<void(MsgT&&, const rclcpp::Time&)> addObservationSource(
        const std::string& source_name, SourceBufferPolicy policy, Fuser<MsgT> fuser);

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
    std::vector<std::function<void(MapT&)>>          pre_update_hooks_;
    std::vector<std::function<void(MapT&)>>          post_update_hooks_;
    std::vector<std::shared_ptr<IObservationSource<MapT>>> observation_sources_;
    std::vector<std::shared_ptr<void>>               held_param_handles_;
    rclcpp::TimerBase::SharedPtr                     update_timer_;
};

}  // namespace finenav::core

#include "finenav_core/map/map_server_impl.hpp"

