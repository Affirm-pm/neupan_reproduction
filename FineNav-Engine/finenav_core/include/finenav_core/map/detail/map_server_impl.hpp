// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <typeinfo>

#include "finenav_core/map/detail/map_registry.hpp"
#include "finenav_util/common/log_macros.hpp"
#include "finenav_util/common/stopwatch.hpp"
#include "finenav_util/ros/node_thread.hpp"

namespace finenav::core {
namespace detail {

// Parameter declaration + validation for the MapServer private node.
// (Formerly src/map_server_helpers.cpp.)

inline void declareMapServerParameters(
    rclcpp::Node& pnode,
    double& update_rate_hz,
    double& shift_rate_hz,
    double& shift_margin_ratio)
{
    // update_rate_hz
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description = "Sensor fusion update rate (Hz). Must be > 0.0.";
        pnode.declare_parameter<double>("update_rate_hz", update_rate_hz, desc);
        update_rate_hz = pnode.get_parameter("update_rate_hz").as_double();
        if (update_rate_hz <= 0.0) {
            FINENAV_LOG_WARN(pnode.get_logger(),
                "update_rate_hz must be > 0.0 (got {:.1f}). Falling back to 10.0.",
                update_rate_hz);
            update_rate_hz = 10.0;
        }
    }
    // shift_rate_hz
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description = "Map window shift check rate (Hz). Must be > 0.0.";
        pnode.declare_parameter<double>("shift_rate_hz", shift_rate_hz, desc);
        shift_rate_hz = pnode.get_parameter("shift_rate_hz").as_double();
        if (shift_rate_hz <= 0.0) {
            FINENAV_LOG_WARN(pnode.get_logger(),
                "shift_rate_hz must be > 0.0 (got {:.1f}). Falling back to 50.0.",
                shift_rate_hz);
            shift_rate_hz = 50.0;
        }
    }
    // shift_margin_ratio
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description =
            "Distance-from-centre ratio triggering a window shift "
            "(0 = centre only, 1 = at edge). Must be in [0.0, 1.0].";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value   = 1.0;
        range.step       = 0.0;
        desc.floating_point_range = {range};
        pnode.declare_parameter<double>("shift_margin_ratio", shift_margin_ratio, desc);
        shift_margin_ratio = pnode.get_parameter("shift_margin_ratio").as_double();
        if (shift_margin_ratio < 0.0) {
            FINENAV_LOG_WARN(pnode.get_logger(),
                "shift_margin_ratio must be >= 0.0 (got {:.2f}). Falling back to 0.0.",
                shift_margin_ratio);
            shift_margin_ratio = 0.0;
        }
        if (shift_margin_ratio > 1.0) {
            FINENAV_LOG_WARN(pnode.get_logger(),
                "shift_margin_ratio must be <= 1.0 (got {:.2f}). Falling back to 1.0.",
                shift_margin_ratio);
            shift_margin_ratio = 1.0;
        }
    }
}

}  // namespace detail

// --- MapServer::MapServer  (see map_server.hpp) --------------------------------
template <Map MapT>
MapServer<MapT>::MapServer(
    rclcpp::Node::SharedPtr node,
    const std::string& algo_name,
    const std::string& server_name,
    RobotStateProvider* state_provider)
    : IMapResource(server_name),
      node_(node), server_name_(server_name), state_provider_(state_provider),
      runtime_(node, node->get_logger()) {

    // ── Private node + timers infrastructure ─────────────────────────────
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(server_name, node->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);

    detail::declareMapServerParameters(
        *pnode_, update_rate_hz_, shift_rate_hz_, shift_margin_ratio_);

    // Provisional update timer (re-bound below once runUpdateCycle exists).
    auto provisional_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), provisional_period,
        [pnode = pnode_]() {
            FINENAV_LOG_WARN_ONCE(pnode->get_logger(),
                "MapServer update timer fired but runUpdateCycle not yet bound.");
        });

    map_instance_ = std::make_unique<MapT>();
    auto pollable = detail::MapRegistry::instance().loadMapParams(
        algo_name, pnode_, map_instance_.get());
    if (!pollable) {
        // No FINENAV_REGISTER_MAP entry reached the registry for this name.
        // Parameters will NOT load; the map still runs with defaults.
        FINENAV_LOG_WARN(node_->get_logger(),
            "MapServer '{}': no params registered under '{}' — parameters will NOT "
            "be loaded. Register it in its .cpp with FINENAV_REGISTER_MAP.",
            server_name_, algo_name);
    }
    param_keeper_.keep_alive(std::move(pollable));

    map_instance_->selfManage(runtime_);

    if (state_provider == nullptr) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "MapServer '{}': state_provider is required but nullptr was passed. "
            "No timers will be created — this MapServer is dormant.",
            server_name_);
        return;
    }

    // Re-bind the update timer to the actual runUpdateCycle method.
    auto update_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), update_period,
        [this]() { this->runUpdateCycle(this->pnode_->now()); });

    FINENAV_LOG_INFO(node_->get_logger(),
        "MapServer '{}': update timer started at {:.1f} Hz.",
        server_name_, update_rate_hz_);

    if constexpr (ShiftableMap<MapT>) {
        auto shift_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / shift_rate_hz_));
        shift_timer_ = rclcpp::create_timer(
            pnode_, pnode_->get_clock(), shift_period,
            [this]() { this->runShiftCycle(this->pnode_->now()); });
        FINENAV_LOG_INFO(node_->get_logger(),
            "MapServer '{}': shift timer started at {:.1f} Hz.",
            server_name_, shift_rate_hz_);
    }
}

// --- MapServer::mapTypeId  (see map_server.hpp) -------------------------------
template <Map MapT>
std::string MapServer<MapT>::mapTypeId() const {
    return typeid(MapT).name();
}

// --- MapServer::mapData  (see map_server.hpp) ---------------------------------
template <Map MapT>
const void* MapServer<MapT>::mapData() const {
    return map_instance_.get();
}

// --- MapServer::revision  (see map_server.hpp) --------------------------------
template <Map MapT>
std::uint64_t MapServer<MapT>::revision() const noexcept {
    return revision_.load(std::memory_order_acquire);
}

// --- MapServer::lockRead  (see map_server.hpp) --------------------------------
template <Map MapT>
std::shared_ptr<void> MapServer<MapT>::lockRead() {
    return std::make_shared<LockedMapRO<MapT>>(getLockedReadView());
}

// --- MapServer::getLockedReadView  (see map_server.hpp) ------------------------
template <Map MapT>
LockedMapRO<MapT> MapServer<MapT>::getLockedReadView() {
    return LockedMapRO<MapT>(*map_instance_, rw_mutex_);
}

// --- MapServer::getLockedWriteView  (see map_server.hpp) -----------------------
template <Map MapT>
LockedMapRW<MapT> MapServer<MapT>::getLockedWriteView() {
    // Coarse change signal: anyone taking write access may mutate the map.
    revision_.fetch_add(1, std::memory_order_release);
    return LockedMapRW<MapT>(*map_instance_, rw_mutex_);
}

// --- MapServer::runUpdateCycle  (private, see map_server.hpp) ------------------
template <Map MapT>
void MapServer<MapT>::runUpdateCycle(const rclcpp::Time& now) {
    param_keeper_.pollAll();

    finenav::util::Stopwatch sw(node_->get_logger(), "runUpdateCycle");

    std::unique_lock<std::shared_mutex> lock(rw_mutex_);

    // ── Map pre-update hooks (via MapRuntime) ────────────────────────────
    if (auto maybe_rs = state_provider_->getRobotState()) {
        runtime_.applyPreHooks(*maybe_rs);
    } else {
        FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 2000,
            "MapServer '{}': robot state unavailable — skipping pre-update hooks.",
            server_name_);
    }

    // ── All observation sources ──────────────────────────────────────────
    runtime_.applyObservationSources(now, state_provider_);

    // ── Map post-update hooks (via MapRuntime) ───────────────────────────
    if (auto maybe_rs = state_provider_->getRobotState()) {
        runtime_.applyPostHooks(*maybe_rs);
    } else {
        FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 2000,
            "MapServer '{}': robot state unavailable — skipping post-update hooks.",
            server_name_);
    }

    // ── Robot-inside-window check (windowed maps only) ───────────────────
    if constexpr (ShiftableMap<MapT>) {
        const auto maybe_state = state_provider_->getRobotState();
        if (!maybe_state) {
            return;
        }
        const auto& pt = maybe_state->pose.position;
        const common::Position3D robot_pose{pt.x, pt.y, pt.z};
        if (!map_instance_->isInside(robot_pose)) {
            const auto bounds = map_instance_->getWindowBounds();
            const auto center = map_instance_->getWindowCenter();
            const auto extent = sizes(bounds);

            FINENAV_LOG_ERROR_THROTTLE(node_->get_logger(), 3000,
                "\n[MapServer: {}] ROBOT OUTSIDE MAP WINDOW - HARD FAULT\n"
                "  Robot position   : ({:.2f}, {:.2f}, {:.2f})\n"
                "  Window center    : ({:.2f}, {:.2f}, {:.2f})\n"
                "  Window min       : ({:.2f}, {:.2f}, {:.2f})\n"
                "  Window max       : ({:.2f}, {:.2f}, {:.2f})\n"
                "  Window extent    : ({:.1f}, {:.1f}, {:.1f}) m",
                server_name_,
                robot_pose.x, robot_pose.y, robot_pose.z,
                center.x, center.y, center.z,
                bounds.min.x, bounds.min.y, bounds.min.z,
                bounds.max.x, bounds.max.y, bounds.max.z,
                extent.x, extent.y, extent.z);
        }
    }

    // Coarse change signal: one bump per completed cycle, after every mutation
    // site has run — observation fusion (or its absence), the pre/post hooks,
    // and any configure() applied by the parameter poll at the top.  Tracking
    // is the framework's job, so a map author never reports its own changes.
    revision_.fetch_add(1, std::memory_order_release);

    sw.warn_if_over(1000.0 / update_rate_hz_);
}

// --- MapServer::runShiftCycle  (private, see map_server.hpp) -------------------
template <Map MapT>
void MapServer<MapT>::runShiftCycle(const rclcpp::Time& now) {
    finenav::util::Stopwatch sw(pnode_->get_logger(), "runShiftCycle");

    if constexpr (ShiftableMap<MapT>) {
        const auto maybe_state = state_provider_->getRobotState();
        if (!maybe_state) return;

        const auto& pt = maybe_state->pose.position;
        const common::Position3D robot_pose{pt.x, pt.y, pt.z};
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        if (!map_instance_->isInside(robot_pose)) {
            FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 1000,
                "MapServer '{}': robot escaped window — forcing immediate shift.",
                server_name_);
            map_instance_->shiftWindowTo(robot_pose);
            revision_.fetch_add(1, std::memory_order_release);
            return;
        }

        const auto center      = map_instance_->getWindowCenter();
        const auto bounds      = map_instance_->getWindowBounds();
        const auto half_extent = sizes(bounds) * 0.5;

        if (half_extent.x > 0.0 && half_extent.y > 0.0 && half_extent.z > 0.0) {
            const auto offset = abs(robot_pose - center);
            const bool near_edge =
                (offset.x / half_extent.x > shift_margin_ratio_) ||
                (offset.y / half_extent.y > shift_margin_ratio_) ||
                (offset.z / half_extent.z > shift_margin_ratio_);

            if (!near_edge) return;
        }

        map_instance_->shiftWindowTo(robot_pose);
        revision_.fetch_add(1, std::memory_order_release);

        FINENAV_LOG_DEBUG(pnode_->get_logger(),
            "MapServer window shifted to ({:.2f}, {:.2f}, {:.2f})",
            robot_pose.x, robot_pose.y, robot_pose.z);
    }

    sw.warn_if_over(1000.0 / shift_rate_hz_);
}

} // namespace finenav::core
