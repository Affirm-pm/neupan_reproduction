// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/algo_configurator.hpp"
#include "finenav_core/detail/map_server_infra.hpp"
#include "finenav_util/common/log_macros.hpp"
#include "finenav_util/common/stopwatch.hpp"

namespace finenav::core {

// --- MapServer::MapServer  (see map_server.hpp) --------------------------------
template <Map MapT>
MapServer<MapT>::MapServer(
    rclcpp::Node::SharedPtr node,
    const std::string& server_name,
    RobotStateProvider* state_provider)
    : IMapResource(server_name),
      node_(node), server_name_(server_name), state_provider_(state_provider),
      runtime_(node, node->get_logger()) {

    auto infra = detail::buildMapServerInfra(
        node, server_name, update_rate_hz_, shift_rate_hz_, shift_margin_ratio_);
    pnode_              = std::move(infra.pnode);
    pnode_thread_       = std::move(infra.pnode_thread);
    update_timer_       = std::move(infra.update_timer);
    shift_timer_        = std::move(infra.shift_timer);
    param_keeper_       = std::move(infra.param_keeper);
    update_rate_hz_     = infra.update_rate_hz;
    shift_rate_hz_      = infra.shift_rate_hz;
    shift_margin_ratio_ = infra.shift_margin_ratio;

    map_instance_ = std::make_unique<MapT>();
    param_keeper_.keep_alive(AlgoConfigurator<MapT>::load(pnode_, *map_instance_));

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

// --- MapServer::registerPreUpdateHook  (see map_server.hpp) --------------------
template <Map MapT>
void MapServer<MapT>::registerPreUpdateHook(PreUpdateHook hook) {
    pre_update_hook_ = std::move(hook);
}

// --- MapServer::registerPostUpdateHook  (see map_server.hpp) -------------------
template <Map MapT>
void MapServer<MapT>::registerPostUpdateHook(PostUpdateHook hook) {
    post_update_hook_ = std::move(hook);
}

// --- MapServer::getLockedReadView  (see map_server.hpp) ------------------------
template <Map MapT>
LockedMapRO<MapT> MapServer<MapT>::getLockedReadView() {
    return LockedMapRO<MapT>(*map_instance_, rw_mutex_);
}

// --- MapServer::getLockedWriteView  (see map_server.hpp) -----------------------
template <Map MapT>
LockedMapRW<MapT> MapServer<MapT>::getLockedWriteView() {
    return LockedMapRW<MapT>(*map_instance_, rw_mutex_);
}

// --- MapServer::addObservationSource (manual injector)  (see map_server.hpp) ---
template <Map MapT>
template <typename MsgT>
std::function<void(MsgT&&, const rclcpp::Time&)>
MapServer<MapT>::addObservationSource(
    const std::string& source_name,
    ObservationBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    // Wrap the user-facing Fuser (MsgT, Time, State, MapT&) → MapRuntime Fuser
    // (MsgT, Time, State) by capturing `this` to provide MapT&.
    return runtime_.addObservationSource<MsgT>(
        source_name, policy,
        [this, fuser = std::move(fuser)](
            const MsgT& msg, const rclcpp::Time& stamp, const RobotState& state) {
            fuser(msg, stamp, state, *map_instance_);
        });
}

// --- MapServer::addObservationSource (topic-based)  (see map_server.hpp) -------
template <Map MapT>
template <typename MsgT>
void MapServer<MapT>::addObservationSource(
    const std::string& source_name,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    ObservationBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    runtime_.addObservationSource<MsgT>(
        source_name, topic_name, qos, policy,
        [this, fuser = std::move(fuser)](
            const MsgT& msg, const rclcpp::Time& stamp, const RobotState& state) {
            fuser(msg, stamp, state, *map_instance_);
        });

    FINENAV_LOG_INFO(node_->get_logger(),
        "MapServer '{}': subscribed to '{}' for observation source '{}'.",
        server_name_, topic_name, source_name);
}

// --- MapServer::runUpdateCycle  (private, see map_server.hpp) ------------------
template <Map MapT>
void MapServer<MapT>::runUpdateCycle(const rclcpp::Time& now) {
    param_keeper_.pollAll();

    finenav::util::Stopwatch sw(node_->get_logger(), "runUpdateCycle");

    std::unique_lock<std::shared_mutex> lock(rw_mutex_);

    // ── Map-developer pre-update hooks (via MapRuntime) ──────────────────
    if (auto maybe_rs = state_provider_->getRobotState()) {
        runtime_.applyPreHooks(*maybe_rs);
    } else {
        FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 2000,
            "MapServer '{}': robot state unavailable — skipping pre-update hooks.",
            server_name_);
    }

    // ── User-facing pre-update hook (with MapT&) ─────────────────────────
    if (pre_update_hook_) {
        if (auto maybe_rs = state_provider_->getRobotState()) {
            pre_update_hook_(*maybe_rs, *map_instance_);
        } else {
            FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 2000,
                "MapServer '{}': robot state unavailable — skipping user pre-update hook.",
                server_name_);
        }
    }

    // ── All observation sources (both dev and user) ──────────────────────
    runtime_.applyObservationSources(now, state_provider_);

    // ── Map-developer post-update hooks (via MapRuntime) ─────────────────
    if (auto maybe_rs = state_provider_->getRobotState()) {
        runtime_.applyPostHooks(*maybe_rs);
    } else {
        FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 2000,
            "MapServer '{}': robot state unavailable — skipping post-update hooks.",
            server_name_);
    }

    // ── User-facing post-update hook (with MapT&) ────────────────────────
    if (post_update_hook_) {
        if (auto maybe_rs = state_provider_->getRobotState()) {
            post_update_hook_(*maybe_rs, *map_instance_);
        } else {
            FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 2000,
                "MapServer '{}': robot state unavailable — skipping user post-update hook.",
                server_name_);
        }
    }

    // ── Robot-inside-window check ────────────────────────────────────────
    {
        const auto maybe_state = state_provider_->getRobotState();
        if (!maybe_state) {
            return;
        }
        const auto& pt = maybe_state->pose.position;
        const Position3D robot_pose(pt.x, pt.y, pt.z);
        if (!map_instance_->isInside(robot_pose)) {
            const auto bounds = map_instance_->getWindowBounds();
            const auto center = map_instance_->getWindowCenter();
            const auto extent = bounds.sizes();

            FINENAV_LOG_ERROR_THROTTLE(node_->get_logger(), 3000,
                "\n[MapServer: {}] ROBOT OUTSIDE MAP WINDOW - HARD FAULT\n"
                "  Robot position   : ({:.2f}, {:.2f}, {:.2f})\n"
                "  Window center    : ({:.2f}, {:.2f}, {:.2f})\n"
                "  Window min       : ({:.2f}, {:.2f}, {:.2f})\n"
                "  Window max       : ({:.2f}, {:.2f}, {:.2f})\n"
                "  Window extent    : ({:.1f}, {:.1f}, {:.1f}) m",
                server_name_,
                robot_pose.x(), robot_pose.y(), robot_pose.z(),
                center.x(), center.y(), center.z(),
                bounds.min().x(), bounds.min().y(), bounds.min().z(),
                bounds.max().x(), bounds.max().y(), bounds.max().z(),
                extent.x(), extent.y(), extent.z());
        }
    }

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
        const Position3D robot_pose(pt.x, pt.y, pt.z);
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        if (!map_instance_->isInside(robot_pose)) {
            FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 1000,
                "MapServer '{}': robot escaped window — forcing immediate shift.",
                server_name_);
            map_instance_->shiftWindowTo(robot_pose);
            return;
        }

        const auto center      = map_instance_->getWindowCenter();
        const auto bounds      = map_instance_->getWindowBounds();
        const auto half_extent = (bounds.max() - bounds.min()) * 0.5;

        if (half_extent.x() > 0.0 && half_extent.y() > 0.0 && half_extent.z() > 0.0) {
            const auto offset = (robot_pose - center).cwiseAbs();
            const bool near_edge =
                (offset.x() / half_extent.x() > shift_margin_ratio_) ||
                (offset.y() / half_extent.y() > shift_margin_ratio_) ||
                (offset.z() / half_extent.z() > shift_margin_ratio_);

            if (!near_edge) return;
        }

        map_instance_->shiftWindowTo(robot_pose);

        FINENAV_LOG_DEBUG(pnode_->get_logger(),
            "MapServer window shifted to ({:.2f}, {:.2f}, {:.2f})",
            robot_pose.x(), robot_pose.y(), robot_pose.z());
    }

    sw.warn_if_over(1000.0 / shift_rate_hz_);
}

} // namespace finenav::core
