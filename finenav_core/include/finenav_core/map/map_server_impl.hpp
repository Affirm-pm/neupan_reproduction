// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
// This file is included at the bottom of map_server.hpp.
// It contains out-of-line template method implementations for MapServer.
// Do NOT include this file directly — include map_server.hpp instead.
#pragma once
namespace finenav::core {

// ==============================================================================
// Constructor
// ==============================================================================

template <SpatialContainer MapT>
MapServer<MapT>::MapServer(
    rclcpp::Node::SharedPtr node,
    const std::string& server_name,
    double update_rate_hz,
    IRobotStateProvider* state_provider,
    double shift_rate_hz)
    : node_(node), server_name_(server_name), state_provider_(state_provider)
{
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(server_name_, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);
    map_instance_ = std::make_unique<MapT>();
    keep_alive(AlgoConfigurator<MapT>::load(pnode_, *map_instance_));

    // Sensor fusion update timer — always needed.
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period,
        [this]() { this->runUpdateCycle(this->pnode_->now()); });

    // Map window shift timer — compile-time decision.
    if constexpr (ShiftableMap<MapT>) {
        if (state_provider_ != nullptr) {
            auto shift_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / shift_rate_hz));
            shift_timer_ = rclcpp::create_timer(
                pnode_, pnode_->get_clock(), shift_period,
                [this]() { this->runShiftCycle(this->pnode_->now()); });
            RCLCPP_INFO(node_->get_logger(),
                "MapServer '%s': shift timer started at %.1f Hz (ShiftableMap).",
                server_name_.c_str(), shift_rate_hz);
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "MapServer '%s': ShiftableMap but no localizer provided; "
                "shift timer disabled. Robot-following will NOT occur.",
                server_name_.c_str());
        }
    } else {
        if (state_provider_ != nullptr) {
            RCLCPP_INFO(node_->get_logger(),
                "MapServer '%s': map is a fixed-location container "
                "(not ShiftableMap); localizer pointer ignored.",
                server_name_.c_str());
        }
    }
}

// ==============================================================================
// Hook Registration
// ==============================================================================

template <SpatialContainer MapT>
void MapServer<MapT>::registerPreUpdateHook(PreUpdateHook hook) {
    pre_update_hooks_.push_back(std::move(hook));
}

template <SpatialContainer MapT>
void MapServer<MapT>::registerPostUpdateHook(PostUpdateHook hook) {
    post_update_hooks_.push_back(std::move(hook));
}

// ==============================================================================
// Locked View Accessors
// ==============================================================================

template <SpatialContainer MapT>
LockedMapRO<MapT> MapServer<MapT>::getLockedReadView() {
    return LockedMapRO<MapT>(*map_instance_, rw_mutex_);
}

template <SpatialContainer MapT>
LockedMapRW<MapT> MapServer<MapT>::getLockedWriteView() {
    return LockedMapRW<MapT>(*map_instance_, rw_mutex_);
}

// ==============================================================================
// addObservationSource
// ==============================================================================

template <SpatialContainer MapT>
template <typename MsgT>
std::function<void(MsgT&&, const rclcpp::Time&)>
MapServer<MapT>::addObservationSource(
    const std::string& source_name,
    SourceBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    using Source = ObservationSource<MapT, MsgT>;
    auto source = std::make_shared<Source>(source_name, policy, std::move(fuser));
    observation_sources_.push_back(source);
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
        "MapServer added update source: " << source_name);
    std::weak_ptr<Source> weak_source = source;
    return [weak_source](MsgT&& msg, const rclcpp::Time& stamp) {
        if (auto src = weak_source.lock()) {
            src->enqueueThreadSafe(std::move(msg), stamp);
        }
    };
}

// ==============================================================================
// runUpdateCycle — sensor fusion + window health check
// ==============================================================================

template <SpatialContainer MapT>
void MapServer<MapT>::runUpdateCycle(const rclcpp::Time& now) {
    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::duration<double, std::milli>;

    std::unique_lock<std::shared_mutex> lock(rw_mutex_);

    auto t0 = Clock::now();
    for (auto& hook : pre_update_hooks_)  hook(*map_instance_);
    auto t1 = Clock::now();
    for (auto& src  : observation_sources_)   src->applyBuffered(*map_instance_, now);
    auto t2 = Clock::now();
    for (auto& hook : post_update_hooks_) hook(*map_instance_);
    auto t3 = Clock::now();

    // ── Timing report ──────────────────────────────────────────────────────
    const double ms_pre   = Ms(t1 - t0).count();
    const double ms_fuse  = Ms(t2 - t1).count();
    const double ms_post  = Ms(t3 - t2).count();
    const double ms_total = Ms(t3 - t0).count();

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "\n[MapServer: %s] update cycle timing\n"
        "  preUpdateHook  : %7.2f ms\n"
        "  applyBuffered  : %7.2f ms\n"
        "  postUpdateHook : %7.2f ms\n"
        "  ─────────────────────────\n"
        "  Total          : %7.2f ms",
        server_name_.c_str(),
        ms_pre, ms_fuse, ms_post, ms_total);

    // ── Window health check ────────────────────────────────────────────────
    //
    // If a localizer is available, verify that the robot is still inside the
    // map window. Being outside is a hard fault: planners reading this map
    // will get invalid or missing data near the robot.
    //
    // Common root causes:
    //   - Shift timer is disabled (no localizer, or ShiftableMap without one)
    //   - shift_rate_hz too slow for the robot's speed
    //   - shiftWindowTo() implementation is a no-op when it shouldn't be
    //   - Window extent is too small for the operational area
    //
    // This check runs on every update cycle (not a separate timer) because it
    // is cheap (one AABB test) and the failure mode is catastrophically silent.
    if (state_provider_ != nullptr) {
        const auto maybe_state = state_provider_->getRobotState();
        if (!maybe_state) {
            // Provider not yet initialised — cannot check window containment.
            // This is normal during startup; will resolve on the next cycle.
            return;
        }
        const auto& robot_pose = maybe_state->position;
        if (!map_instance_->isInside(robot_pose)) {
            const auto bounds = map_instance_->getWindowBounds();
            const auto center = map_instance_->getWindowCenter();
            const auto extent = bounds.sizes();

            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                "\n[MapServer: %s] ROBOT OUTSIDE MAP WINDOW — HARD FAULT\n"
                "  ───────────────────────────────────────────────────\n"
                "  Robot position   : (%7.2f, %7.2f, %7.2f)\n"
                "  Window center    : (%7.2f, %7.2f, %7.2f)\n"
                "  Window min       : (%7.2f, %7.2f, %7.2f)\n"
                "  Window max       : (%7.2f, %7.2f, %7.2f)\n"
                "  Window extent    : (%7.1f, %7.1f, %7.1f) m\n"
                "  ───────────────────────────────────────────────────\n"
                "  Planners reading this map will receive no valid data\n"
                "  near the robot. Check:\n"
                "    - Is the shift timer enabled and running? Is\n"
                "      shift_rate_hz fast enough for robot speed?\n"
                "    - Does shiftWindowTo() actually move the window?\n"
                "    - Is the map's window extent large enough for\n"
                "      the operational area?",
                server_name_.c_str(),
                robot_pose.x(), robot_pose.y(), robot_pose.z(),
                center.x(), center.y(), center.z(),
                bounds.min().x(), bounds.min().y(), bounds.min().z(),
                bounds.max().x(), bounds.max().y(), bounds.max().z(),
                extent.x(), extent.y(), extent.z());
        }
    }
}

// ==============================================================================
// runShiftCycle — smart-gated window recentering (ShiftableMap only)
// ==============================================================================

template <SpatialContainer MapT>
void MapServer<MapT>::runShiftCycle(const rclcpp::Time& now) {
    // Compile-time guard: this method body is only valid for ShiftableMap.
    // For fixed-location maps (SpatialContainer only), the shift timer is
    // never created (see constructor), so this method is never called at
    // runtime. The if constexpr ensures it also never compiles the
    // shiftWindowTo() call for non-shiftable types.
    if constexpr (ShiftableMap<MapT>) {
        const auto maybe_state = state_provider_->getRobotState();
        if (!maybe_state) return;  // provider not initialised yet — skip this cycle

        const auto& robot_pose = maybe_state->position;
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        // ── Gate 1: is the robot even inside the current window? ───────────
        if (!map_instance_->isInside(robot_pose)) {
            // Robot escaped the window entirely. Shift immediately — no
            // gating, no margin check. This is an urgent corrective action.
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "MapServer '%s': robot escaped window — forcing immediate shift.",
                server_name_.c_str());
            map_instance_->shiftWindowTo(robot_pose);
            return;
        }

        // ── Gate 2: is the robot near enough to the edge to justify shifting?
        //
        // We compute the normalised offset of the robot from the window centre
        // in each axis. A value of 0.0 means the robot is at the centre; 1.0
        // means it is at the edge. We trigger a shift when the robot exceeds
        // shift_margin_ratio_ in any axis.
        //
        // Example: a 100 m × 100 m window, margin = 0.5.
        //   Centre = (0, 0). Robot at (30, 0) → offset_ratio = 0.6 → SHIFT.
        //   Robot at (10, 0) → offset_ratio = 0.2 → skip (still well inside).
        //
        // TODO: make shift_margin_ratio a dynamic parameter.
        constexpr double kShiftMarginRatio = 0.5;

        const auto center     = map_instance_->getWindowCenter();
        const auto bounds     = map_instance_->getWindowBounds();
        const auto half_extent = (bounds.max() - bounds.min()) * 0.5;

        // Avoid division by zero for degenerate (zero-size) windows.
        if (half_extent.x() > 0.0 && half_extent.y() > 0.0) {
            const auto offset = (robot_pose - center).cwiseAbs();
            const bool near_edge =
                (offset.x() / half_extent.x() > kShiftMarginRatio) ||
                (offset.y() / half_extent.y() > kShiftMarginRatio);

            if (!near_edge) {
                // Robot is well within the window — nothing to do.
                return;
            }
        }

        // ── Perform the shift ──────────────────────────────────────────────
        map_instance_->shiftWindowTo(robot_pose);

        RCLCPP_DEBUG_STREAM(pnode_->get_logger(),
            "MapServer window shifted to ("
            << robot_pose.x() << ", " << robot_pose.y() << ", "
            << robot_pose.z() << ")");
    }
}

// ==============================================================================
// Utility
// ==============================================================================

template <SpatialContainer MapT>
template <typename T>
void MapServer<MapT>::keep_alive(std::shared_ptr<T> handle) {
    if (handle) held_param_handles_.push_back(handle);
}

} // namespace finenav::core
