// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/algo_configurator.hpp"
#include "finenav_util/common/log_macros.hpp"
#include "finenav_util/common/stopwatch.hpp"

namespace finenav::core {

// --- MapServer::declare_parameters --------------------------------------------
template <SpatialContainer MapT>
void MapServer<MapT>::declare_parameters() {
    // update_rate_hz
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description = "Sensor fusion update rate (Hz). Must be > 0.0.";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value   = 0.0;  // no upper bound
        range.step       = 0.0;
        desc.floating_point_range = {range};
        pnode_->declare_parameter<double>("update_rate_hz", update_rate_hz_, desc);
        update_rate_hz_ = pnode_->get_parameter("update_rate_hz").as_double();
        if (update_rate_hz_ <= 0.0) {
            FINENAV_LOG_WARN(pnode_->get_logger(),
                "update_rate_hz must be > 0.0 (got {:.1f}). Falling back to 10.0.",
                update_rate_hz_);
            update_rate_hz_ = 10.0;
        }
    }
    // shift_rate_hz
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description = "Map window shift check rate (Hz). Must be > 0.0.";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value   = 0.0;
        range.step       = 0.0;
        desc.floating_point_range = {range};
        pnode_->declare_parameter<double>("shift_rate_hz", shift_rate_hz_, desc);
        shift_rate_hz_ = pnode_->get_parameter("shift_rate_hz").as_double();
        if (shift_rate_hz_ <= 0.0) {
            FINENAV_LOG_WARN(pnode_->get_logger(),
                "shift_rate_hz must be > 0.0 (got {:.1f}). Falling back to 50.0.",
                shift_rate_hz_);
            shift_rate_hz_ = 50.0;
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
        pnode_->declare_parameter<double>("shift_margin_ratio", shift_margin_ratio_, desc);
        shift_margin_ratio_ = pnode_->get_parameter("shift_margin_ratio").as_double();
        if (shift_margin_ratio_ < 0.0) {
            FINENAV_LOG_WARN(pnode_->get_logger(),
                "shift_margin_ratio must be >= 0.0 (got {:.2f}). Falling back to 0.0.",
                shift_margin_ratio_);
            shift_margin_ratio_ = 0.0;
        }
        if (shift_margin_ratio_ > 1.0) {
            FINENAV_LOG_WARN(pnode_->get_logger(),
                "shift_margin_ratio must be <= 1.0 (got {:.2f}). Falling back to 1.0.",
                shift_margin_ratio_);
            shift_margin_ratio_ = 1.0;
        }
    }
}

// --- MapServer::MapServer  (see map_server.hpp) --------------------------------
template <SpatialContainer MapT>
MapServer<MapT>::MapServer(
    rclcpp::Node::SharedPtr node,
    const std::string& server_name,
    IRobotStateProvider* state_provider)
    : node_(node), server_name_(server_name), state_provider_(state_provider) {

    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(server_name_, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);
    map_instance_ = std::make_unique<MapT>();
    param_keeper_.keep_alive(AlgoConfigurator<MapT>::load(pnode_, *map_instance_));

    if (state_provider == nullptr) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "MapServer '{}': state_provider is required but nullptr was passed. "
            "No timers will be created — this MapServer is dormant.",
            server_name_);
        return;
    }

    declare_parameters();

    // Sensor fusion update timer.
    auto update_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), update_period,
        [this]() { this->runUpdateCycle(this->pnode_->now()); });

    FINENAV_LOG_INFO(node_->get_logger(),
        "MapServer '{}': update timer started at {:.1f} Hz.",
        server_name_, update_rate_hz_);

    // Map window shift timer — only when the map is shiftable.
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
template <SpatialContainer MapT>
void MapServer<MapT>::registerPreUpdateHook(PreUpdateHook hook) {
    pre_update_hook_ = std::move(hook);
}

// --- MapServer::registerPostUpdateHook  (see map_server.hpp) -------------------
template <SpatialContainer MapT>
void MapServer<MapT>::registerPostUpdateHook(PostUpdateHook hook) {
    post_update_hook_ = std::move(hook);
}

// --- MapServer::getLockedReadView  (see map_server.hpp) ------------------------
template <SpatialContainer MapT>
LockedMapRO<MapT> MapServer<MapT>::getLockedReadView() {
    return LockedMapRO<MapT>(*map_instance_, rw_mutex_);
}

// --- MapServer::getLockedWriteView  (see map_server.hpp) -----------------------
template <SpatialContainer MapT>
LockedMapRW<MapT> MapServer<MapT>::getLockedWriteView() {
    return LockedMapRW<MapT>(*map_instance_, rw_mutex_);
}

// --- MapServer::addObservationSource (manual injector)  (see map_server.hpp) ---
template <SpatialContainer MapT>
template <typename MsgT>
std::function<void(MsgT&&, const rclcpp::Time&)>
MapServer<MapT>::addObservationSource(
    const std::string& source_name,
    ObservationBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    using Source = ObservationSource<MapT, MsgT>;
    auto source = std::make_shared<Source>(source_name, policy, std::move(fuser),
                                            node_->get_logger());
    observation_sources_.push_back(source);
    FINENAV_LOG_DEBUG(node_->get_logger(),
        "MapServer added observation source: {}", source_name);
    std::weak_ptr<Source> weak_source = source;
    return [weak_source](MsgT&& msg, const rclcpp::Time& stamp) {
        if (auto src = weak_source.lock()) {
            src->enqueueThreadSafe(std::move(msg), stamp);
        }
    };
}

// --- MapServer::addObservationSource (topic-based)  (see map_server.hpp) -------
template <SpatialContainer MapT>
template <typename MsgT>
void MapServer<MapT>::addObservationSource(
    const std::string& source_name,
    const std::string& topic_name,
    const rclcpp::QoS& qos,
    ObservationBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    auto injector = addObservationSource(source_name, policy, std::move(fuser));

    auto sub = node_->create_subscription<MsgT>(
        topic_name, qos,
        [injector = std::move(injector), source_name,
         logger = node_->get_logger()](std::shared_ptr<MsgT> msg) {
            rclcpp::Time stamp;
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

    FINENAV_LOG_INFO(node_->get_logger(),
        "MapServer '{}': subscribed to '{}' for observation source '{}'.",
        server_name_, topic_name, source_name);
}

// --- MapServer::runUpdateCycle  (private, see map_server.hpp) ------------------
template <SpatialContainer MapT>
void MapServer<MapT>::runUpdateCycle(const rclcpp::Time& now) {
    finenav::util::Stopwatch sw(node_->get_logger(), "runUpdateCycle");

    std::unique_lock<std::shared_mutex> lock(rw_mutex_);

    // Pre-Update
    if (pre_update_hook_) {
        if (auto maybe_rs = state_provider_->getRobotState()) {
            pre_update_hook_(*maybe_rs, *map_instance_);
        } else {
            FINENAV_LOG_WARN(node_->get_logger(),
                "MapServer '{}': try to get current robot state but failed — skipping pre-update hook.",
                server_name_);
        }
    }

    // Update
    for (auto& src : observation_sources_) {
        src->applyBuffered(*map_instance_, now, state_provider_);
    }

    // Post-Update
    if (post_update_hook_) {
        if (auto maybe_rs = state_provider_->getRobotState()) {
            post_update_hook_(*maybe_rs, *map_instance_);
        } else {
            FINENAV_LOG_WARN(node_->get_logger(),
                "MapServer '{}': try to get current robot state but failed — skipping post-update hook.",
                server_name_);
        }
    }

    // ── Window health check ────────────────────────────────────────────────
    {
        const auto maybe_state = state_provider_->getRobotState();
        if (!maybe_state) {
            // Cannot get robot state, silent return.
            return;
        }
        const auto& robot_pose = maybe_state->position;
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
template <SpatialContainer MapT>
void MapServer<MapT>::runShiftCycle(const rclcpp::Time& now) {
    finenav::util::Stopwatch sw(pnode_->get_logger(), "runShiftCycle");

    // Compile-time guard: this method body is only valid for ShiftableMap.
    if constexpr (ShiftableMap<MapT>) {
        const auto maybe_state = state_provider_->getRobotState();
        if (!maybe_state) return;

        const auto& robot_pose = maybe_state->position;
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        // If the robot is not even inside the current window, shift.
        if (!map_instance_->isInside(robot_pose)) {
            FINENAV_LOG_WARN_THROTTLE(node_->get_logger(), 1000,
                "MapServer '{}': robot escaped window — forcing immediate shift.",
                server_name_);
            map_instance_->shiftWindowTo(robot_pose);
            return;
        }

        // If the robot near enough to the edge, shift.
        const auto center      = map_instance_->getWindowCenter();
        const auto bounds      = map_instance_->getWindowBounds();
        const auto half_extent = (bounds.max() - bounds.min()) * 0.5;

        if (half_extent.x() > 0.0 && half_extent.y() > 0.0 && half_extent.z() > 0.0) {
            const auto offset = (robot_pose - center).cwiseAbs();
            const bool near_edge =
                (offset.x() / half_extent.x() > shift_margin_ratio_) ||
                (offset.y() / half_extent.y() > shift_margin_ratio_) ||
                (offset.z() / half_extent.z() > shift_margin_ratio_);

            if (!near_edge) return;  // well within window, nothing to do
        }

        // ── Perform the shift ──────────────────────────────────────────────
        map_instance_->shiftWindowTo(robot_pose);

        FINENAV_LOG_DEBUG(pnode_->get_logger(),
            "MapServer window shifted to ({:.2f}, {:.2f}, {:.2f})",
            robot_pose.x(), robot_pose.y(), robot_pose.z());
    }

    sw.warn_if_over(1000.0 / shift_rate_hz_);
}

} // namespace finenav::core
