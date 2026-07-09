// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of control_layer.hpp.
// It contains the out-of-line template method implementations for ControlLayer.
// Do NOT include this file directly — include control_layer.hpp instead.

#pragma once

#include "finenav_util/common/log_macros.hpp"

namespace finenav::core {

// ==============================================================================
// Constructor
// ==============================================================================

template <typename... Planners>
ControlLayer<Planners...>::ControlLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& layer_name,
    ExecutionPolicyVariant policy,
    IRobotStateProvider* state_provider)
    : node_(node), layer_name_(layer_name), policy_(std::move(policy)),
      state_provider_(state_provider)
{
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(layer_name, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);

    std::visit([this](auto& p) {
        if constexpr (requires { p.declare_parameters(*pnode_); }) {
            p.declare_parameters(*pnode_);
        }
    }, policy_);

    if (!layer_name.empty()) {
        action_server_ = std::make_unique<ActionServer>(
            pnode_,
            layer_name,
            std::bind(&ControlLayer::execute, this));
    }

    std::apply([this](auto&... planners) {
        ([this](auto& p) {
            using T = std::decay_t<decltype(p)>;
            FINENAV_LOG_INFO(pnode_->get_logger(), "Loading planner: {}", AlgoConfigurator<T>::name);
            param_keeper_.keep_alive(AlgoConfigurator<T>::load(pnode_, p));
        }(planners), ...);
    }, planners_instance_);
}

// ==============================================================================
// Topic / Port Setup
// ==============================================================================

template <typename... Planners>
void ControlLayer<Planners...>::setInputTopic(const std::string& topic) {
    input_topic_ = topic;
}

template <typename... Planners>
void ControlLayer<Planners...>::setOutputTopic(const std::string& topic) {
    output_topic_ = topic;
}

template <typename... Planners>
std::string ControlLayer<Planners...>::getName() const {
    return pnode_->get_name();
}

template <typename... Planners>
void ControlLayer<Planners...>::setupDataPorts() {
    if (!input_topic_.empty()) {
        plan_ref_sub_ = pnode_->create_subscription<finenav_interfaces::msg::Trajectory>(
            input_topic_, 10,
            std::bind(&ControlLayer::PlanRefCallback, this, std::placeholders::_1));
    }
    if (!output_topic_.empty()) {
        plan_result_pub_ = pnode_->create_publisher<finenav_interfaces::msg::Trajectory>(output_topic_, 10);
    }
}

// ==============================================================================
// Utilities
// ==============================================================================

template <typename... Planners>
void ControlLayer<Planners...>::PlanRefCallback(
    const finenav_interfaces::msg::Trajectory& msg)
{
    std::lock_guard<std::mutex> lock(plan_ref_mutex_);
    plan_ref_msg_ = msg;
}

// ==============================================================================
// Shared Helpers
// ==============================================================================

template <typename... Planners>
void ControlLayer<Planners...>::resetActivePlanner() {
    std::apply([&](auto&... planners) {
        ([&](auto& p) {
            using PlannerT = std::decay_t<decltype(p)>;
            if (AlgoConfigurator<PlannerT>::name == active_planner_id_) {
                if constexpr (requires { p.reset(); }) {
                    p.reset();
                    FINENAV_LOG_INFO(pnode_->get_logger(),
                        "Planner '{}' reset at episode boundary.", active_planner_id_);
                }
            }
        }(planners), ...);
    }, planners_instance_);
}

template <typename... Planners>
bool ControlLayer<Planners...>::planOnce(geometry_msgs::msg::Pose goal_pose) {
    if (!map_view_factory_) {
        FINENAV_LOG_ERROR(pnode_->get_logger(), "MapViewFactory not set!");
        return false;
    }
    if (!state_provider_) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "IRobotStateProvider not set — cannot plan without robot state.");
        return false;
    }

    auto maybe_state = state_provider_->getRobotState();
    if (!maybe_state.has_value()) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "Failed to get current robot state — localizer may not be initialized yet.");
        return false;
    }
    RobotState current_robot_state = *maybe_state;

    finenav_interfaces::msg::Trajectory current_ref_msg;
    {
        std::lock_guard<std::mutex> lock(plan_ref_mutex_);
        current_ref_msg = plan_ref_msg_;
    }

    bool plan_success  = false;
    bool planner_found = false;
    finenav_interfaces::msg::Trajectory output_traj_msg;

    auto try_run_planner = [&](auto& planner) -> bool {
        using PlannerT = std::decay_t<decltype(planner)>;
        if (AlgoConfigurator<PlannerT>::name != active_planner_id_) {
            return false;
        }

        auto opt_ctx = [&]() -> std::optional<PlanningContext<PlannerT>> {
            PrePlanContext pre_ctx{
                active_planner_id_, current_robot_state, goal_pose, current_ref_msg};

            if (on_pre_plan_ && !on_pre_plan_(pre_ctx)) {
                return std::nullopt;
            }

            PlanningContext<PlannerT> ctx;
            ctx.robot_state = current_robot_state;
            ctx.goal_pose = pre_ctx.goal_pose;
            fromROSMsg(std::move(pre_ctx.ref_traj), ctx.ref_traj);
            return ctx;
        }();

        if (!opt_ctx.has_value()) {
            return true; // 短路遍历
        }

        auto& ctx   = opt_ctx.value();
        auto  bound = map_view_factory_(active_planner_id_);

        using ExpectedPtrType = std::shared_ptr<typename PlannerT::IMapView>;
        if (auto* ptr = std::get_if<ExpectedPtrType>(&bound.view_variant)) {
            if (*ptr) {
                // bound.lock_guard 持有读锁，plan() 调用期间地图不被写入
                auto result = planner.plan(ctx, **ptr);
                if (result.has_value()) {
                    plan_success = true;
                    toROSMsg(std::move(result.value()), output_traj_msg);
                } else {
                    FINENAV_LOG_WARN(pnode_->get_logger(), "plan() returned nullopt.");
                }
            } else {
                FINENAV_LOG_ERROR(pnode_->get_logger(),
                    "MapView ptr is null for planner '{}'.", active_planner_id_);
            }
        } else {
            FINENAV_LOG_ERROR(pnode_->get_logger(),
                "MapView type mismatch for planner '{}'.", active_planner_id_);
        }

        if (on_post_plan_) {
            PostPlanContext post_ctx{active_planner_id_, plan_success, output_traj_msg};
            plan_success = on_post_plan_(post_ctx);
        }

        return true; // 短路遍历
    };

    std::apply([&](auto&... planners) {
        planner_found = (try_run_planner(planners) || ...);
    }, planners_instance_);

    if (!planner_found) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "Planner '{}' not found.", active_planner_id_);
        return false;
    }

    if (plan_success && plan_result_pub_) {
        plan_result_pub_->publish(output_traj_msg);
    }

    return plan_success;
}

// ==============================================================================
// execute() — Episode Entry-Point
// ==============================================================================

template <typename... Planners>
void ControlLayer<Planners...>::execute() {
    FINENAV_LOG_INFO(pnode_->get_logger(), "New goal received — starting episode.");

    auto goal          = action_server_->get_current_goal();
    active_planner_id_ = goal->planner_id;

    // Episode 边界：在任何 plan() 调用前重置规划器内部状态
    resetActivePlanner();

    // 按策略执行
    std::visit([this](auto& p) { this->executeImpl(p); }, policy_);
}

// ==============================================================================
// executeImpl — SingleShotPolicy
// ==============================================================================

template <typename... Planners>
void ControlLayer<Planners...>::executeImpl(SingleShotPolicy&) {
    auto goal_pose = action_server_->get_current_goal()->goal;

    if (action_server_->is_cancel_requested()) {
        FINENAV_LOG_INFO(pnode_->get_logger(), "SingleShot: goal cancelled before plan.");
        action_server_->terminate_all();
        return;
    }

    if (!planOnce(goal_pose)) {
        FINENAV_LOG_ERROR(pnode_->get_logger(), "SingleShot: plan() failed.");
        action_server_->terminate_all();
        return;
    }

    action_server_->succeeded_current();
    FINENAV_LOG_INFO(pnode_->get_logger(), "SingleShot: goal succeeded.");
}

// ==============================================================================
// executeImpl — ContinuousPolicy
// ==============================================================================

template <typename... Planners>
void ControlLayer<Planners...>::executeImpl(ContinuousPolicy& policy) {
    auto goal_pose = action_server_->get_current_goal()->goal;
    rclcpp::Rate rate(policy.frequency_hz);

    FINENAV_LOG_INFO(pnode_->get_logger(),
        "Tracking: control loop started at %.1f Hz.", policy.frequency_hz);

    while (rclcpp::ok()) {
        if (action_server_->is_cancel_requested()) {
            // BT 驱动的正常取消（如 IsGoalReached/IsStuck 触发），不代表错误。
            FINENAV_LOG_INFO(pnode_->get_logger(),
                "Tracking: goal cancelled by BT (normal termination).");
            action_server_->terminate_all();
            return;
        }

        // 检测 BT 侧通过 shouldResendGoal() 发来的新 goal（进入 pending slot）。
        // 原地切换 goal_pose，不调用 resetActivePlanner()，保留 MPPI 热启动状态。
        if (action_server_->is_preempt_requested()) {
            auto new_goal = action_server_->accept_pending_goal();
            if (new_goal) {
                goal_pose = new_goal->goal;
                FINENAV_LOG_INFO(pnode_->get_logger(),
                    "Tracking: goal preempted in-loop (%.2f, %.2f) — MPPI warm-start preserved.",
                    goal_pose.position.x, goal_pose.position.y);
            }
        }

        if (!planOnce(goal_pose)) {
            FINENAV_LOG_ERROR(pnode_->get_logger(), "Tracking: plan() failed — aborting.");
            action_server_->terminate_all();
            return;
        }


        rate.sleep();
    }

    action_server_->succeeded_current();
    FINENAV_LOG_INFO(pnode_->get_logger(), "Tracking: goal succeeded.");
}

// ==============================================================================
// bind_map_view
// ==============================================================================

template <typename... Planners>
template <typename MapT, typename BuilderFn>
void ControlLayer<Planners...>::bind_map_view(
    std::shared_ptr<MapServer<MapT>> server,
    BuilderFn&& builder)
{
    static_assert(SpatialContainer<MapT>, // Use `static_assert` instead of `requires` to prevent clangd error.
        "MapT must satisfy SpatialContainer.");
    static_assert(std::is_invocable_v<BuilderFn, const MapT&, const std::string&>,
        "BuilderFn must be callable with (const MapT&, const std::string& active_planner_id).");
    static_assert(std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&, const std::string&>,
                                      MapViewsVariant>,
        "BuilderFn must return a type convertible to MapViewsVariant.");
    std::function<MapViewsVariant(const MapT&, const std::string&)> erased_builder =
        std::forward<BuilderFn>(builder);
    std::weak_ptr<MapServer<MapT>> weak_server = server;

    map_view_factory_ = [weak_server,
                         erased_builder = std::move(erased_builder)]
                        (const std::string& active_planner_id) -> BoundMapView {
        auto srv = weak_server.lock();
        if (!srv) { return BoundMapView{}; }
        auto locked = std::make_shared<LockedMapRO<MapT>>(srv->getLockedReadView());
        MapViewsVariant view_variant = erased_builder(locked->get(), active_planner_id);
        return BoundMapView{std::move(view_variant), std::move(locked)};
    };
}

} // namespace finenav::core

