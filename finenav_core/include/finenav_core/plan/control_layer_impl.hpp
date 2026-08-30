// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of control_layer.hpp.
// It contains the out-of-line template method implementations for ControlLayer.
// Do NOT include this file directly — include control_layer.hpp instead.

#pragma once

#include "finenav_core/algo_configurator.hpp"
#include "finenav_core/detail/control_layer_infra.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/ros_conversions.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::core {

// ==============================================================================
// Constructor
// ==============================================================================

template <Planner PlannerT>
ControlLayer<PlannerT>::ControlLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& layer_name,
    ExecutionPolicyVariant policy,
    RobotStateProvider* state_provider)
    : IControlLayer(layer_name),
      node_(node), layer_name_(layer_name), policy_(std::move(policy)),
      state_provider_(state_provider)
{
    auto infra = detail::buildControlLayerInfra(node, layer_name, policy_);
    pnode_        = std::move(infra.pnode);
    pnode_thread_ = std::move(infra.pnode_thread);
    worker_       = std::move(infra.worker);

    FINENAV_LOG_INFO(pnode_->get_logger(), "Loading planner: {}", AlgoConfigurator<PlannerT>::name);
    param_keeper_.keep_alive(AlgoConfigurator<PlannerT>::load(pnode_, planner_instance_));
    planner_name_ = std::string{AlgoConfigurator<PlannerT>::name};

    worker_thread_ = std::thread([this]() {
        while (rclcpp::ok() && !stop_execution_) {
            if (worker_->hasNewGoal()) {
                execute();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

template <Planner PlannerT>
ControlLayer<PlannerT>::~ControlLayer() {
    stop_execution_ = true;
    if (worker_thread_.joinable()) worker_thread_.join();
}

// ==============================================================================
// IControlLayer
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::connectFrom(const std::string& parent_topic) {
    setInputTopic(parent_topic);
    setupDataPorts();
}

// ==============================================================================
// Topic / Port Setup
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::setInputTopic(const std::string& topic) {
    input_topic_ = topic;
}

template <Planner PlannerT>
void ControlLayer<PlannerT>::setupDataPorts() {
    if (!input_topic_.empty()) {
        plan_ref_sub_ = pnode_->create_subscription<finenav_interfaces::msg::TaskFeedback>(
            input_topic_, 10,
            [this](std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg) {
                PlanRefCallback(std::move(msg));
            });
    }
}

// ==============================================================================
// Utilities
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::PlanRefCallback(
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg)
{
    std::lock_guard<std::mutex> lock(plan_ref_mutex_);
    plan_ref_msg_ = std::move(msg);  // zero-copy: shared_ptr pass-through
}

// ==============================================================================
// Shared Helpers
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::resetPlanner() {
    if constexpr (requires { planner_instance_.reset(); }) {
        planner_instance_.reset();
    }
}

template <Planner PlannerT>
bool ControlLayer<PlannerT>::planOnce(geometry_msgs::msg::Pose goal_pose) {
    if (!map_view_factory_) {
        FINENAV_LOG_ERROR(pnode_->get_logger(), "MapViewFactory not set!");
        return false;
    }
    if (!state_provider_) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "RobotStateProvider not set — cannot plan without robot state.");
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
        if (plan_ref_msg_) {
            current_ref_msg = plan_ref_msg_->trajectory;  // copy only the trajectory
        }
    }

    param_keeper_.pollAll(); // Apply pending dynamic parameter updates

    bool plan_success = false;
    finenav_interfaces::msg::Trajectory output_traj_msg;

    PrePlanContext pre_ctx{
        planner_name_, current_robot_state, goal_pose, current_ref_msg};

    if (on_pre_plan_ && !on_pre_plan_(pre_ctx)) {
        return false;
    }

    PlanningContext<PlannerT> ctx;
    ctx.robot_state = current_robot_state;
    ctx.goal_pose = pre_ctx.goal_pose;
    fromROSMsg(std::move(pre_ctx.ref_traj), ctx.ref_traj);

    auto bound = map_view_factory_();

    if (bound.view) {
        // bound.lock_guard 持有读锁，plan() 调用期间地图不被写入
        auto result = planner_instance_.plan(ctx, *bound.view);
        if (result.has_value()) {
            plan_success = true;
            toROSMsg(std::move(result.value()), output_traj_msg);
        } else {
            FINENAV_LOG_WARN(pnode_->get_logger(), "plan() returned nullopt.");
        }
    } else {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "MapView ptr is null for planner '{}'.", planner_name_);
    }

    if (on_post_plan_) {
        PostPlanContext post_ctx{planner_name_, plan_success, output_traj_msg};
        plan_success = on_post_plan_(post_ctx);
    }

    if (plan_success) {
        // Publish trajectory via TaskChannel
        worker_->publishRunning(std::move(output_traj_msg));
    }

    return plan_success;
}

// ==============================================================================
// execute() — Episode Entry-Point
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::execute() {
    auto goal = worker_->latestGoal();
    if (!goal) return;
    worker_->ackGoal();  // consume the goal

    // ── Wait for the localizer to initialise before any Policy runs ─────────
    while (rclcpp::ok() && !worker_->shouldStop()) {
        if (state_provider_ && state_provider_->getRobotState().has_value()) break;
        FINENAV_LOG_WARN_THROTTLE(pnode_->get_logger(), 2000,
            "Waiting for valid robot state...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (worker_->shouldStop()) {
        worker_->publishFailed();
        return;
    }

    FINENAV_LOG_INFO(pnode_->get_logger(), "New goal received — starting episode.");
    resetPlanner();
    std::visit([this](auto& p) { this->executeImpl(p); }, policy_);
}

// ==============================================================================
// executeImpl — SingleShotPolicy
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::executeImpl(SingleShotPolicy&) {
    auto goal = worker_->latestGoal();
    if (!goal) return;

    if (worker_->shouldStop()) {
        FINENAV_LOG_INFO(pnode_->get_logger(), "SingleShot: cancelled.");
        worker_->publishFailed();
        return;
    }

    if (!planOnce(goal->goal)) {
        FINENAV_LOG_ERROR(pnode_->get_logger(), "SingleShot: plan() failed.");
        worker_->publishFailed();
        return;
    }

    worker_->publishCompleted({});
    FINENAV_LOG_INFO(pnode_->get_logger(), "SingleShot: goal succeeded.");
}

// ==============================================================================
// executeImpl — ContinuousPolicy
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::executeImpl(ContinuousPolicy& policy) {
    auto goal = worker_->latestGoal();
    if (!goal) return;
    auto goal_pose = goal->goal;
    rclcpp::Rate rate(policy.frequency_hz);

    FINENAV_LOG_INFO(pnode_->get_logger(),
        "Tracking: control loop started at {:.1f} Hz.", policy.frequency_hz);

    while (rclcpp::ok()) {
        if (worker_->shouldStop()) {
            FINENAV_LOG_INFO(pnode_->get_logger(),
                "Tracking: goal cancelled by BT (normal termination).");
            worker_->publishFailed();
            return;
        }

        // Preemption: BT sent a new goal → switch in-loop, preserve warm-start
        if (worker_->hasNewGoal()) {
            auto new_goal = worker_->latestGoal();
            if (new_goal) {
                goal_pose = new_goal->goal;
                worker_->ackGoal();
                FINENAV_LOG_INFO(pnode_->get_logger(),
                    "Tracking: goal preempted in-loop ({:.2f}, {:.2f}) — warm-start preserved.",
                    goal_pose.position.x, goal_pose.position.y);
            }
        }

        if (!planOnce(goal_pose)) {
            FINENAV_LOG_ERROR(pnode_->get_logger(), "Tracking: plan() failed — aborting.");
            worker_->publishFailed();
            return;
        }

        rate.sleep();
    }

    worker_->publishCompleted({});
    FINENAV_LOG_INFO(pnode_->get_logger(), "Tracking: goal succeeded.");
}

// ==============================================================================
// bind_map_view
// ==============================================================================

template <Planner PlannerT>
template <typename MapT, typename BuilderFn>
void ControlLayer<PlannerT>::bind_map_view(
    std::shared_ptr<MapServer<MapT>> server,
    BuilderFn&& builder)
{
    static_assert(Map<MapT>, // Use `static_assert` instead of `requires` to prevent clangd error.
        "MapT must satisfy Map.");
    static_assert(std::is_invocable_v<BuilderFn, const MapT&>,
        "BuilderFn must be callable with (const MapT&).");
    static_assert(std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&>,
                                      MapViewPtr>,
        "BuilderFn must return a type convertible to MapViewPtr.");
    std::function<MapViewPtr(const MapT&)> erased_builder =
        std::forward<BuilderFn>(builder);
    std::weak_ptr<MapServer<MapT>> weak_server = server;

    map_view_factory_ = [weak_server,
                         erased_builder = std::move(erased_builder)]
                        () -> BoundMapView {
        auto srv = weak_server.lock();
        if (!srv) { return BoundMapView{}; }
        auto locked = std::make_shared<LockedMapRO<MapT>>(srv->getLockedReadView());
        MapViewPtr view = erased_builder(locked->get());
        return BoundMapView{std::move(view), std::move(locked)};
    };
}

} // namespace finenav::core

