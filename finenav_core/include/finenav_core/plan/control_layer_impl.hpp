// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of control_layer.hpp.
// It contains the out-of-line template method implementations for ControlLayer.
// Do NOT include this file directly — include control_layer.hpp instead.

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "finenav_core/common/plan_registry.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/ros_conversions.hpp"
#include "finenav_util/common/log_macros.hpp"
#include "finenav_util/ros/node_thread.hpp"
#include "finenav_util/ros/task_channel.hpp"

namespace finenav::core {

// ==============================================================================
// Constructor
// ==============================================================================

template <Planner PlannerT>
ControlLayer<PlannerT>::ControlLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& algo_name,
    const std::string& layer_name,
    ExecutionPolicyVariant policy,
    RobotStateProvider* state_provider)
    : IControlLayer(layer_name),
      node_(node), layer_name_(layer_name), policy_(std::move(policy)),
      state_provider_(state_provider)
{
    // ── Private node + worker infrastructure ─────────────────────────────
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(layer_name, node->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);

    std::visit([&pnode = pnode_](auto& p) {
        if constexpr (requires { p.declare_parameters(*pnode); }) {
            p.declare_parameters(*pnode);
        }
    }, policy_);

    worker_ = std::make_unique<finenav::util::TaskWorker>(pnode_, layer_name);

    planner_name_ = algo_name;
    auto pollable = PlanRegistry::instance().loadPlannerParams(
        algo_name, pnode_, &planner_instance_);
    if (!pollable) {
        // No FINENAV_REGISTER_PLANNER entry reached the registry for this name.
        // Parameters will NOT load; the planner still runs with defaults.
        FINENAV_LOG_WARN(pnode_->get_logger(),
            "ControlLayer '{}': no params registered under '{}' — parameters will NOT "
            "be loaded. Register it in its .cpp with FINENAV_REGISTER_PLANNER.",
            layer_name_, algo_name);
    }
    param_keeper_.keep_alive(std::move(pollable));
    FINENAV_LOG_INFO(pnode_->get_logger(), "Loading planner '{}' (layer '{}').",
        planner_name_, layer_name_);

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
    if (!view_) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "MapView not attached — call attachMapView() or createLayerByMapView() first.");
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

    // Acquire read locks on every map this view reads, held until plan() returns.
    std::vector<std::shared_ptr<void>> map_locks;
    map_locks.reserve(map_resources_.size());
    for (const auto& res : map_resources_) {
        map_locks.push_back(res->lockRead());
    }

    auto result = planner_instance_.plan(ctx, *view_);
    if (result.has_value()) {
        plan_success = true;
        toROSMsg(std::move(result.value()), output_traj_msg);
    } else {
        FINENAV_LOG_WARN(pnode_->get_logger(), "plan() returned nullopt.");
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
// attachMapView
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::attachMapView(
    std::shared_ptr<void> view,
    std::vector<std::shared_ptr<IMapResource>> maps)
{
    // The erased pointer already points at the PlannerT::IMapView base subobject
    // (MapViewRegistry's build upcasts before erasing to void*), so this static
    // cast is an identity cast — no undefined pointer adjustment.
    view_ = std::static_pointer_cast<typename PlannerT::IMapView>(std::move(view));
    map_resources_ = std::move(maps);
}

} // namespace finenav::core

