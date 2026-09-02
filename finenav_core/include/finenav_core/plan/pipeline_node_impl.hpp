// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of pipeline_node.hpp.
// It contains the out-of-line template method implementations for PipelineNode.
// Do NOT include this file directly — include pipeline_node.hpp instead.

#pragma once

#include "finenav_core/common/plan_registry.hpp"
#include "finenav_core/plan/ros_conversions.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::core {

// ==============================================================================
// Construction
// ==============================================================================

template <Planner PlannerT>
PipelineNode<PlannerT>::PipelineNode(
    rclcpp::Node::SharedPtr node,
    const std::string& algo_name,
    const std::string& layer_name,
    RobotStateProvider* state_provider)
    : IPipelineNode(layer_name),
      node_(std::move(node)),
      layer_name_(layer_name),
      state_provider_(state_provider)
{
    // ── Private node ──────────────────────────────────────────────────────
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(layer_name, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);

    // ── Planner instance + parameter wiring ───────────────────────────────
    planner_name_ = algo_name;
    auto pollable = PlanRegistry::instance().loadPlannerParams(
        algo_name, pnode_, &planner_instance_);
    if (!pollable) {
        // No FINENAV_REGISTER_PLANNER entry reached the registry for this name.
        // Parameters will NOT load; the planner still runs with defaults.
        FINENAV_LOG_WARN(pnode_->get_logger(),
            "PipelineNode '{}': no params registered under '{}' — parameters will NOT "
            "be loaded. Register it in its .cpp with FINENAV_REGISTER_PLANNER.",
            layer_name_, algo_name);
    }
    param_keeper_.keep_alive(std::move(pollable));
    FINENAV_LOG_INFO(pnode_->get_logger(), "Loading planner '{}' (layer '{}').",
        planner_name_, layer_name_);
}

template <Planner PlannerT>
PipelineNode<PlannerT>::~PipelineNode() = default;

// ==============================================================================
// IPipelineNode
// ==============================================================================

template <Planner PlannerT>
void PipelineNode<PlannerT>::connectFrom(const std::string& parent_topic) {
    setInputTopic(parent_topic);
    setupDataPorts();
}

template <Planner PlannerT>
void PipelineNode<PlannerT>::attachMapView(
    std::shared_ptr<void> view,
    std::vector<std::shared_ptr<IMapResource>> maps)
{
    if constexpr (HasMapView<PlannerT>) {
        // Stored type-erased: the pointer already targets the PlannerT::IMapView
        // base subobject (MapViewRegistry upcasts before erasing), so planOnce
        // recovers the typed pointer with a static cast.
        view_ = std::move(view);
        map_resources_ = std::move(maps);
    } else {
        // Map-less planner: it declares no IMapView, so no view can target it
        // (a MapView can never be registered against it either).
        (void)view;
        (void)maps;
        FINENAV_LOG_WARN(pnode_->get_logger(),
            "attachMapView ignored: '{}' (layer '{}') is a map-less planner.",
            planner_name_, layer_name_);
    }
}

// ==============================================================================
// Topic / Port Setup
// ==============================================================================

template <Planner PlannerT>
void PipelineNode<PlannerT>::setInputTopic(const std::string& topic) {
    input_topic_ = topic;
}

template <Planner PlannerT>
void PipelineNode<PlannerT>::setupDataPorts() {
    if (!input_topic_.empty()) {
        plan_ref_sub_ = pnode_->create_subscription<finenav_interfaces::msg::TaskFeedback>(
            input_topic_, 10,
            [this](std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg) {
                onPlanRef(std::move(msg));
            });
    }
}

template <Planner PlannerT>
void PipelineNode<PlannerT>::onPlanRef(
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg)
{
    storePlanRef(std::move(msg));
}

template <Planner PlannerT>
void PipelineNode<PlannerT>::storePlanRef(
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg)
{
    std::lock_guard<std::mutex> lock(plan_ref_mutex_);
    plan_ref_msg_ = std::move(msg);  // zero-copy: shared_ptr pass-through
}

// ==============================================================================
// Shared Helpers
// ==============================================================================

template <Planner PlannerT>
void PipelineNode<PlannerT>::resetPlanner() {
    if constexpr (requires { planner_instance_.reset(); }) {
        planner_instance_.reset();
    }
}

// ==============================================================================
// planOnce — the shared plan pipeline
// ==============================================================================

template <Planner PlannerT>
std::optional<finenav_interfaces::msg::Trajectory>
PipelineNode<PlannerT>::planOnce(geometry_msgs::msg::Pose goal_pose) {
    if constexpr (HasMapView<PlannerT>) {
        if (!view_) {
            FINENAV_LOG_ERROR(pnode_->get_logger(),
                "MapView not attached — call attachMapView() or createLayerByMapView() first.");
            return std::nullopt;
        }
    }
    if (!state_provider_) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "RobotStateProvider not set — cannot plan without robot state.");
        return std::nullopt;
    }

    auto maybe_state = state_provider_->getRobotState();
    if (!maybe_state.has_value()) {
        FINENAV_LOG_ERROR(pnode_->get_logger(),
            "Failed to get current robot state — localizer may not be initialized yet.");
        return std::nullopt;
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
        return std::nullopt;
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

    auto result = [&]() -> std::optional<OutputTrajectoryT<PlannerT>> {
        if constexpr (HasMapView<PlannerT>) {
            // view_ is type-erased (already targeting the PlannerT::IMapView base
            // subobject); recover the typed pointer here.  This branch is only
            // compiled for map-reading planners.
            auto typed_view = std::static_pointer_cast<typename PlannerT::IMapView>(view_);
            return planner_instance_.plan(ctx, *typed_view);
        } else {
            return planner_instance_.plan(ctx);
        }
    }();
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
        return output_traj_msg;
    }
    return std::nullopt;
}

} // namespace finenav::core
