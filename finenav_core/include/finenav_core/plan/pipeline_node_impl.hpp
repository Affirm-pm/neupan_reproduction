// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of pipeline_node.hpp.
// It contains the out-of-line template method implementations for PipelineNode.
// Do NOT include this file directly — include pipeline_node.hpp instead.

#pragma once

#include "finenav_core/plan/plan_registry.hpp"
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
    pnode_opts.use_intra_process_comms(true);
    pnode_ = rclcpp::Node::make_shared(layer_name, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);

    // ── Task channel worker: owns cmd_sub_/feedback_pub_ (+ ref_subs_ after
    //    connectFrom, one per possible predecessor) on the private node. ────
    worker_ = std::make_unique<finenav::core::TaskWorker>(pnode_, layer_name_);

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
void PipelineNode<PlannerT>::connectFrom(const std::string& parent_name) {
    worker_->connectFrom(parent_name,
        [this](std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg) {
            onPlanRef(std::move(msg));
        });
}

template <Planner PlannerT>
rclcpp::Node::SharedPtr PipelineNode<PlannerT>::getPrivateNode() const { return pnode_; }

template <Planner PlannerT>
void PipelineNode<PlannerT>::attachWorldView(
    BuiltWorldView built,
    std::vector<std::shared_ptr<IMapResource>> maps)
{
    if constexpr (HasWorldView<PlannerT>) {
        // Stored type-erased: the pointer already targets the PlannerT::IWorldView
        // base subobject (WorldViewRegistry upcasts before erasing), so planOnce
        // recovers the typed pointer with a static cast.
        view_ = std::move(built.view);
        prepare_ = std::move(built.prepare);
        map_resources_ = std::move(maps);
        // The view's parameter pollable also owns its visualizer (and holds a
        // borrow of the view).  param_keeper_ is declared BEFORE view_, so it is
        // destroyed AFTER it — the view never outlives its own resources.
        param_keeper_.keep_alive(std::move(built.params));
    } else {
        // Map-less planner: it declares no IWorldView, so no view can target it
        // (a WorldView can never be registered against it either).
        (void)built;
        (void)maps;
        FINENAV_LOG_WARN(pnode_->get_logger(),
            "attachWorldView ignored: '{}' (layer '{}') is a map-less planner.",
            planner_name_, layer_name_);
    }
}

// ==============================================================================
// Reference handling
// ==============================================================================

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
PipelineNode<PlannerT>::planOnce() {
    if constexpr (HasWorldView<PlannerT>) {
        if (!view_) {
            FINENAV_LOG_ERROR(pnode_->get_logger(),
                "WorldView not attached — call attachWorldView() or createLayerByWorldView() first.");
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

    // ── Reference trajectory (read-only view) ──────────────────────────────
    // The upstream reference arrives as a const shared_ptr (zero-copy IPC), so it
    // cannot be moved out of.  Convert it to the planner's InputTrajectoryT only
    // when a *new* reference arrives (copy-on-change), then hand the planner a
    // const view — a planner that must mutate/retain it copies it itself.
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> ref_holder;
    {
        std::lock_guard<std::mutex> lock(plan_ref_mutex_);
        ref_holder = plan_ref_msg_;  // copy the shared_ptr; the only step needing the lock
    }
    const bool has_ref = static_cast<bool>(ref_holder);
    if (has_ref && ref_holder.get() != cached_ref_src_) {
        fromROSMsg(ref_holder->trajectory, cached_ref_);
        cached_ref_src_ = ref_holder.get();
    }

    // A pose-reading planner derives its goal from the reference trajectory
    // (`ref_traj.poses.back()`), so a missing reference means "no goal yet".
    if constexpr (PlannerT::InputProfile.pose) {
        if (!has_ref) {
            FINENAV_LOG_WARN(pnode_->get_logger(),
                "planOnce: no reference trajectory received yet (layer '{}').",
                layer_name_);
            return std::nullopt;
        }
    }

    param_keeper_.pollAll(); // Apply pending dynamic parameter updates

    bool plan_success = false;
    finenav_interfaces::msg::Trajectory output_traj_msg;

    const finenav_interfaces::msg::Trajectory empty_ref;  // fallback when no reference yet
    PrePlanContext pre_ctx{
        planner_name_, current_robot_state,
        has_ref ? ref_holder->trajectory : empty_ref};

    if (on_pre_plan_ && !on_pre_plan_(pre_ctx)) {
        return std::nullopt;
    }

    PlanningContext<PlannerT> ctx{current_robot_state, cached_ref_};

    // Acquire read locks on every map this view reads, held until plan() returns.
    std::vector<std::shared_ptr<void>> map_locks;
    map_locks.reserve(map_resources_.size());
    for (const auto& res : map_resources_) {
        map_locks.push_back(res->lockRead());
    }

    // WorldView heavy-derivation hook: once per plan, inside the map read lock
    // and before plan().  This is where inflation / terrain analysis / cost-field
    // construction belongs; plan() must only read what prepare() left behind.
    if (prepare_) prepare_(std::addressof(ctx));

    auto result = [&]() -> std::optional<OutputTrajectoryT<PlannerT>> {
        if constexpr (HasWorldView<PlannerT>) {
            // view_ is type-erased (already targeting the PlannerT::IWorldView base
            // subobject); recover the typed pointer here.  This branch is only
            // compiled for map-reading planners.
            auto typed_view = std::static_pointer_cast<typename PlannerT::IWorldView>(view_);
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
