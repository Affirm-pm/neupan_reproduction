// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/common/param_pollable_keeper.hpp"
#include "finenav_core/map/i_map_resource.hpp"
#include "finenav_core/plan/i_pipeline_node.hpp"
#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_core/task_bus/task_channel.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"
#include "finenav_interfaces/msg/trajectory.hpp"
#include "finenav_util/ros/node_thread.hpp"

namespace finenav::core {

/**
 * @brief PipelineNode — the shared core of a single-planner pipeline stage.
 *
 * Holds the planner instance and the whole plan-once pipeline:
 *   pre-callback → read-lock MapView → planner.plan() → post-callback → result.
 *
 * It deliberately knows nothing about *how* a plan is triggered or *where* the
 * result is published — that is the single dimension the two concrete node types
 * differ on:
 *   - ControlLayer  (commanded): a TaskWorker feeds it goals, an episode loop
 *     calls planOnce() at policy rate and publishes through the TaskChannel.
 *   - DecoratorNode (reactive): the upstream feedback callback fires planOnce()
 *     immediately and publishes straight to its feedback topic.
 *
 * @tparam PlannerT  The planner type owned by this node.
 */
template <Planner PlannerT>
class PipelineNode : public IPipelineNode {
public:
    using PrePlanCallback  = std::function<bool(PrePlanContext&)>;
    using PostPlanCallback = std::function<bool(PostPlanContext&)>;

    // ── IPipelineNode ─────────────────────────────────────────────────────

    void connectFrom(const std::string& parent_name) override;

    /// IPipelineNode — attach a pre-built MapView plus its backing map resources.
    void attachMapView(
        std::shared_ptr<void> view,
        std::vector<std::shared_ptr<IMapResource>> maps) override;

    // ── Configuration ──────────────────────────────────────────────────────

    void set_on_pre_plan(PrePlanCallback cb)   { on_pre_plan_  = std::move(cb); }
    void set_on_post_plan(PostPlanCallback cb) { on_post_plan_ = std::move(cb); }

protected:
    PipelineNode(
        rclcpp::Node::SharedPtr node,
        const std::string& algo_name,
        const std::string& layer_name,
        RobotStateProvider* state_provider);

    ~PipelineNode() override;

    /// React to an upstream reference-trajectory message.  Default (commanded)
    /// behaviour is to stash it for the next planOnce(); the reactive subclass
    /// overrides this to fire a plan immediately.
    virtual void onPlanRef(std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg);

    /// Stash the latest upstream reference (thread-safe).
    void storePlanRef(std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg);

    /// Reset the planner at an episode boundary.
    void resetPlanner();

    /**
     * @brief Run one planning step: pre-cb → read-lock MapView → plan() → post-cb.
     *
     * The goal is read from the stashed reference trajectory
     * (`ref_traj.poses.back()`), so no goal argument is needed.
     * @return The output trajectory on success, nullopt otherwise.
     */
    std::optional<finenav_interfaces::msg::Trajectory> planOnce();

    // ── ROS infrastructure ─────────────────────────────────────────────────

    rclcpp::Node::SharedPtr      node_;
    rclcpp::Node::SharedPtr      pnode_;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread_;

    // ── Planner state ──────────────────────────────────────────────────────

    PlannerT      planner_instance_;
    std::string   planner_name_;

    // ── Map view + callbacks ───────────────────────────────────────────────

    std::shared_ptr<void>       view_;              ///< type-erased IMapView (map-reading planners only)
    std::vector<std::shared_ptr<IMapResource>> map_resources_;  ///< maps to read-lock per plan
    PrePlanCallback             on_pre_plan_;
    PostPlanCallback            on_post_plan_;
    RobotStateProvider*         state_provider_{nullptr};

    // ── Data ───────────────────────────────────────────────────────────────

    std::string layer_name_;

    /// The TaskChannel worker for this stage: owns all three channel endpoints
    /// (cmd_sub_, feedback_pub_ and, after connectFrom(), ref_sub_).
    std::unique_ptr<finenav::core::TaskWorker> worker_;

    std::mutex                       plan_ref_mutex_;
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> plan_ref_msg_;

    /// Converted reference (msg → planner InputTrajectoryT), rebuilt only when a
    /// new upstream reference arrives.  planOnce() hands the planner a const view
    /// into this, so the per-tick deep copy of the reference is eliminated.
    InputTrajectoryT<PlannerT> cached_ref_;
    const void*                cached_ref_src_{nullptr};  ///< identity of plan_ref_msg_ at last conversion

    ParamPollableKeeper param_keeper_;
};

} // namespace finenav::core

#include "finenav_core/plan/pipeline_node_impl.hpp"
