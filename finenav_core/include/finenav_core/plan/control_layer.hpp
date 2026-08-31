// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <variant>

#include "../common/param_pollable_keeper.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/i_control_layer.hpp"
#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_interfaces/msg/trajectory.hpp"
#include "finenav_util/ros/node_thread.hpp"
#include "finenav_util/ros/task_channel.hpp"
#include "rclcpp/rclcpp.hpp"

namespace finenav::core {

template <Map MapT> class MapServer;  // full definition in map_server.hpp

/**
 * @brief ControlLayer — binds a single planner to a MapServer under an execution policy.
 *
 * Each ControlLayer owns exactly one planner instance and runs it under the configured
 * execution policy. It manages the ROS action server and orchestrates the plan-once flow:
 * pre-callback → read-lock MapView → planner.plan() → post-callback.
 *
 * @tparam PlannerT  The planner type owned by this layer.
 */
template <Planner PlannerT>
class ControlLayer : public IControlLayer {
public:
    // ── Types ──────────────────────────────────────────────────────────────

    using MapViewPtr = std::shared_ptr<typename PlannerT::IMapView>;
    using PrePlanCallback = std::function<bool(PrePlanContext&)>;
    using PostPlanCallback = std::function<bool(PostPlanContext&)>;

    // ── Construction ───────────────────────────────────────────────────────

    explicit ControlLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& algo_name,
        const std::string& layer_name,
        ExecutionPolicyVariant policy = SingleShotPolicy{},
        RobotStateProvider* state_provider = nullptr);

    ~ControlLayer();

    // ── IControlLayer ─────────────────────────────────────────────────────

    void connectFrom(const std::string& parent_topic) override;

    // ── Configuration ──────────────────────────────────────────────────────

    void setInputTopic(const std::string& topic);
    void setupDataPorts();

    /**
     * @brief Bind a MapServer to this layer so the planner can read the map.
     *
     * At every plan() call the framework acquires a read-lock from @p server,
     * passes the locked const MapT& to @p builder, and holds the lock until
     * plan() returns via BoundMapView::lock_guard.
     *
     * @param builder  (const MapT&) → MapViewPtr
     * @tparam MapT      Must satisfy Map.
     */
    template <typename MapT, typename BuilderFn>
    void bind_map_view(std::shared_ptr<MapServer<MapT>> server, BuilderFn&& builder);

    void set_on_pre_plan(PrePlanCallback cb)   { on_pre_plan_  = std::move(cb); }
    void set_on_post_plan(PostPlanCallback cb) { on_post_plan_ = std::move(cb); }

private:
    // ── Internal types ─────────────────────────────────────────────────────

    struct BoundMapView {
        MapViewPtr            view;
        std::shared_ptr<void> lock_guard;
    };

    using MapViewFactory = std::function<BoundMapView()>;

    // ── Methods ────────────────────────────────────────────────────────────

    void PlanRefCallback(std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg);

    /// Episode entry-point invoked by the ROS action server.

    /**
     * @brief Entry point of a planning episode
     */
    void execute();

    /// Policy dispatch.
    void executeImpl(SingleShotPolicy& policy);
    void executeImpl(ContinuousPolicy&   policy);

    /// Reset the planner at an episode boundary.
    void resetPlanner();

    /**
     * @brief Run one planning step: pre-cb → read-lock MapView → plan() → post-cb.
     * @return true on success.
     */
    bool planOnce(geometry_msgs::msg::Pose goal_pose);

    // ── ROS infrastructure ─────────────────────────────────────────────────

    rclcpp::Node::SharedPtr      node_;
    rclcpp::Node::SharedPtr      pnode_;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<finenav::util::TaskWorker> worker_;
    std::thread worker_thread_;
    std::atomic<bool> stop_execution_{false};

    rclcpp::Subscription<finenav_interfaces::msg::TaskFeedback>::SharedPtr plan_ref_sub_;

    // ── Planner state ──────────────────────────────────────────────────────

    PlannerT                planner_instance_;
    std::string             planner_name_;
    ExecutionPolicyVariant  policy_;

    // ── Callbacks ──────────────────────────────────────────────────────────

    MapViewFactory              map_view_factory_;
    PrePlanCallback             on_pre_plan_;
    PostPlanCallback            on_post_plan_;
    RobotStateProvider*        state_provider_{nullptr};

    // ── Data ───────────────────────────────────────────────────────────────

    std::string layer_name_;
    std::string input_topic_;

    std::mutex                       plan_ref_mutex_;
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> plan_ref_msg_;

    ParamPollableKeeper param_keeper_;
};

} // namespace finenav::core

#include "finenav_core/plan/control_layer_impl.hpp"
