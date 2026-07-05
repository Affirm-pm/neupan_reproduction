// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "finenav_interfaces/action/compute_plan.hpp"
#include "finenav_interfaces/msg/trajectory.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/planner_set.hpp"
#include "finenav_core/plan/ros_conversions.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_core/param_handle_keeper.hpp"
#include "finenav_util/ros/simple_action_server.hpp"
#include "finenav_util/ros/node_thread.hpp"

namespace finenav::core {

/**
 * @brief ControlLayer — binds a set of planners to a MapServer under an execution policy.
 *
 * Each ControlLayer runs one planner at a time (selected at runtime by active_planner_id).
 * It owns the planner instances, manages the ROS action server, and orchestrates the
 * plan-once flow: pre-callback → read-lock MapView → planner.plan() → post-callback.
 *
 * @tparam PlannersT       PlannerSet<...> listing the supported planner types.
 * @tparam ExecutionPolicy  SingleShotPolicy or TrackingPolicy.
 */
template <typename PlannersT, typename ExecutionPolicy = SingleShotPolicy>
class ControlLayer {
public:
    // ── Types ──────────────────────────────────────────────────────────────

    using MapViewsVariant = GenerateMapViewsVariant<PlannersT>;
    using ActionT         = finenav_interfaces::action::ComputePlan;
    using ActionServer    = finenav::util::SimpleActionServer<ActionT>;
    using PrePlanCallback = std::function<bool(PrePlanContext&)>;
    using PostPlanCallback = std::function<bool(PostPlanContext&)>;
    using RobotStateProvider = std::function<RobotState()>;

    // ── Construction ───────────────────────────────────────────────────────

    explicit ControlLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& layer_name,
        ExecutionPolicy policy = ExecutionPolicy{});

    // ── Configuration ──────────────────────────────────────────────────────

    void setInputTopic(const std::string& topic);
    void setOutputTopic(const std::string& topic);
    std::string getName() const;
    void setupDataPorts();

    /**
     * @brief Bind a MapServer to this layer so planners can read the map.
     *
     * At every plan() call the framework acquires a read-lock from @p server,
     * passes the locked const MapT& to @p builder, and holds the lock until
     * plan() returns via BoundMapView::lock_guard.
     *
     * @tparam MapT      Must satisfy SpatialContainer.
     * @tparam BuilderFn (const MapT&) → MapViewsVariant-compatible
     */
    template <SpatialContainer MapT, typename BuilderFn>
    requires std::is_invocable_v<BuilderFn, const MapT&> &&
             std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&>, MapViewsVariant>
    void bind_map_view(std::shared_ptr<MapServer<MapT>> server, BuilderFn&& builder);

    void set_on_pre_plan(PrePlanCallback cb)               { on_pre_plan_           = std::move(cb); }
    void set_on_post_plan(PostPlanCallback cb)             { on_post_plan_          = std::move(cb); }
    void set_robot_state_provider(RobotStateProvider prov) { robot_state_provider_  = std::move(prov); }

    const std::string& get_active_planner_id() const { return active_planner_id_; }

private:
    // ── Internal types ─────────────────────────────────────────────────────

    using MapViewFactory = std::function<BoundMapView()>;

    /**
     * @brief Map view + RAII read-lock.  lock_guard holds a shared_ptr to the
     * LockedMapRO so that the read-lock is released when this struct is destroyed.
     */
    struct BoundMapView {
        MapViewsVariant        view_variant;
        std::shared_ptr<void>  lock_guard;
    };

    // ── Methods ────────────────────────────────────────────────────────────

    void PlanRefCallback(const finenav_interfaces::msg::Trajectory& msg);

    /// Episode entry-point invoked by the ROS action server.

    /**
     * @brief Entry point of a planning episode
     */
    void execute();

    /// Policy dispatch.
    /** @brief Policy dispatch */
    void executeImpl(SingleShotPolicy& policy);
    void executeImpl(TrackingPolicy&   policy);

    /// Reset each planner whose AlgoConfigurator name matches active_planner_id_.
    void resetActivePlanner();

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
    std::unique_ptr<ActionServer> action_server_;

    rclcpp::Subscription<finenav_interfaces::msg::Trajectory>::SharedPtr plan_ref_sub_;
    rclcpp::Publisher<finenav_interfaces::msg::Trajectory>::SharedPtr    plan_result_pub_;

    // ── Planner state ──────────────────────────────────────────────────────

    typename PlannersT::Tuple planners_instance_;
    std::string               active_planner_id_;
    ExecutionPolicy           policy_;

    // ── Callbacks ──────────────────────────────────────────────────────────

    MapViewFactory      map_view_factory_;
    PrePlanCallback     on_pre_plan_;
    PostPlanCallback    on_post_plan_;
    RobotStateProvider  robot_state_provider_;

    // ── Data ───────────────────────────────────────────────────────────────

    std::string layer_name_;
    std::string input_topic_;
    std::string output_topic_;

    std::mutex                       plan_ref_mutex_;
    finenav_interfaces::msg::Trajectory plan_ref_msg_;

    ParamHandleKeeper param_keeper_;
};

} // namespace finenav::core

#include "finenav_core/plan/control_layer_impl.hpp"
