// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <atomic>
#include <mutex>
#include <thread>
#include <variant>

#include "rclcpp/rclcpp.hpp"
#include "finenav_interfaces/msg/trajectory.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/generate_map_views_variant.hpp"
#include "finenav_core/plan/ros_conversions.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_core/param_handle_keeper.hpp"
#include "finenav_core/plan/i_control_layer.hpp"
#include "finenav_util/ros/task_channel.hpp"
#include "finenav_util/ros/node_thread.hpp"

namespace finenav::core {

/**
 * @brief ControlLayer — binds a set of planners to a MapServer under an execution policy.
 *
 * Each ControlLayer runs one planner at a time (selected at runtime by active_planner_id).
 * It owns the planner instances, manages the ROS action server, and orchestrates the
 * plan-once flow: pre-callback → read-lock MapView → planner.plan() → post-callback.
 *
 * @tparam Planners  The planner types supported by this layer (one or more).
 */
template <typename... Planners>
class ControlLayer : public IControlLayer {
public:
    // ── Types ──────────────────────────────────────────────────────────────

    using MapViewsVariant = GenerateMapViewsVariant<Planners...>;
    using PrePlanCallback = std::function<bool(PrePlanContext&)>;
    using PostPlanCallback = std::function<bool(PostPlanContext&)>;
    using ExecutionPolicyVariant = std::variant<SingleShotPolicy, ContinuousPolicy>;

    // ── Construction ───────────────────────────────────────────────────────

    explicit ControlLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& layer_name,
        ExecutionPolicyVariant policy = SingleShotPolicy{},
        RobotStateProvider* state_provider = nullptr);

    ~ControlLayer();

    // ── IControlLayer ─────────────────────────────────────────────────────

    void connectFrom(const std::string& parent_topic) override;

    // ── Configuration ──────────────────────────────────────────────────────

    void setInputTopic(const std::string& topic);
    void setOutputTopic(const std::string& topic);
    void setupDataPorts();

    /**
     * @brief Bind a MapServer to this layer so planners can read the map.
     *
     * At every plan() call the framework acquires a read-lock from @p server,
     * passes the locked const MapT& to @p builder together with the currently
     * active planner name, and holds the lock until plan() returns via
     * BoundMapView::lock_guard.
     *
     * @param builder  (const MapT&, const std::string& active_planner_id) → MapViewsVariant
     * @tparam MapT      Must satisfy SpatialContainer.
     */
    template <typename MapT, typename BuilderFn>
    void bind_map_view(std::shared_ptr<MapServer<MapT>> server, BuilderFn&& builder);

    void set_on_pre_plan(PrePlanCallback cb)   { on_pre_plan_  = std::move(cb); }
    void set_on_post_plan(PostPlanCallback cb) { on_post_plan_ = std::move(cb); }

    const std::string& get_active_planner_id() const { return active_planner_id_; }

private:
    // ── Internal types ─────────────────────────────────────────────────────

    struct BoundMapView {
        MapViewsVariant        view_variant;
        std::shared_ptr<void>  lock_guard;
    };

    using MapViewFactory = std::function<BoundMapView(const std::string& active_planner_id)>;

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
    std::unique_ptr<finenav::util::TaskWorker> worker_;
    std::thread worker_thread_;
    std::atomic<bool> stop_execution_{false};

    rclcpp::Subscription<finenav_interfaces::msg::TaskFeedback>::SharedPtr plan_ref_sub_;
    rclcpp::Publisher<finenav_interfaces::msg::Trajectory>::SharedPtr      plan_result_pub_;

    // ── Planner state ──────────────────────────────────────────────────────

    std::tuple<Planners...> planners_instance_;
    std::string               active_planner_id_;
    ExecutionPolicyVariant   policy_;

    // ── Callbacks ──────────────────────────────────────────────────────────

    MapViewFactory              map_view_factory_;
    PrePlanCallback             on_pre_plan_;
    PostPlanCallback            on_post_plan_;
    RobotStateProvider*        state_provider_{nullptr};

    // ── Data ───────────────────────────────────────────────────────────────

    std::string layer_name_;
    std::string input_topic_;
    std::string output_topic_;

    std::mutex                       plan_ref_mutex_;
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> plan_ref_msg_;

    ParamHandleKeeper param_keeper_;
};

} // namespace finenav::core

#include "finenav_core/plan/control_layer_impl.hpp"
