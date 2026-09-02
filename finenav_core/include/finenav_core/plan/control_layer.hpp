// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <variant>

#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/pipeline_node.hpp"
#include "finenav_util/ros/task_channel.hpp"
#include "rclcpp/rclcpp.hpp"

namespace finenav::core {

/**
 * @brief ControlLayer — a single-planner pipeline stage driven by an
 *        ExecutionPolicyVariant.
 *
 * Inherits the plan-once pipeline (planner instance, planOnce(), MapView
 * attachment) from PipelineNode and adds the execution policy, which decides
 * how the node is triggered and where its output goes:
 *   - SingleShotPolicy  → commanded: one plan() per goal
 *   - ContinuousPolicy  → commanded: plan() loop at N Hz
 *   - ReactivePolicy    → data-driven: plan() on every upstream reference
 *
 * @tparam PlannerT  The planner type owned by this layer.
 */
template <Planner PlannerT>
class ControlLayer : public PipelineNode<PlannerT> {
public:
    explicit ControlLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& algo_name,
        const std::string& layer_name,
        ExecutionPolicyVariant policy = SingleShotPolicy{},
        RobotStateProvider* state_provider = nullptr);

    ~ControlLayer() override;

protected:
    void onPlanRef(std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg) override;

private:
    // ── construction: one policy → one plumbing set + one episode body ──────
    void startWorker(std::function<void()> episode_body);   // Single / Continuous
    void startReactive();                                   // Reactive
    bool waitForState();

    // ── episode bodies ──────────────────────────────────────────────────────
    void runSingleShot();
    void runContinuous();
    void react();

    ExecutionPolicyVariant policy_;

    // commanded plumbing (Single / Continuous)
    std::unique_ptr<finenav::util::TaskWorker> worker_;
    std::thread        worker_thread_;
    std::atomic<bool>  stop_{false};

    // reactive plumbing (Reactive)
    rclcpp::Publisher<finenav_interfaces::msg::TaskFeedback>::SharedPtr feedback_pub_;
};

} // namespace finenav::core

#include "finenav_core/plan/control_layer_impl.hpp"
