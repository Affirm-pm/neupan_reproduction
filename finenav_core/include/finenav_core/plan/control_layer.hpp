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
#include "finenav_core/task_bus/task_bus.hpp"
#include "rclcpp/rclcpp.hpp"

namespace finenav::core {

/**
 * @brief ControlLayer — a single-planner pipeline stage driven by an
 *        ExecutionPolicyVariant.
 *
 * Inherits the plan-once pipeline (planner instance, planOnce(), WorldView
 * attachment) from PipelineNode and adds the execution policy, which decides
 * how the node is triggered and where its output goes:
 *   - PeriodicPolicy     → clock-driven: plan() loop at N Hz
 *   - EventDrivenPolicy  → event-driven: plan() once per new goal / upstream
 *     reference while STARTed
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
        ExecutionPolicyVariant policy = EventDrivenPolicy{},
        RobotStateProvider* state_provider = nullptr);

    ~ControlLayer() override;

protected:
    void onPlanRef(std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg) override;

private:
    // ── construction: one policy → one plumbing set + one episode body ──────
    void startWorker(std::function<void()> episode_body);   // both policies
    bool waitForState();

    // ── episode bodies ──────────────────────────────────────────────────────
    void runPeriodic();
    void runEventDriven();
    void react();

    ExecutionPolicyVariant policy_;

    // commanded plumbing (shared by both policies); worker_ itself is
    // inherited from PipelineNode.
    std::thread        worker_thread_;
    std::atomic<bool>  stop_{false};

    // event gating: true while the layer is STARTed and listening
    std::atomic<bool>  started_{false};
};

} // namespace finenav::core

#include "finenav_core/plan/detail/control_layer_impl.hpp"
