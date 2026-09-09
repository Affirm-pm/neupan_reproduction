// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of control_layer.hpp.
// It contains the out-of-line template method implementations for ControlLayer.
// Do NOT include this file directly — include control_layer.hpp instead.

#pragma once

#include <chrono>
#include <thread>
#include <type_traits>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::core {

// ==============================================================================
// Construction
// ==============================================================================

template <Planner PlannerT>
ControlLayer<PlannerT>::ControlLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& algo_name,
    const std::string& layer_name,
    ExecutionPolicyVariant policy,
    RobotStateProvider* state_provider)
    : PipelineNode<PlannerT>(node, algo_name, layer_name, state_provider),
      policy_(std::move(policy))
{
    // 1) Parameters: each policy declares its own.
    std::visit([&pnode = this->pnode_](auto& p) {
        if constexpr (requires { p.declare_parameters(*pnode); }) {
            p.declare_parameters(*pnode);
        }
    }, policy_);

    // 2) Plumbing + episode body: one branch per policy (exhaustive).
    std::visit([this](auto& p) {
        using P = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<P, PeriodicPolicy>) {
            startWorker([this] { runPeriodic(); });
        } else if constexpr (std::is_same_v<P, EventDrivenPolicy>) {
            startWorker([this] { runEventDriven(); });
        }
    }, policy_);
}

template <Planner PlannerT>
ControlLayer<PlannerT>::~ControlLayer() {
    stop_ = true;
    if (worker_thread_.joinable()) worker_thread_.join();
}

// ==============================================================================
// Construction helpers
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::startWorker(std::function<void()> episode_body) {
    worker_thread_ = std::thread([this, episode_body = std::move(episode_body)]() {
        while (rclcpp::ok() && !stop_) {
            if (this->worker_->hasStart()) {
                this->worker_->ackStart();
                if (!waitForState()) {
                    this->worker_->publishFailed();
                    continue;
                }
                this->resetPlanner();
                episode_body();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

template <Planner PlannerT>
bool ControlLayer<PlannerT>::waitForState() {
    while (rclcpp::ok() && !this->worker_->shouldStop()) {
        if (this->state_provider_ && this->state_provider_->getRobotState().has_value()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

// ==============================================================================
// Commanded episode bodies
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::runPeriodic() {
    rclcpp::Rate rate(std::get<PeriodicPolicy>(policy_).frequency_hz);

    while (rclcpp::ok()) {
        if (this->worker_->shouldStop()) {
            this->worker_->publishFailed();
            return;
        }

        auto out = this->planOnce();
        if (!out) {
            FINENAV_LOG_ERROR(this->pnode_->get_logger(), "Periodic: plan() failed — aborting.");
            this->worker_->publishFailed();
            return;
        }
        this->worker_->publishRunning(std::move(*out));
        rate.sleep();
    }
}

// ==============================================================================
// Event-driven trigger
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::onPlanRef(
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg)
{
    this->storePlanRef(std::move(msg));

    std::visit([this](auto& p) {
        using P = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<P, EventDrivenPolicy>) {
            if (started_.load()) react();
        }
    }, policy_);
}

template <Planner PlannerT>
void ControlLayer<PlannerT>::runEventDriven() {
    // While STARTed, keep listening; the subscription callback (onPlanRef)
    // fires react() for every incoming reference.  STOP flips started_ off so
    // later references are only stashed until the next START.
    started_.store(true);
    while (rclcpp::ok() && !this->worker_->shouldStop()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    started_.store(false);
}

template <Planner PlannerT>
void ControlLayer<PlannerT>::react() {
    // An event-driven node reacts to the upstream reference trajectory alone
    // (the goal is its last pose).  Runs on the subscription callback thread.
    auto out = this->planOnce();
    if (!out) return;
    this->worker_->publishRunning(std::move(*out));
}

} // namespace finenav::core
