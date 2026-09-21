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
                    this->clearFaultedPlanRef();
                    if (this->worker_->shouldStop()) this->worker_->publishStopped();
                    else this->worker_->publishFaulted();
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
bool ControlLayer<PlannerT>::waitForState() {   // TODO: 为什么要 Wait?
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
            this->clearFaultedPlanRef();
            this->worker_->publishStopped();
            return;
        }

        switch (this->planRefState()) {
            case PlanRefState::kUpstreamFaulted:
                FINENAV_LOG_WARN(this->pnode_->get_logger(),
                    "Periodic layer '{}': upstream faulted — propagating fault.", this->layer_name_);
                this->clearFaultedPlanRef();
                this->worker_->publishFaulted();
                return;
            case PlanRefState::kNoInput:
                rate.sleep();
                continue;
            case PlanRefState::kValid:
                break;
        }

        auto out = this->planOnce();
        if (!out) {
            FINENAV_LOG_ERROR(this->pnode_->get_logger(), "Periodic: plan() faulted — aborting.");
            this->clearFaultedPlanRef();
            this->worker_->publishFaulted();
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
    // Both policies only stash the reference here and flag a pending event; the
    // EventDriven episode body consumes the flag on the worker thread.  (Periodic
    // ignores the flag and plans on its own clock.)
    this->storePlanRef(std::move(msg));
    ref_pending_.store(true, std::memory_order_release);
}

template <Planner PlannerT>
void ControlLayer<PlannerT>::runEventDriven() {
    // Event-driven = one plan() per new upstream reference, executed on the
    // worker thread.  The subscription callback (onPlanRef) only stashes the ref
    // and sets ref_pending_; this loop consumes it.  Planning here — not on the
    // callback thread — keeps the planner single-threaded with resetPlanner() and
    // makes FAULTED terminal (the body returns outright, like runPeriodic).
    while (rclcpp::ok() && !this->worker_->shouldStop()) {
        if (!ref_pending_.exchange(false, std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        switch (this->planRefState()) {
            case PlanRefState::kUpstreamFaulted:
                FINENAV_LOG_WARN(this->pnode_->get_logger(),
                    "EventDriven layer '{}': upstream faulted — propagating fault.", this->layer_name_);
                this->clearFaultedPlanRef();
                this->worker_->publishFaulted();
                return;
            case PlanRefState::kNoInput:
                continue;   // event consumed but no usable reference — wait for the next
            case PlanRefState::kValid:
                break;
        }

        auto out = this->planOnce();
        if (!out) {
            FINENAV_LOG_ERROR(this->pnode_->get_logger(),
                "EventDriven layer '{}': plan() faulted — aborting.", this->layer_name_);
            this->clearFaultedPlanRef();
            this->worker_->publishFaulted();
            return;
        }
        this->worker_->publishRunning(std::move(*out));
    }
    this->clearFaultedPlanRef();
    this->worker_->publishStopped();
}

} // namespace finenav::core
