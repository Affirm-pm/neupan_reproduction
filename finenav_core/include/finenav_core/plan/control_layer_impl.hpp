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
        if constexpr (std::is_same_v<P, SingleShotPolicy>) {
            startWorker([this] { runSingleShot(); });
        } else if constexpr (std::is_same_v<P, ContinuousPolicy>) {
            startWorker([this] { runContinuous(); });
        } else if constexpr (std::is_same_v<P, ReactivePolicy>) {
            startReactive();
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
    worker_ = std::make_unique<finenav::util::TaskWorker>(this->pnode_, this->layer_name_);
    worker_thread_ = std::thread([this, episode_body = std::move(episode_body)]() {
        while (rclcpp::ok() && !stop_) {
            if (worker_->hasStart()) {
                worker_->ackStart();
                if (!waitForState()) {
                    worker_->publishFailed();
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
void ControlLayer<PlannerT>::startReactive() {
    feedback_pub_ = this->pnode_->template create_publisher<finenav_interfaces::msg::TaskFeedback>(
        finenav::util::TaskChannel::feedbackTopic(this->layer_name_), 10);
}

template <Planner PlannerT>
bool ControlLayer<PlannerT>::waitForState() {
    while (rclcpp::ok() && !worker_->shouldStop()) {
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
void ControlLayer<PlannerT>::runSingleShot() {
    if (worker_->shouldStop()) {
        worker_->publishFailed();
        return;
    }

    auto out = this->planOnce();
    if (!out) {
        FINENAV_LOG_ERROR(this->pnode_->get_logger(), "SingleShot: plan() failed.");
        worker_->publishFailed();
        return;
    }

    worker_->publishRunning(std::move(*out));
    worker_->publishCompleted({});
    FINENAV_LOG_INFO(this->pnode_->get_logger(), "SingleShot: goal succeeded.");
}

template <Planner PlannerT>
void ControlLayer<PlannerT>::runContinuous() {
    rclcpp::Rate rate(std::get<ContinuousPolicy>(policy_).frequency_hz);

    while (rclcpp::ok()) {
        if (worker_->shouldStop()) {
            worker_->publishFailed();
            return;
        }

        auto out = this->planOnce();
        if (!out) {
            FINENAV_LOG_ERROR(this->pnode_->get_logger(), "Tracking: plan() failed — aborting.");
            worker_->publishFailed();
            return;
        }
        worker_->publishRunning(std::move(*out));
        rate.sleep();
    }
}

// ==============================================================================
// Reactive trigger
// ==============================================================================

template <Planner PlannerT>
void ControlLayer<PlannerT>::onPlanRef(
    std::shared_ptr<const finenav_interfaces::msg::TaskFeedback> msg)
{
    this->storePlanRef(std::move(msg));

    std::visit([this](auto& p) {
        using P = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<P, ReactivePolicy>) {
            react();
        }
    }, policy_);
}

template <Planner PlannerT>
void ControlLayer<PlannerT>::react() {
    // A reactive node is data-driven: the upstream reference trajectory alone
    // drives planning (the goal is its last pose).
    auto out = this->planOnce();
    if (!out) return;

    auto fb = std::make_unique<finenav_interfaces::msg::TaskFeedback>();
    fb->status     = finenav_interfaces::msg::TaskFeedback::RUNNING;
    fb->trajectory = std::move(*out);
    feedback_pub_->publish(std::move(fb));
}

} // namespace finenav::core
