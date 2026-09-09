// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <variant>

#include <rclcpp/rclcpp.hpp>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::core {

/**
 * @brief PeriodicPolicy — runs plan() in a loop at a fixed frequency.
 *
 * Used for fixed-rate output planners (MPPI) that need warm-start state across
 * cycles.  The loop continues until the action is canceled or an external
 * state monitor signals goal_reached.
 *
 * @param frequency_hz  Control loop rate (Hz), default 10.0.
 */
struct PeriodicPolicy {
    double frequency_hz = 10.0;

    void declare_parameters(rclcpp::Node& node) {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description =
            "Control loop frequency for PeriodicPolicy (Hz). Must be > 0.0.";

        node.declare_parameter<double>("frequency_hz", frequency_hz, desc);
        frequency_hz = node.get_parameter("frequency_hz").as_double();

        if (frequency_hz <= 0.0) {
            FINENAV_LOG_WARN(node.get_logger(),
                "frequency_hz must be > 0.0 (got {:.1f}). Using default 10.0.",
                frequency_hz);
            frequency_hz = 10.0;
        }
    }
};

/**
 * @brief EventDrivenPolicy — event-triggered listening transform.
 *
 * Controlled by TaskCmd START/STOP like the other policy.  While STARTed it
 * listens for a new upstream input (a goal or an upstream reference trajectory
 * on the __feedback subscription) and reacts immediately — one planOnce() +
 * RUNNING publish — for every incoming event; while STOPped it only stashes the
 * reference.  Used by map-less trajectory decorators (seek / smooth / trim)
 * wired as intermediate pipeline stages, and as the general server default for
 * one-shot route planners that replan on each new goal.
 */
struct EventDrivenPolicy {
    static void declare_parameters(rclcpp::Node&) {}
};

using ExecutionPolicyVariant =
    std::variant<PeriodicPolicy, EventDrivenPolicy>;

} // namespace finenav::core
