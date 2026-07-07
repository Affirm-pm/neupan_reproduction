// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace finenav::core {

/**
 * @brief SingleShotPolicy — runs plan() exactly once per action goal.
 *
 * Used for one-shot planners (A*, BFS, route search) where a single call
 * produces the complete result.  Has no tunable parameters.
 */
struct SingleShotPolicy {
    static void declare_parameters(rclcpp::Node&) {}
};

/**
 * @brief ContinuousPolicy — runs plan() in a loop at a fixed frequency.
 *
 * Used for continuous-output planners (MPPI, D*-lite) that need warm-start
 * state across cycles.  The loop continues until the action is canceled or
 * an external state monitor signals goal_reached.
 *
 * @param frequency_hz  Control loop rate (Hz), default 10.0.
 */
struct ContinuousPolicy {
    double frequency_hz = 10.0;

    void declare_parameters(rclcpp::Node& node) {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description =
            "Control loop frequency for ContinuousPolicy (Hz). Must be > 0.0.";

        node.declare_parameter<double>("frequency_hz", frequency_hz, desc);
        frequency_hz = node.get_parameter("frequency_hz").as_double();

        if (frequency_hz <= 0.0) {
            FINENAV_LOG_WARN(node.get_logger(),
                "frequency_hz must be > 0.0 (got %.1f). Using default 10.0.",
                frequency_hz);
            frequency_hz = 10.0;
        }
    }
};

} // namespace finenav::core
