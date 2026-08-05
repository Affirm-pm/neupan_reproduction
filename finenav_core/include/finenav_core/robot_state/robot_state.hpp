// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include "finenav_interfaces/msg/robot_state.hpp"

namespace finenav::core {

using RobotState = finenav_interfaces::msg::RobotState;

/**
 * @brief RobotStateProvider — pull-based robot-state source with a rolling history.
 * @details
 * Support interpolated state at a past timestamp;
 * A derived localizer only computes its state each cycle and calls the protected `pushState()`
 * @note All public methods are thread-safe.
 */
class RobotStateProvider {
public:
    /// @param history_duration_sec  Length of the retained history window [s]
    ///                              (≤ 0 keeps only the latest sample).
    explicit RobotStateProvider(double history_duration_sec = 2.0)
        : history_duration_sec_(history_duration_sec) {}

    virtual ~RobotStateProvider() = default;

    [[nodiscard]] virtual std::optional<RobotState> getRobotState() const;
    [[nodiscard]] virtual std::optional<RobotState> getRobotState(const rclcpp::Time & t) const;

protected:
    void pushState(const RobotState & s);
    void resetHistory();
    void setHistoryDuration(double sec);

private:
    void prune(const rclcpp::Time & newest);
    static rclcpp::Time stampOf(const RobotState & s);
    static RobotState interpolate(const RobotState & a, const RobotState & b, double alpha);

    double                   history_duration_sec_;
    mutable std::mutex       history_mutex_;
    std::deque<RobotState>   history_;
};

}  // namespace finenav::core
