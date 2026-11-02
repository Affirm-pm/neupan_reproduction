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
    /// @note The window must cover the latency between a consumer's measurement
    ///       stamp and its query (scan + processing + transport) plus the period
    ///       of the slowest back-dated source — a 0.5 Hz pose channel spends 2 s
    ///       per period alone.  A concrete provider sizes it for its own inputs
    ///       via setHistoryDuration().
    explicit RobotStateProvider(double history_duration_sec = 5.0)
        : history_duration_sec_(history_duration_sec) {}

    virtual ~RobotStateProvider() = default;

    [[nodiscard]] virtual std::optional<RobotState> getRobotState() const;
    [[nodiscard]] virtual std::optional<RobotState> getRobotState(const rclcpp::Time & t) const;

    /// @brief Stamp of the oldest retained state; nullopt when the history is empty.
    [[nodiscard]] std::optional<rclcpp::Time> oldestStamp() const;
    /// @brief Stamp of the newest retained state; nullopt when the history is empty.
    [[nodiscard]] std::optional<rclcpp::Time> newestStamp() const;

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
