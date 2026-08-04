// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <algorithm>
#include <cmath>
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

    /**
     * @brief Return the latest available robot state.
     * @return std::nullopt if no state has been pushed yet.
     */
    [[nodiscard]] virtual std::optional<RobotState> getRobotState() const
    {
        std::lock_guard<std::mutex> lk(history_mutex_);
        if (history_.empty()) {
            return std::nullopt;
        }
        return history_.back();
    }

    /**
     * @brief Return the interpolated robot state at timestamp @p t.
     * @return std::nullopt if the history is empty, @p t is in the future, or @p t is
     *         older than the retained window.
     */
    [[nodiscard]] virtual std::optional<RobotState> getRobotState(const rclcpp::Time & t) const
    {
        const rclcpp::Time t_ros(t.nanoseconds(), RCL_ROS_TIME);  // normalize to internal clock type

        std::lock_guard<std::mutex> lk(history_mutex_);
        if (history_.empty()) {
            return std::nullopt;
        }
        if (t_ros > stampOf(history_.back()) || t_ros < stampOf(history_.front())) {
            return std::nullopt;  // outside the retained window
        }

        // First entry with stamp >= t_ros.
        auto it = std::lower_bound(
            history_.begin(), history_.end(), t_ros,
            [](const RobotState & s, const rclcpp::Time & q) { return stampOf(s) < q; });

        if (stampOf(*it) == t_ros) {
            return *it;
        }

        const RobotState & s2 = *it;
        const RobotState & s1 = *(it - 1);
        const double span = (stampOf(s2) - stampOf(s1)).seconds();
        if (span <= 0.0) {
            return s1;  // degenerate: identical stamps
        }
        const double alpha = (t_ros - stampOf(s1)).seconds() / span;
        return interpolate(s1, s2, alpha);
    }

protected:
    /**
     * @brief Append a robot state; prunes samples older than the window.
     * @note The timestamp is read from @p s.header.stamp.
     *       Out-of-order pushes (stamp < newest) are silently ignored.
     */
    void pushState(const RobotState & s)
    {
        const rclcpp::Time stamp = stampOf(s);
        std::lock_guard<std::mutex> lk(history_mutex_);
        if (!history_.empty() && stamp < stampOf(history_.back())) {
            return;
        }
        history_.push_back(s);
        prune(stamp);
    }

    /**
     * @brief Drop all history (e.g. on re-initialisation).
     */
    void resetHistory()
    {
        std::lock_guard<std::mutex> lk(history_mutex_);
        history_.clear();
    }

    void setHistoryDuration(double sec)
    {
        std::lock_guard<std::mutex> lk(history_mutex_);
        history_duration_sec_ = sec;
        if (!history_.empty()) {
            prune(stampOf(history_.back()));
        }
    }

private:
    /// Prune entries older than (newest − window).  Must hold history_mutex_.
    void prune(const rclcpp::Time & newest)
    {
        if (history_duration_sec_ <= 0.0) {
            while (history_.size() > 1) history_.pop_front();
            return;
        }
        const rclcpp::Time cutoff = newest - rclcpp::Duration::from_seconds(history_duration_sec_);
        while (history_.size() > 1 && stampOf(history_.front()) < cutoff) {
            history_.pop_front();
        }
    }

    static rclcpp::Time stampOf(const RobotState & s)
    {
        return rclcpp::Time(s.header.stamp.sec, s.header.stamp.nanosec, RCL_ROS_TIME);
    }

    static RobotState interpolate(const RobotState & a, const RobotState & b, double alpha)
    {
        RobotState out;
        out.header = a.header;

        out.pose.position.x = a.pose.position.x + alpha * (b.pose.position.x - a.pose.position.x);
        out.pose.position.y = a.pose.position.y + alpha * (b.pose.position.y - a.pose.position.y);
        out.pose.position.z = a.pose.position.z + alpha * (b.pose.position.z - a.pose.position.z);

        out.twist.linear.x = a.twist.linear.x + alpha * (b.twist.linear.x - a.twist.linear.x);
        out.twist.linear.y = a.twist.linear.y + alpha * (b.twist.linear.y - a.twist.linear.y);
        out.twist.linear.z = a.twist.linear.z + alpha * (b.twist.linear.z - a.twist.linear.z);
        out.twist.angular.x = a.twist.angular.x + alpha * (b.twist.angular.x - a.twist.angular.x);
        out.twist.angular.y = a.twist.angular.y + alpha * (b.twist.angular.y - a.twist.angular.y);
        out.twist.angular.z = a.twist.angular.z + alpha * (b.twist.angular.z - a.twist.angular.z);

        double qx = a.pose.orientation.x + alpha * (b.pose.orientation.x - a.pose.orientation.x);
        double qy = a.pose.orientation.y + alpha * (b.pose.orientation.y - a.pose.orientation.y);
        double qz = a.pose.orientation.z + alpha * (b.pose.orientation.z - a.pose.orientation.z);
        double qw = a.pose.orientation.w + alpha * (b.pose.orientation.w - a.pose.orientation.w);
        double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        out.pose.orientation.x = qx / norm;
        out.pose.orientation.y = qy / norm;
        out.pose.orientation.z = qz / norm;
        out.pose.orientation.w = qw / norm;

        return out;
    }

    double                   history_duration_sec_;
    mutable std::mutex       history_mutex_;
    std::deque<RobotState>   history_;
};

}  // namespace finenav::core
