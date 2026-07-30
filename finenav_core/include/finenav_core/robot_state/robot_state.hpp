// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <algorithm>
#include <deque>
#include <mutex>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

namespace finenav::core {

/**
 *  @brief RobotState — the canonical robot state representation in FineNav-Engine.
 */
struct RobotState {
    Eigen::Vector3d    position{0, 0, 0};                ///< world-frame position (m)
    Eigen::Quaterniond orientation{1, 0, 0, 0};          ///< world ← body rotation
    Eigen::Vector3d    linear_velocity{0, 0, 0};         ///< body-frame linear velocity (m/s)
    Eigen::Vector3d    angular_velocity{0, 0, 0};        ///< body-frame angular velocity (rad/s)
};

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
        return history_.back().state;
    }

    /**
     * @brief Return the interpolated robot state at timestamp @p t.
     * @return std::nullopt if the history is empty, @p t is in the future, or @p t is
     *         older than the retained window.
     */
    [[nodiscard]] virtual std::optional<RobotState> getRobotState(const rclcpp::Time & t) const
    {
        std::lock_guard<std::mutex> lk(history_mutex_);
        if (history_.empty()) {
            return std::nullopt;
        }
        if (t > history_.back().stamp || t < history_.front().stamp) {
            return std::nullopt;  // outside the retained window
        }

        // First entry with stamp >= t.
        auto it = std::lower_bound(
            history_.begin(), history_.end(), t,
            [](const StampedState & s, const rclcpp::Time & q) { return s.stamp < q; });

        if (it->stamp == t) {
            return it->state;
        }

        const StampedState & s2 = *it;
        const StampedState & s1 = *(it - 1);
        const double span = (s2.stamp - s1.stamp).seconds();
        if (span <= 0.0) {
            return s1.state;  // degenerate: identical stamps
        }
        const double alpha = (t - s1.stamp).seconds() / span;
        return interpolate(s1.state, s2.state, alpha);
    }

protected:
    /**
     * @brief Append the latest state; prunes samples older than the window.
     * @note Out-of-order pushes (stamp < newest) are silently ignored.
     */
    void pushState(const rclcpp::Time & stamp, const RobotState & s)
    {
        std::lock_guard<std::mutex> lk(history_mutex_);
        if (!history_.empty() && stamp < history_.back().stamp) {
            return;
        }
        history_.push_back({stamp, s});
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
            prune(history_.back().stamp);
        }
    }

private:
    struct StampedState {
        rclcpp::Time stamp;
        RobotState   state;
    };

    /// Prune entries older than (newest − window).  Must hold history_mutex_.
    void prune(const rclcpp::Time & newest)
    {
        if (history_duration_sec_ <= 0.0) {
            while (history_.size() > 1) history_.pop_front();
            return;
        }
        const rclcpp::Time cutoff = newest - rclcpp::Duration::from_seconds(history_duration_sec_);
        while (history_.size() > 1 && history_.front().stamp < cutoff) {
            history_.pop_front();
        }
    }

    static RobotState interpolate(const RobotState & a, const RobotState & b, double alpha)
    {
        RobotState out;
        out.position         = a.position         + alpha * (b.position         - a.position);
        out.linear_velocity  = a.linear_velocity  + alpha * (b.linear_velocity  - a.linear_velocity);
        out.angular_velocity = a.angular_velocity + alpha * (b.angular_velocity - a.angular_velocity);
        out.orientation      = a.orientation.slerp(alpha, b.orientation);
        return out;
    }

    double                   history_duration_sec_;
    mutable std::mutex       history_mutex_;
    std::deque<StampedState> history_;
};

}  // namespace finenav::core
