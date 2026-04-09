// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <algorithm>
#include <deque>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace finenav {

// ============================================================================
//  ObservationQueue<MsgT>
//
//  Thread-safe timestamped observation buffer.
//
//  MsgT must expose a `header.stamp` field (builtin_interfaces::msg::Time),
//  which is standard for all stamped ROS 2 messages.
//
//  smoothing_steps is stored here as a parameter, but consumed externally by
//  FineNavLocalizer when applying observations:
//      R_eff = R_obs * smoothing_steps
//  N=1 → standard ESKF; larger N → filter output changes more slowly.
// ============================================================================
template <typename MsgT>
class ObservationQueue {
public:
    struct Entry {
        rclcpp::Time stamp;
        MsgT         msg;
    };

    explicit ObservationQueue(std::size_t max_queue_size = 32,
                              int         smoothing_steps = 1)
        : max_queue_size_(max_queue_size)
        , smoothing_steps_(smoothing_steps)
    {}

    /// Push a message; timestamp is extracted from msg.header.stamp.
    void push(const MsgT & msg)
    {
        rclcpp::Time stamp{msg.header.stamp};
        std::lock_guard<std::mutex> lk(mutex_);
        if (queue_.size() >= max_queue_size_) {
            RCLCPP_WARN(
                rclcpp::get_logger("ObservationQueue"),
                "[ObservationQueue] Capacity (%zu) exceeded — dropping oldest entry.",
                max_queue_size_);
            queue_.pop_front();
        }
        queue_.push_back({stamp, msg});
    }

    /// Atomically drain all entries; the returned vector is sorted by stamp
    /// (ascending) so FineNavLocalizer can apply them in chronological order.
    std::vector<Entry> drainAll()
    {
        std::vector<Entry> out;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            out.assign(queue_.begin(), queue_.end());
            queue_.clear();
        }
        std::sort(out.begin(), out.end(),
                  [](const Entry & a, const Entry & b) { return a.stamp < b.stamp; });
        return out;
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return queue_.size();
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return queue_.empty();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        queue_.clear();
    }

    int  smoothingSteps()      const noexcept { return smoothing_steps_; }
    void setSmoothingSteps(int n) noexcept   { smoothing_steps_ = n; }

private:
    std::size_t        max_queue_size_;
    int                smoothing_steps_;
    mutable std::mutex mutex_;
    std::deque<Entry>  queue_;
};

}  // namespace finenav

