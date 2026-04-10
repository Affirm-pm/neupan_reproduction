// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include <algorithm>
#include <deque>
#include <mutex>
#include <variant>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
namespace finenav {
// ============================================================================
//  ObservationQueue  —  single chronological buffer for all localizer inputs
//
//  Stores both PoseWithCovarianceStamped and TwistWithCovarianceStamped in
//  one deque via std::variant.  drainAll() returns entries sorted ascending
//  by stamp so FineNavLocalizer::timerCallback() can feed them directly into
//  the sequential retrocorrection loop — no separate merge step needed.
//
//  Thread-safety: all public methods are guarded by an internal mutex.
//  push* calls are non-blocking relative to the filter (the queue has its
//  own separate mutex so feedPose/feedTwist never contend with timerCallback).
// ============================================================================
class ObservationQueue {
public:
    using PoseMsg  = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;
    struct Entry {
        rclcpp::Time                    stamp;
        std::variant<PoseMsg, TwistMsg> data;
    };
    explicit ObservationQueue(std::size_t max_size = 64)
        : max_size_(max_size)
    {}
    // ── Push ────────────────────────────────────────────────────────────────
    void pushPose(const PoseMsg & msg)
    {
        push({rclcpp::Time(msg.header.stamp), msg});
    }
    void pushTwist(const TwistMsg & msg)
    {
        push({rclcpp::Time(msg.header.stamp), msg});
    }
    // ── Drain ───────────────────────────────────────────────────────────────
    /// Atomically move all entries out of the queue.
    /// Returns a vector sorted ascending by stamp.
    /// stable_sort preserves push order for equal-stamp entries
    /// (a pose pushed before a twist at the same stamp stays first).
    std::vector<Entry> drainAll()
    {
        std::vector<Entry> out;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            out.assign(std::make_move_iterator(queue_.begin()),
                       std::make_move_iterator(queue_.end()));
            queue_.clear();
        }
        return out;
    }
    // ── Introspection ───────────────────────────────────────────────────────
    std::size_t size()  const { std::lock_guard<std::mutex> lk(mutex_); return queue_.size(); }
    bool        empty() const { std::lock_guard<std::mutex> lk(mutex_); return queue_.empty(); }
    void        clear()       { std::lock_guard<std::mutex> lk(mutex_); queue_.clear(); }
private:
    void push(Entry e)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (queue_.size() >= max_size_) {
            RCLCPP_WARN(rclcpp::get_logger("ObservationQueue"),
                "[ObservationQueue] Capacity (%zu) exceeded — dropping oldest entry.",
                max_size_);
            queue_.pop_front();
        }
        queue_.push_back(std::move(e));
    }
    std::size_t        max_size_;
    mutable std::mutex mutex_;
    std::deque<Entry>  queue_;
};
}  // namespace finenav
