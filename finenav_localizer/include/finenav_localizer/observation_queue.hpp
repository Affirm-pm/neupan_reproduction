// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.


#pragma once

#include <cstddef>
#include <mutex>
#include <queue>

#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::localizer {

/**
 * @brief AgedObservationQueue<MsgT>  —  Observation FIFO with smoothing mechanism
 * @details
 *      Smoothing mechanism: each push() enters the message with age = 0. pop_increment_age() returns the front message and:
 *      if age + 1 < max_age → re-enqueues the message with age + 1 ;
 *      if age + 1 >= max_age → discards (processed max_age times).
 *      Example:
 * @code
 *      const std::size_t n = queue.size();
 *      for (std::size_t i = 0; i < n; ++i) {
 *          auto msg = queue.pop_increment_age();  // may re-enqueue at back
 *          // ... process msg ...
 *      }
 * @endcode
 * @tparam MsgT message type stored in the queue.
 */
template <typename MsgT>
class AgedObservationQueue {
public:
    /// @param smoothing_steps  Number of times each observation is processed (≥ 1).
    ///                         Corresponds to AgedObjectQueue::max_age.
    /// @param max_queue_size   Hard capacity cap (prevents unbounded growth).
    /**
     * @brief
     * @param smoothing_steps  Number of times each observation is processed (≥ 1). Corresponds to AgedObjectQueue::max_age.
     * @param max_queue_size   Hard capacity cap.
     */
    explicit AgedObservationQueue(std::size_t smoothing_steps,
                                  std::size_t max_queue_size = 64)
        : max_age_       (smoothing_steps > 0u ? smoothing_steps : 1u)
        , max_queue_size_(max_queue_size > max_age_ ? max_queue_size : max_age_)
    {}

    /**
     * @brief push message to observation buffer
     * @param msg message
     */
    void push(const MsgT & msg)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (objects_.size() >= max_queue_size_) {
            FINENAV_LOG_WARN_ONCE(rclcpp::get_logger("finenav.AgedObservationQueue"),
                "[AgedObservationQueue] capacity ({}) exceeded — dropping oldest entry.",
                max_queue_size_);
            objects_.pop();
            ages_.pop();
        }
        objects_.push(msg);
        ages_.push(0u);
    }


    /**
     * @brief Pop with age increment.
     * @details If the resulting age < max_age_, re-enqueue a copy at the back.
     * @return Poped message
     */
    MsgT pop_increment_age()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        const MsgT        msg = objects_.front();
        const std::size_t age = ages_.front() + 1u;
        objects_.pop();
        ages_.pop();
        if (age < max_age_) {
            objects_.push(msg);
            ages_.push(age);
        }
        return msg;
    }


    [[nodiscard]] std::size_t size() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return objects_.size();
    }

    [[nodiscard]] bool empty() const { return size() == 0u; }

    void clear()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        objects_ = std::queue<MsgT>{};
        ages_    = std::queue<std::size_t>{};
    }

    [[nodiscard]] std::size_t maxAge()       const { return max_age_; }
    [[nodiscard]] std::size_t maxQueueSize() const { return max_queue_size_; }

private:
    const std::size_t       max_age_;
    const std::size_t       max_queue_size_;
    mutable std::mutex      mutex_;
    std::queue<MsgT>        objects_;
    std::queue<std::size_t> ages_;
};

// ── Concrete type aliases ─────────────────────────────────────────────────────
using PoseObsQueue  = AgedObservationQueue<geometry_msgs::msg::PoseWithCovarianceStamped>;
using TwistObsQueue = AgedObservationQueue<geometry_msgs::msg::TwistWithCovarianceStamped>;

/**
 * @brief RelativePoseObs  —  odometry measurement spanning two epochs.
 * @details
 *      Stores the raw body-frame displacement from two consecutive odometry frames,
 *      plus both timestamps so the ESKF maps each epoch to its own delay slot
 *      independently.  No state snapshot is stored — the two nominal states are
 *      retrieved from the ESKF inside applyUpdate() using the delay steps computed
 *      from stamp_from and stamp_to.
 */
struct RelativePoseObs {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d             delta_p_body_obs;  ///< body-frame translation from odometry (m)
    Sophus::SO3d                delta_R_obs;       ///< relative rotation from odometry
    Eigen::Matrix<double, 6, 6> R_noise;           ///< displacement covariance (m², rad²)
    rclcpp::Time                stamp_to;          ///< timestamp of the "to"   (newer) frame
    rclcpp::Time                stamp_from;        ///< timestamp of the "from" (older) frame
};

using RelativePoseObsQueue = AgedObservationQueue<RelativePoseObs>;

}  // namespace finenav::localizer
