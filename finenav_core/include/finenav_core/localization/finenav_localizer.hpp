// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "finenav_util/node_thread.hpp"
#include "finenav_core/localization/eskf.hpp"
#include "finenav_core/localization/filter_model.hpp"
#include "finenav_core/localization/observation_queue.hpp"

namespace finenav {

// ============================================================================
//  LocalizerOptions  —  all tunable parameters for FineNavLocalizer
// ============================================================================
struct LocalizerOptions {
    double      update_rate_hz        = 100.0;  ///< Timer frequency [Hz]
    double      max_delay_sec         = 0.5;    ///< Max tolerable observation delay [s]
    int         pose_smoothing_steps  = 1;      ///< Pose processed N times, R scaled ×N
    int         twist_smoothing_steps = 1;      ///< Twist processed N times, R scaled ×N
    double      pose_gate_dist        = 12.59;  ///< Mahalanobis gate  χ²(6) 95 %
    double      twist_gate_dist       = 12.59;  ///< Mahalanobis gate  χ²(6) 95 %
    double      max_dt_sec            = 0.5;    ///< dt clamp upper-bound [s]
    std::size_t max_obs_queue_size    = 64;     ///< Per-type queue capacity
    NavigationModelD::Options model_opts{};     ///< ESKF process-noise parameters
};

// ============================================================================
//  FineNavLocalizer  —  timer-driven ESKF localizer with time-delay compensation
//
//  Internal threading model
//  ─────────────────────────────────────────────────────────────────────────
//  A private sub-node (pnode_) runs inside a dedicated NodeThread.
//  timerCallback() is the sole writer of eskf_.
//
//  setInitialPose() may be called from any external thread; it uses
//  filter_mutex_ to serialise access with timerCallback().
//
//  feedObservation() pushes into pose_queue_ / twist_queue_, which are each
//  mutex-protected internally; the call never contends with timerCallback().
//
//  getState() / getCovariance() / getLastUpdateTime() read from an output
//  cache protected by cached_state_mutex_ (shared_mutex).
//
//  Time-delay handling
//  ─────────────────────────────────────────────────────────────────────────
//  Delayed observations are handled by the ESKF's TimeDelayKalmanFilter
//  (state-expansion approach, following autoware EKFModule).
//  Every timerCallback():
//    1. predict(dt)  — slides delay slots, accumulates elapsed time
//    2. For each observation popped from the queue:
//         a. Compute delay_time = now − stamp (clamped to ≥ 0)
//         b. delay_step = eskf_.findDelayStep(delay_time)
//         c. Reject if delay_step ≥ maxDelayStep (too old)
//         d. applyUpdate(msg, delay_step)  →  eskf_.update<6>(..., delay_step)
//
//  Smoothing (AgedObservationQueue)
//  ─────────────────────────────────────────────────────────────────────────
//  Each observation is processed smoothing_steps times (via pop_increment_age).
//  Each processing uses R_eff = R_msg * smoothing_steps, distributing the
//  correction gradually and preventing sharp jumps.
// ============================================================================
class FineNavLocalizer {
public:
    using PMat     = NavigationModelD::PMat;
    using Vec3d    = NavigationModelD::Vec3T;
    using SO3d     = NavigationModelD::SO3T;
    using PoseMsg  = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;

    /// @param parent_node  Owning node (used for clock & namespace).
    /// @param opts         Localizer configuration.
    explicit FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                              LocalizerOptions        opts = {});

    FineNavLocalizer(const FineNavLocalizer &)             = delete;
    FineNavLocalizer & operator=(const FineNavLocalizer &) = delete;
    ~FineNavLocalizer() = default;

    /**
     * @brief Reset the filter to the given pose.  Clears all delay-slot history.
     * @note  Thread-safe.
     */
    void setInitialPose(const PoseMsg & msg);

    /**
     * @brief Enqueue a pose or twist observation for the next timer tick.
     * @note  Thread-safe.
     */
    void feedObservation(const PoseMsg  & msg);
    void feedObservation(const TwistMsg & msg);

    /**
     * @brief Read-only access to the latest filter output.
     * @note  Thread-safe.
     */
    NavStateD    getState()          const;
    PMat         getCovariance()     const;
    rclcpp::Time getLastUpdateTime() const;

private:

    // ── Timer callback ───────────────────────────────────────────────────────
    void timerCallback();

    // ── Observation update (called inside timerCallback, single-threaded) ────
    void applyUpdate(const PoseMsg  & msg, int delay_step);
    void applyUpdate(const TwistMsg & msg, int delay_step);

    // ── Message extraction helpers ───────────────────────────────────────────
    static std::tuple<Vec3d, SO3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const PoseMsg & msg);

    static std::tuple<Vec3d, Vec3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const TwistMsg & msg);

    // ── ROS infrastructure ───────────────────────────────────────────────────
    rclcpp::Node::SharedPtr                        parent_node_;
    rclcpp::Node::SharedPtr                        pnode_;
    std::unique_ptr<NodeThread>                    pnode_thread_;
    rclcpp::TimerBase::SharedPtr                   update_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    // ── Filter state  (protected by filter_mutex_) ───────────────────────────
    mutable std::mutex          filter_mutex_;
    LocalizerOptions            opts_;
    ESKFd                       eskf_;
    std::optional<rclcpp::Time> last_predict_time_;

    // ── Observation queues (each internally mutex-protected) ─────────────────
    PoseObsQueue  pose_queue_;
    TwistObsQueue twist_queue_;

    // ── Initialisation guard ─────────────────────────────────────────────────
    std::atomic<bool> is_initialized_{false};

    // ── Output cache  (protected by cached_state_mutex_) ────────────────────
    mutable std::shared_mutex cached_state_mutex_;
    NavStateD    cached_state_{};
    PMat         cached_covariance_ = PMat::Identity() * 1e-2;
    rclcpp::Time cached_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Logger logger_;
};

}  // namespace finenav

