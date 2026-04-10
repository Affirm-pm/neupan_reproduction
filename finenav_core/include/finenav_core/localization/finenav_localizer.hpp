// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <tuple>
#include <vector>

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
    int         pose_smoothing_steps  = 1;      ///< Pose R scale factor (≥ 1)
    int         twist_smoothing_steps = 1;      ///< Twist R scale factor (≥ 1)
    double      pose_gate_dist        = 12.59;  ///< Mahalanobis gate  χ²(6) 95 %
    double      twist_gate_dist       = 12.59;  ///< Mahalanobis gate  χ²(6) 95 %
    double      max_dt_sec            = 0.5;    ///< dt clamp upper-bound [s]
    std::size_t max_obs_queue_size    = 64;  ///< Combined observation queue capacity
    NavigationModelD::Options model_opts{};     ///< ESKF process-noise parameters
};

// ============================================================================
//  FineNavLocalizer  —  timer-driven ESKF localizer (pure C++ API)
//
//  Internal threading model
//  ─────────────────────────────────────────────────────────────────────────
//  A private sub-node (pnode_) runs inside a dedicated NodeThread.
//  timerCallback() is the *sole* owner of eskf_ and state_history_.
//
//  setInitialPose() may be called from any external thread; it uses
//  filter_mutex_ to serialise access with timerCallback().
//
//  feedPose() / feedTwist() are fully lock-free relative to the filter —
//  they only touch the ObservationQueue's internal mutex.
//
//  getState() / getCovariance() / getLastUpdateTime() read from a separate
//  output cache protected by cached_state_mutex_ (shared_mutex), so
//  multiple readers never block each other.
//
    //  Sequential Retrocorrection
    //  ─────────────────────────────────────────────────────────────────────
    //  All pose and twist observations drained in one timer tick are merged
    //  and sorted by effective timestamp, then processed in a single pass:
    //    1. Roll eskf_ back to the snapshot just before the earliest obs.
    //    2. For each obs in chronological order:
    //         a. Micro-predict from cursor to obs.stamp (if positive).
    //         b. Apply the Kalman update (with Mahalanobis gate) to eskf_.
    //         c. Advance cursor to obs.stamp.
    //    3. Replay-predict from cursor to now.
    //  This guarantees that no observation overwrites a prior one's correction.
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

    // ── Public API ─────────────────────────────────────────────────────────

    /// Reset the filter to a known pose; clears all history.
    /// Must be called at least once before timerCallback() produces output.
    /// Thread-safe: may be called from any thread.
    void setInitialPose(const PoseMsg & msg);

    /// Push a pose observation (thread-safe, non-blocking).
    void feedPose(const PoseMsg & msg);

    /// Push a twist observation (thread-safe, non-blocking).
    void feedTwist(const TwistMsg & msg);

    /// Read the latest estimated state  (thread-safe).
    NavStateD    getState()          const;
    PMat         getCovariance()     const;
    rclcpp::Time getLastUpdateTime() const;

private:
    // ── Snapshot saved after predict(), before measurement updates ──────────
    struct StateSnapshot {
        rclcpp::Time stamp;
        NavStateD    state;
        PMat         covariance;
    };

    // ── Core update loop ────────────────────────────────────────────────────
    void timerCallback();

    // ── Per-observation Kalman update helpers ────────────────────────────────
    //  Precondition: filter_mutex_ held; eskf_ is already micro-predicted to
    //  the observation's effective timestamp by the sequential retrocorrection
    //  loop in timerCallback(). These helpers only apply the Kalman update.
    void applyPoseUpdate (const PoseMsg  & msg);
    void applyTwistUpdate(const TwistMsg & msg);

    /// Find the snapshot with the largest stamp ≤ target.
    /// Falls back to the oldest snapshot if none precedes target.
    /// NOTE: must be called while filter_mutex_ is held.
    std::optional<StateSnapshot> findSnapshot(const rclcpp::Time & target) const;

    // ── Message → ESKF observation conversion ───────────────────────────────
    static std::tuple<Vec3d, SO3d, Eigen::Matrix<double, 6, 6>>
    extractPoseObservation(const PoseMsg & msg);

    static std::tuple<Vec3d, Vec3d, Eigen::Matrix<double, 6, 6>>
    extractTwistObservation(const TwistMsg & msg);

    // ── ROS infrastructure ───────────────────────────────────────────────────
    rclcpp::Node::SharedPtr                    parent_node_;
    rclcpp::Node::SharedPtr                    pnode_;
    std::unique_ptr<NodeThread>                pnode_thread_;
    rclcpp::TimerBase::SharedPtr               update_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    // ── Filter state  (protected by filter_mutex_) ──────────────────────────
    mutable std::mutex          filter_mutex_;
    LocalizerOptions            opts_;
    ESKFd                       eskf_;
    std::deque<StateSnapshot>   state_history_;
    std::optional<rclcpp::Time> last_predict_time_;

    // ── Observation queue (unified — holds both Pose and Twist entries) ──────
    ObservationQueue            obs_queue_;

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

