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

#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "eskf.hpp"
#include "finenav_localizer/filter_model.hpp"
#include "finenav_localizer/observation_queue.hpp"

namespace finenav::localizer {

// ============================================================================
//  EskfLocalizer  —  ROS-free ESKF localization engine (the C++ API layer)
//
//  This is the inner half of the localizer.  It owns the estimator and the full
//  synchronous C++ API (setInitialPose / feedObservation / feedRelativePose /
//  step / getters) but NO ROS runtime — no node, timer, executor, TF broadcaster
//  or parameter listener.  Time is injected into step(); parameters arrive as a
//  plain Config.  Message *types* (geometry_msgs) are used as the observation
//  API but require no ROS runtime, so this class constructs and runs with no
//  rclcpp::init() — making it directly driveable from benchmarks and
//  deterministic unit tests (like nav2 MPPI's computeVelocityCommands()).
//
//  The ROS wrapper (FineNavLocalizer) owns the node/timer/params/TF and drives
//  this engine: it maps Params → Config, calls step(pnode_->now()) each tick,
//  broadcasts TF, and (future work) subscribes to input topics.
//
//  Threading: setInitialPose() / feedObservation() / feed* are safe to call from
//  external threads; step() is the sole writer of the estimator.  Mutexes guard
//  the estimator (filter_mutex_) and the output cache (cached_state_mutex_).
// ============================================================================
class EskfLocalizer {
public:
    using PMat     = NavigationModelD::PMat;
    using Vec3d    = NavigationModelD::Vec3T;
    using SO3d     = NavigationModelD::SO3T;
    using PoseMsg  = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;

    /// Plain configuration (mirrors finenav_localizer.yaml minus publish_tf,
    /// which is a ROS-only concern handled by the outer wrapper).
    struct Config {
        bool    passthrough              = false;
        double  update_rate_hz           = 100.0;
        double  max_delay_sec            = 0.5;
        int64_t pose_smoothing_steps     = 1;
        int64_t twist_smoothing_steps    = 1;
        int64_t rel_pose_smoothing_steps = 1;
        double  pose_gate_dist           = 1e9;
        double  twist_gate_dist          = 1e9;
        double  rel_pose_gate_dist       = 1e9;
        double  max_dt_sec               = 0.5;
        int64_t max_obs_queue_size       = 64;
        int64_t max_rel_pose_queue_size  = 64;
        bool    force_2d_mode            = false;
        double  vel_noise_std            = 0.10;
        double  omega_noise_std          = 0.10;
    };

    EskfLocalizer() : EskfLocalizer(Config{}) {}
    explicit EskfLocalizer(const Config & config);

    EskfLocalizer(const EskfLocalizer &)             = delete;
    EskfLocalizer & operator=(const EskfLocalizer &) = delete;

    /// Live-update the tunables (gates, smoothing, passthrough, force_2d, …).
    /// Does not rebuild the estimator; mirrors the previous per-tick param read.
    void setConfig(const Config & config);
    [[nodiscard]] const Config & getConfig() const { return config_; }

    // ── C++ API: inputs (msg-based) ──────────────────────────────────────────
    /// Reset the filter to the given pose.  Clears all delay-slot history.  Thread-safe.
    void setInitialPose(const PoseMsg & msg);
    /// Enqueue a pose / twist / relative-pose observation.  Thread-safe.
    void feedObservation(const PoseMsg  & msg);
    void feedObservation(const TwistMsg & msg);
    void feedRelativePose(const RelativePoseObs & obs);

    // ── The tick: predict + drain queues + update + cache (no TF, no logging clock) ─
    /// Advance the estimator by one cycle using @p now as the reference epoch.
    /// This is the unit driven directly by the timer, benchmarks and tests.
    void step(const rclcpp::Time & now);

    // ── C++ API: outputs ─────────────────────────────────────────────────────
    /// Latest state, `v` in the **body frame** (contract of the previous getState()).
    [[nodiscard]] NavStateD    getState()          const;
    /// Raw latest nominal state (world-frame v, no force_2d shaping) — for TF.
    [[nodiscard]] NavStateD    getRawState()       const;
    [[nodiscard]] PMat         getCovariance()     const;
    [[nodiscard]] rclcpp::Time getLastUpdateTime() const;
    [[nodiscard]] bool isInitialized() const { return is_initialized_.load(std::memory_order_acquire); }

private:
    // ── Build the estimator / queues from a Config ──────────────────────────
    void configure(const Config & config);

    // ── Observation update (called inside step, single-threaded) ─────────────
    void applyUpdate(const PoseMsg         & msg, int delay_step);
    void applyUpdate(const TwistMsg        & msg, int delay_step);
    void applyUpdate(const RelativePoseObs & obs, int delay_step_to, int delay_step_from);

    // ── Message extraction helpers ───────────────────────────────────────────
    static std::tuple<Vec3d, SO3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const PoseMsg & msg);
    static std::tuple<Vec3d, Vec3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const TwistMsg & msg);

    // ── 2D output masking ────────────────────────────────────────────────────
    NavStateD force2DOutput(const NavStateD & s) const;

    // ── Parameters ───────────────────────────────────────────────────────────
    Config config_;

    // ── Filter state (protected by filter_mutex_) ────────────────────────────
    mutable std::mutex          filter_mutex_;
    ESKFd                       eskf_;
    std::optional<rclcpp::Time> last_predict_time_;

    // ── Observation queues (each internally mutex-protected) ─────────────────
    std::unique_ptr<PoseObsQueue>         pose_queue_;
    std::unique_ptr<TwistObsQueue>        twist_queue_;
    std::unique_ptr<RelativePoseObsQueue> rel_pose_queue_;

    // ── Initialisation guard ─────────────────────────────────────────────────
    std::atomic<bool> is_initialized_{false};

    // ── Output cache (protected by cached_state_mutex_) ──────────────────────
    mutable std::shared_mutex cached_state_mutex_;
    NavStateD    cached_state_{};
    PMat         cached_covariance_ = PMat::Identity() * 1e-2;
    rclcpp::Time cached_time_{0, 0, RCL_ROS_TIME};

    // ── Diagnostics ──────────────────────────────────────────────────────────
    rclcpp::Logger logger_;
};

}  // namespace finenav::localizer
